# policies/ — training and sim evaluation (LeRobot ACT, ACT + language, Diffusion Policy)

Uses LeRobot 0.7.0's own `ACTPolicy` / `DiffusionPolicy`, their processors (normalisation) and
optimizer presets; this directory only adds the robot's feature config, a fast in-memory data
path, a text encoder for the language variant, and the sim eval. Environment setup: see
`sim/README.md` (same venv, `sim/.venv`). Run from the repo root.

## Modules

| file | what |
|---|---|
| `factory.py` | `make_config(kind, features, image_size, ...)` for `act`, `act_lang`, `diffusion` (sizes `tiny` / `small` / `paper`); `make_policy` → policy + pre/post processors, optional custom backbone |
| `backbones.py` | `SmallCNN` (GroupNorm ResNet-style, stride 8/16) and `apply_backbone`, which swaps it into a LeRobot ACT / DP without forking; the spec is saved in the checkpoint meta |
| `state.py` | `state_mode`: `joints` (6) or `joints_ee` (joints + TCP xyz from FK, 9) |
| `data.py` | `ChunkDataset`: reads LeRobot datasets once (video decode), keeps frames in RAM, returns samples windowed by the policy's own `action_delta_indices` / `observation_delta_indices` (+ `action_is_pad`), ±N px shift and colour-jitter augmentation (shared across an observation window), derived-state stats |
| `text_encoder.py` | frozen instruction encoders: MiniLM-L6 (real) or hashed bag-of-words (**stub**) |
| `train.py` | training entry point (`--backbone`, `--state-mode`, `--color-jitter`, `--val-datasets`, `--init-checkpoint`, `--threads` default 2) |
| `eval_sim.py` | roll a checkpoint out in `sim.envs.TabletopEnv` (optionally domain-randomised), success with Wilson 95% CI, PLAN 6.6 failure categories, first-grasp xy offset, sort colour-mapping counts; strict weight loading |
| `diagnose.py` | open-loop first-action error of a checkpoint on training vs held-out expert episodes |
| `run_experiment.py` | train + eval → one row of a results JSONL; `scripts/wave2_queue*.sh` are the wave-2 runs |

Wave-2 experiments and conclusions: **[RESULTS.md](RESULTS.md)**.

## Train

```bash
# ACT baseline on one task
sim/.venv/bin/python -m policies.train --policy act \
    --datasets outputs/datasets/vla_arm_sim_pick_place \
    --steps 4000 --batch-size 16 --time-limit-min 38 --out outputs/train/act_pick_place

# language-conditioned ACT on all three tasks (PLAN 6.3)
sim/.venv/bin/python -m policies.train --policy act_lang --datasets \
    outputs/datasets/vla_arm_sim_pick_place outputs/datasets/vla_arm_sim_stack outputs/datasets/vla_arm_sim_sort \
    --out outputs/train/act_lang_all

# Diffusion Policy (DDIM, 10 inference steps, horizon 16, 2 obs frames)
sim/.venv/bin/python -m policies.train --policy diffusion --datasets ... --out outputs/train/dp
```

Output: `<out>/pretrained_model/` (LeRobot `config.json`, `model.safetensors`, processor JSON +
stats, and `vla_arm_meta.json` with the text-encoder backend), `train_log.csv`, `train_meta.json`.

Configs (`--size small`, default, CPU-trainable): ACT dim 256 / ffn 1024 / 4 enc / 1 dec layers /
4 VAE layers, chunk 50, temporal ensembling coeff 0.01 (query every step), lr 1e-4, KL weight 10,
ResNet-18 per camera → **18.7 M params**. `--size paper` restores LeRobot's defaults (dim 512,
ffn 3200) → 51.6 M params (PLAN 6.1 says ~80 M; the ResNet-18 ACT is ~52 M).
Diffusion small: U-Net dims (128, 256, 512), GroupNorm ResNet-18, random crop 84 of 96 → 40.6 M.

## Language conditioning (PLAN 6.3)

`act_lang` puts a frozen embedding of the instruction into ACT's `observation.environment_state`
input, which LeRobot's ACT projects into **one extra encoder token** — no fork of ACT. The
embedding is computed per batch from the dataset's `task` string (cached per string).

* `--text-encoder minilm`: `sentence-transformers/all-MiniLM-L6-v2` (384-d, runs on CPU). Needs
  `pip install sentence-transformers` and access to huggingface.co.
* `--text-encoder hashed_bow`: **STUB** — deterministic hashed bag of words + bigrams +
  position-tagged colour words, 256-d. No semantics: a held-out paraphrase only works as far as
  it shares words with training phrasings. It exists because huggingface.co is blocked from the
  build machine.
* `auto` (default) tries MiniLM, falls back to the stub with a warning; the checkpoint records
  which one was used, and `train_meta.json` has `text_encoder_is_stub`.

## Evaluate in sim

```bash
sim/.venv/bin/python -m policies.eval_sim --checkpoint outputs/train/act_pick_place/pretrained_model \
    --task pick_place --episodes 20 --out outputs/eval/act_pick_place.json [--video-dir outputs/eval/videos]
```

Seeds 900000… (never used for recording), same layouts for every policy. `--instructions heldout`
evaluates on held-out paraphrases. Prints e.g. `7/20 = 35% (Wilson 95% CI 18%-57%)`.

## Pipeline-proof run (build machine, 4 CPU cores)

ACT (`small`, 18.7 M params, random-init ResNet-18), 30 pick_place demos (5 333 frames, 96×96,
two cameras), batch 16, lr 1e-4, ±4 px shift augmentation, 4 000 steps in **33.2 min** (≈0.5 s/step
on 4 cores, data load 16 s).

| step | 1 | 400 | 800 | 1200 | 2000 | 2800 | 4000 |
|---|---|---|---|---|---|---|---|
| total loss (L1 + 10·KL) | 77.1 | 1.16 | 0.49 | 0.29 | 0.19 | 0.14 | 0.118 |
| L1 (normalised actions) | 0.85 | 0.28 | 0.21 | 0.17 | 0.13 | 0.14 | 0.108 |

Sim eval, 20 held-out layouts (seeds 900000–900019): **0/20 = 0% (Wilson 95% CI 0–16%)**.
On 10 *training* layouts (seeds 0–9): 4/10 = 40% (CI 17–69%). Open-loop on a training episode the
first action of each chunk is within ~0.02 rad of the expert on the arm joints, so the train/eval
data path is consistent; the policy reaches toward the red block but closes the gripper beside
it (missed grasp) and then hovers. That is overfitting to 30 demos with a backbone trained from
scratch, not a pipeline bug. Levers in rough order: ImageNet-pretrained backbone, more demos
(PLAN: 50/task), more steps / GPU, image-size 128+, lighting/texture randomisation.
Checkpoint: `outputs/train/act_pick_place/pretrained_model`; eval JSON + videos in `outputs/eval/`.

## Wave 2 (details in [RESULTS.md](RESULTS.md))

100 domain-randomised pick_place demos; eval on the same 20 held-out layouts **with** domain
randomisation (unseen lighting / colours / camera poses + action noise), 2 CPU threads, no
pretrained weights:

| run | success, 20 seeds | 40 seeds |
|---|---|---|
| E1 ACT, wave-1 recipe on the new data (3 000 steps, 38 min) | 2/20 = 10 % (3–30 %) | 5/40 = 12.5 % |
| E2 ACT + SmallCNN instead of ResNet-18 | 0/20 | – |
| E3 ACT + EE position in state | 1/20 = 5 % | – |
| E4 ACT tiny transformer, same budget (4 244 steps) | 5/20 = 25 % | 7/40 = 17.5 % |
| **E5 ACT, E1 trained 2× (6 000 steps, 2×37 min)** | **5/20 = 25 % (11–47 %)** | **10/40 = 25 % (14–40 %)** |
| DP1 / DP2 Diffusion Policy, 40 / 80 min | 0/20 / 0/20 | – |
| L1 ACT + hashed-BoW (stub) language, `sort`, seen / held-out paraphrases | 0/20 / 0/20 | – |

Best recipe: ACT small, random-init ResNet-18, 100 DR demos, ≥ 6 000 steps. The levers that
mattered were more (randomised) data and more optimisation; the from-scratch SmallCNN, EE state and
open-loop chunk execution did not help at this budget, and DP needs far more steps than CPU allows.
Nominal-scene eval: E1 4/20, E5 5/20 (wave 1: 0/20).

## Real vs stubbed

* Real: LeRobot ACT / Diffusion implementations, processors, optimizer presets, LeRobot v3.0
  datasets, checkpoints loadable with `ACTPolicy.from_pretrained` (except those with a custom
  `backbone` in `vla_arm_meta.json` — use `policies.eval_sim.load_policy`, which re-applies it and
  loads strictly; LeRobot's `from_pretrained` loads with `strict=False` and would silently skip the
  mismatched weights).
* Stubbed / limited: text encoder (hashed BoW unless MiniLM can be downloaded); vision backbones
  start from **random init** (ImageNet weights come from download.pytorch.org, which is blocked
  here — on the real data set `pretrained_backbone_weights="ResNet18_Weights.IMAGENET1K_V1"` in
  `factory.py`); "small" model sizes and few-thousand-step runs are for CPU only.
* Not yet: SmolVLA fine-tuning (LeRobot 0.7 ships `smolvla`; needs the HF checkpoint + a GPU),
  ONNX/TensorRT export, DP and ACT-lang runs long enough to succeed (wave-2 CPU runs: 0/20 each).
