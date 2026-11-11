# Wave-2 results: sim imitation learning without pretrained weights (CPU only)

Machine: 4 CPU cores shared with two other agents, no GPU; training used 2 threads
(`torch.set_num_threads(2)`), evals 1 thread. huggingface.co and download.pytorch.org are blocked, so
**every vision backbone starts from random init** and the language encoder is the hashed
bag-of-words **stub**. Datasets, checkpoints, logs and eval JSONs are in `outputs/wave2/` (not in git);
the exact commands are in `policies/scripts/wave2_queue{1,2,3,4}.sh`.

## Protocol

* **Data**: 100 scripted-expert `pick_place` episodes recorded with domain randomisation
  (`--randomize default`: lighting, colours inside each colour class, camera jitter, 4 mrad action
  noise; see `sim/README.md`), 96×96, two cameras, 17 798 frames (two halves of 50, seeds 0–49 and
  1000–1049). Validation: 10 more DR episodes (seeds 950 000+), used only for the val-loss curves.
* **Eval**: 20 held-out layouts, seeds 900 000–900 019 (never recorded), **with** DR (`default`
  preset: unseen lighting/colours/camera poses and action noise), 330-step limit; success =
  block in bowl and released. Same layouts for every row. Failure categories (PLAN 6.6) and the xy
  distance TCP→red block when the gripper first closes near the table are logged per episode.
  95 % intervals are Wilson. The best rows were re-run on 20 more seeds (900 020–900 039).
* **One variable at a time** relative to the row named in "vs". Each run was capped at ~40 min of
  training (E5 and DP2 are resumed runs, i.e. two capped segments).

## Results table — pick_place, held-out layouts, DR eval

| run | change (vs) | policy / backbone | steps | train min | val L1¹ | success 20 seeds (95 % CI) | 40 seeds (95 % CI) | failures (20 seeds) | median grasp offset |
|---|---|---|---|---|---|---|---|---|---|
| wave 1 | 30 nominal demos, nominal eval | ACT small / ResNet-18 | 4 000 | 33 | – | 0/20 (0–16 %) | – | missed grasp | – |
| E1 | 100 DR demos (wave 1) | ACT small / ResNet-18 | 3 000 | 38 | 0.165 | 2/20 = 10 % (3–30 %) | 5/40 = 12.5 % (5–26 %) | 17 missed, 1 dropped | 20 mm |
| E2 | SmallCNN stride 16 (E1) | ACT small / SmallCNN | 3 000 | 35 | 0.200 | 0/20 (0–16 %) | – | 20 missed | 215 mm |
| E3 | + EE position in state (E1) | ACT small / ResNet-18 | 3 000 | 35 | 0.173 | 1/20 = 5 % (1–24 %) | – | 19 missed | 99 mm |
| E4 | tiny transformer, same budget → more steps (E3) | ACT tiny / ResNet-18 | 4 244 | 38 | 0.146 | 5/20 = 25 % (11–47 %) | 7/40 = 17.5 % (9–32 %) | 15 missed | 14 mm |
| **E5** | **2× training, resumed (E1)** | ACT small / ResNet-18 | **6 000** | 38 + 35 | 0.154 | **5/20 = 25 % (11–47 %)** | **10/40 = 25 % (14–40 %)** | 13 missed, 2 dropped | 9 mm |
| DP1 | Diffusion Policy, E1 data/recipe | DP small / ResNet-18 (GN) | 2 638 | 40 | n/a² | 0/20 (0–16 %) | – | 20 missed | 100 mm |
| DP2 | DP1 resumed, 2× budget (≈ E5) | DP small / ResNet-18 (GN) | 5 216 | 40 + 40 | n/a² | 0/20 (0–16 %) | – | 20 missed | 118 mm |

¹ ACT inference-mode L1 on normalised 50-step action chunks (z = 0) over the 10 validation episodes.
² DP's validation loss is its noise-prediction MSE (0.042 at the end of DP1), not comparable with ACT's L1.

Other evals of the same checkpoints (not separate training runs):

| checkpoint | eval change | success |
|---|---|---|
| E1 | nominal scene (no DR, no action noise) | 4/20 = 20 % (8–42 %) |
| E5 | nominal scene | 5/20 = 25 % (11–47 %) |
| E1 | no temporal ensembling, execute 10 actions per chunk | 1/20 = 5 % (1–24 %) |
| wave-1 checkpoint | nominal, same seeds (from wave 1) | 0/20 (0–16 %) |

## Training curves (text summary; full CSVs in `outputs/wave2/runs/*/train_log.csv`)

* ACT (E1): total loss 78 → 0.93 (step 500) → 0.39 (1 000) → 0.21 (2 000) → 0.16 (3 000); KL falls
  to ~0.002, i.e. the CVAE latent is barely used. Validation L1: 0.28 (500) → 0.23 (1 000) → 0.20
  (1 500) → 0.18 (2 000) → 0.165 (3 000). Continuing to 6 000 steps (E5) only moved val L1 to 0.154,
  while training L1 kept falling (~0.12) — the open-loop curve flattens long before closed-loop
  success stops improving, so val L1 is a weak proxy for success here.
* SmallCNN (E2): training loss tracks E1 within ~15 % but val L1 stays higher (0.20 vs 0.165).
* Tiny ACT (E4): 0.55 s/step vs 0.75 s/step, 4 244 steps in the same 38 min; val L1 0.146 (best).
* DP1: noise-MSE 1.19 → 0.14 (500) → 0.063 (1 500) → 0.042 (2 500); still falling when the cap hit.
* Open-loop diagnosis (`python -m policies.diagnose`): first-action error of the wave-1 checkpoint
  on held-out layouts was 0.065 rad on base yaw (≈ 16 mm at 25 cm reach; the jaw clears a 30 mm
  block by only ~8 mm per side) vs 0.041 rad on its training layouts — a generalisation gap.
  For E1 (100 DR demos) train and held-out errors are equal (base yaw 0.049 vs 0.055 rad, arm mean
  0.037 vs 0.036 rad): the gap is gone and what remains is plain under-fitting. After 6 000 steps
  (E5) held-out base-yaw error is 0.044 rad and the arm mean 0.030 rad — the improvement that came
  with 25 % success.

## What worked

1. **More, randomised data** (wave 1 → E1): 0/20 → 4/20 on the nominal scene, and the
   train/held-out open-loop gap disappeared. Even under the DR eval (harder: unseen lighting,
   colours, camera poses, action noise) E1 gets 12.5 % over 40 seeds.
2. **More optimisation** is the strongest lever found: doubling E1's steps (E5) gave 25 % (10/40,
   CI 14–40 %), and the median first-grasp offset dropped from 20 mm to 9 mm. Getting more steps per
   wall-clock minute with a narrower transformer (E4) helped the same way on the first 20 seeds, but
   over 40 seeds (17.5 %) it is not distinguishable from E1 or E5.

## What did not work (at this budget)

* **SmallCNN (GroupNorm, stride 16, 2.4 M params) instead of random-init ResNet-18**: 0/20. The
  policy closes the gripper in mid-air and wanders; it never learned to descend onto the block in
  3 000 steps. The ResNet-18's BatchNorm probably makes it optimise faster from scratch at batch 16.
  Not retried at stride 8 (3× slower per step on CPU) or with longer training.
* **EE position in the state** (E3 vs E1): 1/20 vs 2/20 — no measurable effect (it is a
  deterministic function of the joints, so it adds no information; the block position, which is
  what the policy gets wrong, still has to come from the images).
* **Executing chunks open-loop** (10 steps, no temporal ensembling): no better than ensembling.
* **Diffusion Policy at 40 min (2 638 steps)**: 0/20. It closes the gripper early near the home pose
  and hovers; DP is known to need far more steps than ACT (Chi et al. train for thousands of epochs;
  2 638 × 16 samples is 2.4 epochs here). DP2 (resumed to 5 216 steps, the same total budget as
  E5) is still 0/20 with the same behaviour, although its noise-prediction MSE kept falling
  (0.042 → 0.018).
* **Not tested**: colour-jitter augmentation (implemented, `--color-jitter`; it was queued on the
  SmallCNN branch, which was dropped — DR in the data already varies colours and lighting), SmallCNN
  at stride 8 (3× slower per step) or with longer training, image size 128.

## ACT vs Diffusion Policy (same data, same eval)

| budget | ACT | DP |
|---|---|---|
| ~40 min (1 capped run) | E1 2/20 = 10 % (3–30 %) | DP1 0/20 (0–16 %) |
| ~75–80 min (2 capped runs) | E5 5/20 = 25 % (11–47 %) | DP2 0/20 (0–16 %) |

At CPU budgets ACT is clearly ahead: it reaches a usable policy in a few thousand steps, while DP
is still at the "learns the mean trajectory, not when to grasp" stage. This says nothing about the
asymptotic comparison PLAN 6.2 asks for; that needs a GPU and ~50–100× more steps.

## Language-conditioned ACT on `sort` (hashed bag-of-words STUB encoder)

**The text encoder is a stub**: `HashedBowEncoder` (deterministic hashed bag of words + bigrams +
position-tagged colour words, 256-d). It has no semantics; a held-out paraphrase is only "understood"
to the extent that it shares tokens with the training phrasings. MiniLM could not be downloaded.

* Data: 100 DR `sort` demos (36 648 frames, seeds 0–99; recorded in 66 min). In `sort` the
  colour→bowl assignment is random per episode and only stated in the instruction, so the policy
  must read it. Training uses the first 3 paraphrases; eval "seen" uses those, "held-out" the other
  2 (`the {a} block goes in the {A} bowl, ...`, `place {a} in {A} and {b} in {B}`).
* Run L1: `act_lang` (ACT small + one instruction token through `observation.environment_state`),
  ResNet-18 from scratch, joint state, 40-min cap → 3 266 steps (1.4 epochs of this larger
  dataset); training L1 was still 0.24 (pick_place runs end at ~0.15).
* Eval: 20 held-out layouts with DR, 600-step limit.

| instructions | success (95 % CI) | blocks in the instructed bowl | blocks in the other bowl |
|---|---|---|---|
| seen paraphrases | 0/20 (0–16 %) | 0 / 40 | 0 / 40 |
| held-out paraphrases | 0/20 (0–16 %) | 1 / 40 | 0 / 40 |

So the language experiment is **inconclusive**: the policy cannot yet do the two-pick
manipulation, so whether it follows the colour mapping cannot be measured (one block landed in the
correct bowl in 40 episodes). The pipeline (stub encoder → env-state token → eval on seen vs
held-out paraphrases with a colour-mapping metric) works end to end; it needs a policy that can pick
first (more steps, ideally GPU) and a real sentence encoder. The failure category column for this
run counts only the red block (fixed in `eval_sim.py` afterwards: in `sort` either block counts).

## Best recipe (CPU, no pretrained weights)

ACT `small` (dim 256, 4 enc / 1 dec / 4 VAE layers, chunk 50, temporal ensembling 0.01, KL 10, lr 1e-4),
random-init ResNet-18 per camera, two 96×96 cameras, joint-position state, ±4 px shift augmentation,
batch 16, **100 domain-randomised demos, ≥ 6 000 steps** (≈ 75 min on 2 CPU threads). Reproduce:
`policies/scripts/wave2_queue1.sh` (E1) then the E5 line of `wave2_queue3.sh`.

## Compute used

About 7.5 h wall clock (recording, 9 training segments, ~20 evals; training never ran two at a
time). Training: 5.2 h at 2 threads (E1 38, E2 35, E3-SmallCNN 11 (killed), E3 35, E4 38, E5 36,
DP1 40, DP2 40, L1 40 min) ≈ 10.5 CPU-h. Evals ≈ 17 runs × 10–15 min at 1 thread plus the 80-min
sort eval ≈ 4 CPU-h. Recording ≈ 2 CPU-h (MuJoCo EGL falls back to llvmpipe software rendering,
~1.1 cores per recorder). Total ≈ 16–17 CPU-hours.

## Gaps and honest caveats

* Absolute numbers are low (best 25 %) and all CIs between E1, E4 and E5 overlap; the only
  differences that are clearly outside noise are "100 DR demos vs 30 nominal" and "ACT vs DP at
  equal CPU budget".
* Every model is under-trained by normal standards; the trend (E1 → E5) says longer training is
  the next lever. With download access, an ImageNet ResNet-18 is still expected to be the biggest
  single improvement (not testable here).
* The scene camera at 96×96 shows the blocks as 3–5 px; 128 px or a crop around the workspace
  would likely help but costs ~1.8× per step.
* Eval seeds are shared by all rows, so rows are paired, but 20 episodes cannot resolve
  differences smaller than ~25 points.
* The language encoder is a stub with no semantics (see below).
