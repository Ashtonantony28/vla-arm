"""Roll a trained checkpoint out in the MuJoCo sim and report success with a Wilson 95% interval.

    python -m policies.eval_sim --checkpoint outputs/train/act_pick_place/pretrained_model \
        --task pick_place --episodes 20

Eval seeds start at sim.record.EVAL_SEED_BASE, disjoint from the recording seeds, so every policy
is evaluated on the same 20 held-out layouts (the sim analogue of PLAN 6.6's fixed start poses).
`--instructions heldout` uses the paraphrases held out of training (language generalisation).
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch

from policies.factory import LANG_KEY
from policies.text_encoder import get_text_encoder
from sim.envs.tabletop import TabletopEnv
from sim.metrics import fmt_rate, wilson_interval
from sim.record import EVAL_SEED_BASE


def load_policy(ckpt: str | Path):
    from lerobot.policies.factory import make_pre_post_processors

    ckpt = Path(ckpt)
    meta = json.loads((ckpt / "vla_arm_meta.json").read_text())
    if meta["policy"] in ("act", "act_lang"):
        from lerobot.policies.act.modeling_act import ACTPolicy as P
    else:
        from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy as P
    policy = P.from_pretrained(ckpt)
    policy.eval()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=ckpt)
    enc = get_text_encoder(meta["text_encoder"]) if meta.get("text_encoder") else None
    return policy, pre, post, enc, meta


def obs_to_batch(obs: dict, camera_keys: list[str], enc=None) -> dict:
    b = {"observation.state": torch.from_numpy(np.asarray(obs["observation.state"], np.float32))[None],
         "task": [obs["task"]]}
    for c in camera_keys:
        img = torch.from_numpy(obs[c]).permute(2, 0, 1).float() / 255.0
        b[c] = img[None]
    if enc is not None:
        b[LANG_KEY] = torch.from_numpy(enc.encode([obs["task"]]))
    return b


@torch.no_grad()
def evaluate(ckpt, task: str, episodes: int = 20, seed0: int = EVAL_SEED_BASE, instructions: str = "train",
             max_steps: int | None = None, video_dir: Path | None = None, verbose: bool = True) -> dict:
    policy, pre, post, enc, meta = load_policy(ckpt)
    cams = meta["camera_keys"]
    env = TabletopEnv(task, image_size=meta["image_size"], cameras=[c.split(".")[-1] for c in cams],
                      instruction_split=instructions, max_steps=max_steps)
    results, t0 = [], time.time()
    for k in range(episodes):
        seed = seed0 + k
        obs, info = env.reset(seed=seed)
        policy.reset()
        frames, success, steps = [], False, 0
        for steps in range(1, env.max_steps + 1):
            action = post(policy.select_action(pre(obs_to_batch(obs, cams, enc))))
            a = action.squeeze(0).cpu().numpy()
            if video_dir is not None:
                frames.append(np.concatenate([obs[c] for c in cams], axis=1))
            obs, _, terminated, truncated, info = env.step(a)
            if terminated:
                success = True
                break
            if truncated:
                break
        results.append({"seed": seed, "success": success, "steps": steps, "instruction": env.instruction})
        if verbose:
            print(f"[eval] {task} seed {seed} {'OK ' if success else 'FAIL'} steps {steps} '{env.instruction}'",
                  flush=True)
        if video_dir is not None and frames:
            import imageio

            video_dir.mkdir(parents=True, exist_ok=True)
            imageio.mimsave(video_dir / f"{task}_{seed}_{'ok' if success else 'fail'}.mp4", frames, fps=30)
    env.close()
    n_ok = sum(r["success"] for r in results)
    lo, hi = wilson_interval(n_ok, episodes)
    summary = {"checkpoint": str(ckpt), "task": task, "instructions": instructions, "episodes": episodes,
               "successes": n_ok, "rate": n_ok / episodes, "wilson95": [lo, hi],
               "mean_steps_success": float(np.mean([r["steps"] for r in results if r["success"]])) if n_ok else None,
               "seconds": time.time() - t0, "per_episode": results}
    if verbose:
        print(f"[eval] {task}: {fmt_rate(n_ok, episodes)}")
    return summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--task", default="pick_place")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--seed0", type=int, default=EVAL_SEED_BASE)
    ap.add_argument("--instructions", choices=["train", "heldout", "all"], default="train")
    ap.add_argument("--max-steps", type=int, default=None)
    ap.add_argument("--video-dir", type=Path, default=None)
    ap.add_argument("--out", type=Path, default=None, help="write the JSON summary here")
    a = ap.parse_args(argv)
    s = evaluate(a.checkpoint, a.task, a.episodes, a.seed0, a.instructions, a.max_steps, a.video_dir)
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps(s, indent=2))
    print(json.dumps({k: v for k, v in s.items() if k != "per_episode"}))
    return s


if __name__ == "__main__":
    main()
