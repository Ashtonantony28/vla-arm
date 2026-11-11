"""Roll a trained checkpoint out in the MuJoCo sim and report success with a Wilson 95% interval.

    python -m policies.eval_sim --checkpoint outputs/train/act_pick_place/pretrained_model \
        --task pick_place --episodes 20 [--randomize default]

Eval seeds start at sim.record.EVAL_SEED_BASE, disjoint from the recording seeds, so every policy
is evaluated on the same 20 held-out layouts (the sim analogue of PLAN 6.6's fixed start poses).
Domain randomisation uses its own random stream, so `--randomize` changes the look (and adds
action noise) but not the layouts. `--instructions heldout` uses the paraphrases held out of
training (language generalisation). Each episode is also given a PLAN 6.6 failure category.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import torch

from policies.factory import LANG_KEY
from policies.state import transform_state
from policies.text_encoder import get_text_encoder
from sim.envs.tabletop import TabletopEnv
from sim.metrics import fmt_rate, wilson_interval
from sim.record import EVAL_SEED_BASE


def load_policy(ckpt: str | Path):
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.factory import make_pre_post_processors

    from policies.backbones import apply_backbone

    ckpt = Path(ckpt)
    meta = json.loads((ckpt / "vla_arm_meta.json").read_text())
    if meta["policy"] in ("act", "act_lang"):
        from lerobot.policies.act.modeling_act import ACTPolicy as P
    else:
        from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy as P
    # Build, swap in the custom backbone if any, then load the weights *strictly* (LeRobot's
    # from_pretrained defaults to strict=False, which would silently skip mismatched weights).
    cfg = PreTrainedConfig.from_pretrained(ckpt)
    policy = apply_backbone(P(cfg), meta.get("backbone"))
    policy = P._load_as_safetensor(policy, str(ckpt / "model.safetensors"), "cpu", True)
    policy.eval()
    pre, post = make_pre_post_processors(policy.config, pretrained_path=ckpt)
    enc = get_text_encoder(meta["text_encoder"]) if meta.get("text_encoder") else None
    return policy, pre, post, enc, meta


def obs_to_batch(obs: dict, camera_keys: list[str], enc=None, state_mode: str = "joints") -> dict:
    state = transform_state(np.asarray(obs["observation.state"], np.float32), state_mode)
    b = {"observation.state": torch.from_numpy(state)[None], "task": [obs["task"]]}
    for c in camera_keys:
        img = torch.from_numpy(obs[c]).permute(2, 0, 1).float() / 255.0
        b[c] = img[None]
    if enc is not None:
        b[LANG_KEY] = torch.from_numpy(enc.encode([obs["task"]]))
    return b


class FailureTracker:
    """PLAN 6.6 failure categories from privileged sim state:
    success | missed_grasp (target never lifted) | wrong_object (a non-target block lifted before the
    target) | dropped (target lifted but not delivered).
    Also records the xy distance TCP -> target block when the gripper first closes near the table
    (the wave-1 failure mode was "closes beside the block")."""

    LIFT_Z = 0.03  # block centre height that counts as lifted (it rests at 0.015)

    def __init__(self, env: TabletopEnv):
        self.env = env
        self.target = "red_block"
        self.others = [] if env.task_name == "sort" else ["blue_block"]  # sort: both are targets
        self.lifted_target = self.lifted_other = False
        self.first_grasp_offset = None

    def update(self, action):
        env = self.env
        tcp = env.tcp_pos()
        if self.first_grasp_offset is None and action[5] < 0.6 and tcp[2] < 0.05:
            p, _ = env.object_pose(self.target)
            self.first_grasp_offset = float(np.hypot(*(tcp[:2] - p[:2])) * 1000)
        if env.object_pose(self.target)[0][2] > self.LIFT_Z:
            self.lifted_target = True
        for o in self.others:
            if env.object_pose(o)[0][2] > self.LIFT_Z and not self.lifted_target:
                self.lifted_other = True

    def summary(self, success: bool) -> dict:
        if success:
            cat = "success"
        elif self.lifted_other:
            cat = "wrong_object"
        elif not self.lifted_target:
            cat = "missed_grasp"
        else:
            cat = "dropped"
        off = None if self.first_grasp_offset is None else round(self.first_grasp_offset, 1)
        return {"category": cat, "first_grasp_offset_mm": off}


@torch.no_grad()
def evaluate(ckpt, task: str, episodes: int = 20, seed0: int = EVAL_SEED_BASE, instructions: str = "train",
             max_steps: int | None = None, video_dir: Path | None = None, verbose: bool = True,
             randomize=None) -> dict:
    """randomize: domain-randomisation preset for the eval env (None = nominal scene)."""
    policy, pre, post, enc, meta = load_policy(ckpt)
    cams = meta["camera_keys"]
    state_mode = meta.get("state_mode", "joints")
    env = TabletopEnv(task, image_size=meta["image_size"], cameras=[c.split(".")[-1] for c in cams],
                      instruction_split=instructions, max_steps=max_steps, randomize=randomize)
    results, t0 = [], time.time()
    for k in range(episodes):
        seed = seed0 + k
        obs, info = env.reset(seed=seed)
        policy.reset()
        frames, success, steps = [], False, 0
        diag = FailureTracker(env)
        for steps in range(1, env.max_steps + 1):
            action = post(policy.select_action(pre(obs_to_batch(obs, cams, enc, state_mode))))
            a = action.squeeze(0).cpu().numpy()
            diag.update(a)
            if video_dir is not None:
                frames.append(np.concatenate([obs[c] for c in cams], axis=1))
            obs, _, terminated, truncated, info = env.step(a)
            if terminated:
                success = True
                break
            if truncated:
                break
        d = diag.summary(success)
        results.append({"seed": seed, "success": success, "steps": steps, "instruction": env.instruction, **d})
        if verbose:
            print(f"[eval] {task} seed {seed} {'OK ' if success else 'FAIL'} steps {steps:3d} {d['category']:12s} "
                  f"grasp_offset {d['first_grasp_offset_mm']} mm '{env.instruction}'", flush=True)
        if video_dir is not None and frames:
            import imageio

            video_dir.mkdir(parents=True, exist_ok=True)
            imageio.mimsave(video_dir / f"{task}_{seed}_{'ok' if success else 'fail'}.mp4", frames, fps=30)
    env.close()
    n_ok = sum(r["success"] for r in results)
    lo, hi = wilson_interval(n_ok, episodes)
    cats: dict[str, int] = {}
    for r in results:
        cats[r["category"]] = cats.get(r["category"], 0) + 1
    offs = [r["first_grasp_offset_mm"] for r in results if r["first_grasp_offset_mm"] is not None]
    summary = {"checkpoint": str(ckpt), "task": task, "instructions": instructions, "episodes": episodes,
               "randomize": randomize, "successes": n_ok, "rate": n_ok / episodes, "wilson95": [lo, hi],
               "categories": cats,
               "median_first_grasp_offset_mm": float(np.median(offs)) if offs else None,
               "mean_steps_success": float(np.mean([r["steps"] for r in results if r["success"]])) if n_ok else None,
               "seconds": time.time() - t0, "per_episode": results}
    if verbose:
        print(f"[eval] {task}: {fmt_rate(n_ok, episodes)} {cats}")
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
    ap.add_argument("--randomize", default="none", help="eval-env domain randomisation: none|visual|default")
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--out", type=Path, default=None, help="write the JSON summary here")
    a = ap.parse_args(argv)
    torch.set_num_threads(a.threads)
    s = evaluate(a.checkpoint, a.task, a.episodes, a.seed0, a.instructions, a.max_steps, a.video_dir,
                 randomize=None if a.randomize == "none" else a.randomize)
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps(s, indent=2))
    print(json.dumps({k: v for k, v in s.items() if k != "per_episode"}))
    return s


if __name__ == "__main__":
    main()
