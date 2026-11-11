"""Open-loop diagnostics for a checkpoint: inference-mode action error on expert episodes.

    python -m policies.diagnose --checkpoint outputs/.../pretrained_model \
        --datasets TRAIN_DS --heldout HELDOUT_DS

For every frame t the policy predicts its action chunk from the expert's observation at t; we
report the error of the *first* predicted action (what temporal ensembling weights most) against
the expert's action, in radians per joint, over all frames and over the "grasp window" (frames
where the expert gripper command is closing near the table). A large train/held-out gap means the
policy memorises layouts instead of reading the block position from the images.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from policies.data import ChunkDataset, collate
from policies.eval_sim import load_policy
from sim.spec import ACTION_NAMES


@torch.no_grad()
def first_action_errors(ckpt, roots: list[Path], max_frames: int = 1500, batch: int = 64) -> dict:
    policy, pre, post, enc, meta = load_policy(ckpt)
    cfg = policy.config
    ds = ChunkDataset(roots, cfg.action_delta_indices, cfg.observation_delta_indices,
                      state_mode=meta.get("state_mode", "joints"))
    rng = np.random.default_rng(0)
    idx = np.sort(rng.choice(len(ds), size=min(max_frames, len(ds)), replace=False))
    errs, grasp = [], []
    for k in range(0, len(idx), batch):
        items = [ds[int(i)] for i in idx[k:k + batch]]
        b = pre(collate(items))
        if cfg.type == "act":
            chunk = policy.predict_action_chunk(b)          # normalised (B, chunk, 6)
            a0 = post(chunk[:, 0])
        else:
            policy.reset()
            chunk = policy.diffusion.generate_actions(_dp_batch(policy, b))
            a0 = post(chunk[:, 0])
        target = torch.stack([it["action"][0] for it in items])
        e = (a0 - target).abs().numpy()
        errs.append(e)
        for j, it in enumerate(items):
            st = it["observation.state"]
            st = st[-1] if st.ndim == 2 else st
            # grasp window: the expert is closing the gripper (command well below the current opening)
            if it["action"][0, 5] < st[5] - 0.1 and st[5] > 0.9:
                grasp.append(e[j])
    errs = np.concatenate(errs)
    out = {"frames": int(len(errs)), "mean_abs_err_rad": dict(zip(ACTION_NAMES, errs.mean(0).round(4).tolist())),
           "arm_mean_abs_err_rad": float(errs[:, :5].mean())}
    if grasp:
        g = np.stack(grasp)
        out["grasp_window_frames"] = int(len(g))
        out["grasp_window_arm_err_rad"] = float(g[:, :5].mean())
    return out


def _dp_batch(policy, b):
    """Diffusion expects stacked image key + time axis; mirror DiffusionPolicy.select_action's prep."""
    from lerobot.utils.constants import OBS_IMAGES

    b = dict(b)
    if policy.config.image_features:
        b[OBS_IMAGES] = torch.stack([b[k] for k in policy.config.image_features], dim=-4)
    return b


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--datasets", nargs="+", type=Path, required=True, help="training-distribution episodes")
    ap.add_argument("--heldout", nargs="+", type=Path, required=True, help="held-out-layout episodes")
    ap.add_argument("--max-frames", type=int, default=1500)
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    torch.set_num_threads(a.threads)
    r = {"train": first_action_errors(a.checkpoint, a.datasets, a.max_frames),
         "heldout": first_action_errors(a.checkpoint, a.heldout, a.max_frames)}
    print(json.dumps(r, indent=2))
    if a.out:
        a.out.write_text(json.dumps(r, indent=2))


if __name__ == "__main__":
    main()
