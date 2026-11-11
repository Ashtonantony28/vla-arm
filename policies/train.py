"""Train ACT / ACT+language / Diffusion Policy (LeRobot implementations) on LeRobot datasets.

    python -m policies.train --policy act --datasets outputs/datasets/vla_arm_sim_pick_place \
        --steps 3000 --batch-size 16 --out outputs/train/act_pick_place

    python -m policies.train --policy act_lang --datasets outputs/datasets/vla_arm_sim_{pick_place,stack,sort}

Writes <out>/pretrained_model/ (LeRobot policy + processor files, loadable with
`from_pretrained`), <out>/train_log.csv and <out>/train_meta.json.
"""
from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np
import torch

from policies.data import ChunkDataset, collate
from policies.factory import LANG_KEY, config_summary, make_config, make_policy
from policies.state import state_dim
from policies.text_encoder import get_text_encoder


def build(kind: str, roots: list[Path], size: str = "small", episodes=None, augment_shift: int = 4,
          text_encoder: str = "auto", chunk_size: int = 50, backbone: dict | None = None,
          state_mode: str = "joints", color_jitter: float = 0.0, lr: float | None = None):
    from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata

    meta = LeRobotDatasetMetadata(repo_id=f"local/{roots[0].name}", root=roots[0])
    image_size = meta.features[meta.camera_keys[0]]["shape"][0]
    enc = get_text_encoder(text_encoder) if kind == "act_lang" else None
    cfg = make_config(kind, meta.features, image_size, lang_dim=enc.dim if enc else None, size=size,
                      chunk_size=chunk_size, state_dim=state_dim(state_mode) if state_mode != "joints" else None,
                      lr=lr)
    ds = ChunkDataset(roots, cfg.action_delta_indices, cfg.observation_delta_indices, episodes=episodes,
                      augment_shift=augment_shift, color_jitter=color_jitter, state_mode=state_mode)
    policy, pre, post = make_policy(cfg, ds.stats(), backbone=backbone)
    return cfg, ds, policy, pre, post, enc


def backbone_spec(name: str, stride: int = 16, width: int = 32) -> dict | None:
    return None if name == "resnet18" else {"name": name, "stride": stride, "width": width}


@torch.no_grad()
def validation_loss(policy, pre, enc, val_ds, batch_size: int = 64, max_batches: int = 8) -> dict:
    """Mean loss on held-out expert episodes, policy in eval mode. For ACT this is the inference-mode
    L1 on normalised action chunks (latent z = 0, as at deployment)."""
    policy.eval()
    g = torch.Generator().manual_seed(0)
    idx = torch.randperm(len(val_ds), generator=g)[: batch_size * max_batches].tolist()
    losses, l1s = [], []
    for k in range(0, len(idx), batch_size):
        batch = pre(add_language(collate([val_ds[i] for i in idx[k:k + batch_size]]), enc))
        torch.manual_seed(0)
        loss, info = policy.forward(batch)
        losses.append(loss.item())
        if info and "l1_loss" in info:
            l1s.append(info["l1_loss"])
    policy.train()
    return {"val_loss": float(np.mean(losses)), "val_l1": float(np.mean(l1s)) if l1s else None}


def add_language(batch: dict, enc) -> dict:
    if enc is not None:
        batch[LANG_KEY] = torch.from_numpy(enc.encode(batch["task"]))
    return batch


def save_checkpoint(out: Path, policy, pre, post, meta: dict):
    d = out / "pretrained_model"
    policy.save_pretrained(d)
    pre.save_pretrained(d)
    post.save_pretrained(d)
    (d / "vla_arm_meta.json").write_text(json.dumps(meta, indent=2))
    return d


def train(kind: str, roots: list[Path], out: Path, steps: int, batch_size: int = 16, seed: int = 0,
          size: str = "small", episodes=None, time_limit_min: float | None = None, log_every: int = 25,
          save_every: int = 1000, augment_shift: int = 4, text_encoder: str = "auto",
          num_threads: int | None = None, chunk_size: int = 50, verbose: bool = True,
          backbone: dict | None = None, state_mode: str = "joints", color_jitter: float = 0.0,
          lr: float | None = None, val_roots: list[Path] | None = None, val_every: int = 500) -> dict:
    torch.manual_seed(seed)
    np.random.seed(seed)
    if num_threads:
        torch.set_num_threads(num_threads)
    out.mkdir(parents=True, exist_ok=True)
    t_load = time.time()
    cfg, ds, policy, pre, post, enc = build(kind, roots, size, episodes, augment_shift, text_encoder, chunk_size,
                                            backbone, state_mode, color_jitter, lr)
    val_ds = (ChunkDataset(val_roots, cfg.action_delta_indices, cfg.observation_delta_indices,
                           state_mode=state_mode) if val_roots else None)
    t_load = time.time() - t_load
    n_params = sum(p.numel() for p in policy.parameters())
    if verbose:
        print(f"[train] {kind}: {len(ds)} frames, {len(np.unique(ds.ep))} episodes, "
              f"{n_params / 1e6:.1f} M params, data load {t_load:.0f}s"
              + (f", text encoder {enc.name}{' (STUB)' if enc.is_stub else ''}" if enc else ""))
    optim = cfg.get_optimizer_preset().build(policy.get_optim_params())
    sched_cfg = cfg.get_scheduler_preset()
    sched = sched_cfg.build(optim, steps) if sched_cfg is not None else None
    loader = torch.utils.data.DataLoader(ds, batch_size=batch_size, shuffle=True, collate_fn=collate,
                                         drop_last=True, num_workers=0)
    meta = {"policy": kind, "datasets": [str(r) for r in roots], "episodes": episodes, "size": size,
            "text_encoder": enc.name if enc else None, "text_encoder_is_stub": bool(enc and enc.is_stub),
            "image_size": ds.images[ds.camera_keys[0]].shape[-1], "camera_keys": ds.camera_keys,
            "n_params": n_params, "batch_size": batch_size, "seed": seed, "config": config_summary(cfg),
            "backbone": backbone, "state_mode": state_mode, "augment_shift": augment_shift,
            "color_jitter": color_jitter, "val_datasets": [str(r) for r in val_roots or []]}
    log_f = open(out / "train_log.csv", "w", newline="")
    log = csv.writer(log_f)
    log.writerow(["step", "loss", "l1_loss", "kld_loss", "lr", "elapsed_s", "val_loss", "val_l1"])
    val_hist = []
    policy.train()
    step, t0, run_loss, history = 0, time.time(), [], []
    stop = False
    while not stop:
        for batch in loader:
            batch = pre(add_language(batch, enc))
            loss, info = policy.forward(batch)
            optim.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 10.0)
            optim.step()
            if sched is not None:
                sched.step()
            step += 1
            run_loss.append(loss.item())
            if step % log_every == 0 or step == 1:
                info = info or {}
                row = [step, float(np.mean(run_loss)), info.get("l1_loss"), info.get("kld_loss"),
                       optim.param_groups[0]["lr"], time.time() - t0, None, None]
                if val_ds is not None and (step % val_every == 0 or step == 1):
                    v = validation_loss(policy, pre, enc, val_ds)
                    row[6], row[7] = v["val_loss"], v["val_l1"]
                    val_hist.append([step, v["val_loss"], v["val_l1"]])
                log.writerow(row)
                log_f.flush()
                history.append(row)
                if verbose:
                    print(f"[train] step {step:5d} loss {row[1]:.4f} l1 {row[2]} kld {row[3]} "
                          f"{row[5] / step:.2f}s/step" + (f" val {row[6]:.4f} val_l1 {row[7]}" if row[6] else ""),
                          flush=True)
                run_loss = []
            if step % save_every == 0:
                save_checkpoint(out, policy, pre, post, {**meta, "steps": step})
            if step % log_every and val_ds is not None and step % val_every == 0:
                v = validation_loss(policy, pre, enc, val_ds)
                val_hist.append([step, v["val_loss"], v["val_l1"]])
            over_time = time_limit_min is not None and time.time() - t0 > time_limit_min * 60
            if step >= steps or over_time:
                stop = True
                break
    wall = time.time() - t0
    log_f.close()
    meta.update({"steps": step, "train_seconds": wall, "data_load_seconds": t_load,
                 "first_logged_loss": history[0][1] if history else None,
                 "final_logged_loss": history[-1][1] if history else None})
    if val_ds is not None:
        v = validation_loss(policy, pre, enc, val_ds)
        val_hist.append([step, v["val_loss"], v["val_l1"]])
        meta.update({"val_history": val_hist, "final_val_loss": v["val_loss"], "final_val_l1": v["val_l1"]})
    ckpt = save_checkpoint(out, policy, pre, post, meta)
    (out / "train_meta.json").write_text(json.dumps(meta, indent=2))
    if verbose:
        print(f"[train] done: {step} steps in {wall / 60:.1f} min -> {ckpt}")
    return {"checkpoint": str(ckpt), **meta, "history": history}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--policy", choices=["act", "act_lang", "diffusion"], default="act")
    ap.add_argument("--datasets", nargs="+", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--steps", type=int, default=3000)
    ap.add_argument("--batch-size", type=int, default=16)
    ap.add_argument("--size", choices=["small", "paper"], default="small")
    ap.add_argument("--episodes", type=int, nargs="*", default=None, help="subset of episode indices")
    ap.add_argument("--time-limit-min", type=float, default=None)
    ap.add_argument("--augment-shift", type=int, default=4)
    ap.add_argument("--text-encoder", default="auto", choices=["auto", "minilm", "hashed_bow"])
    ap.add_argument("--chunk-size", type=int, default=50)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--threads", type=int, default=2)
    ap.add_argument("--backbone", choices=["resnet18", "small_cnn"], default="resnet18")
    ap.add_argument("--backbone-stride", type=int, default=16, choices=[8, 16])
    ap.add_argument("--backbone-width", type=int, default=32)
    ap.add_argument("--state-mode", choices=["joints", "joints_ee"], default="joints")
    ap.add_argument("--color-jitter", type=float, default=0.0)
    ap.add_argument("--lr", type=float, default=None)
    ap.add_argument("--val-datasets", nargs="*", type=Path, default=None)
    ap.add_argument("--val-every", type=int, default=500)
    ap.add_argument("--log-every", type=int, default=25)
    ap.add_argument("--save-every", type=int, default=1000)
    a = ap.parse_args(argv)
    r = train(a.policy, a.datasets, a.out, a.steps, a.batch_size, a.seed, a.size, a.episodes,
              a.time_limit_min, a.log_every, a.save_every, a.augment_shift, a.text_encoder, a.threads,
              a.chunk_size, backbone=backbone_spec(a.backbone, a.backbone_stride, a.backbone_width),
              state_mode=a.state_mode, color_jitter=a.color_jitter, lr=a.lr, val_roots=a.val_datasets,
              val_every=a.val_every)
    print(json.dumps({k: v for k, v in r.items() if k not in ("history", "config", "val_history")}, default=str))


if __name__ == "__main__":
    main()
