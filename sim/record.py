"""Record scripted-expert episodes into a LeRobot dataset (LeRobotDataset writer, format v3.0).

    python -m sim.record --task pick_place --episodes 30 --image-size 96
    python -m sim.record --task all --episodes 30 --image-size 96      # one dataset per task
    python -m sim.record --task pick_place --episodes 100 --randomize default \
        --root outputs/wave2/datasets --name-suffix _dr                  # domain-randomised

Default output: <repo>/outputs/datasets/vla_arm_sim_<task>  (outputs/ is gitignored).
Only successful expert episodes are kept; failed seeds are skipped and logged.
Seeds: episode k of task T uses env seed RECORD_SEED_BASE + k; eval uses EVAL_SEED_BASE + k, so
train and eval layouts never coincide.
"""
from __future__ import annotations

import argparse
import json
import shutil
import time
from pathlib import Path

import numpy as np

from sim.envs.tabletop import CAMERAS, CONTROL_HZ, TabletopEnv
from sim.envs.randomization import config_dict
from sim.envs.tasks import TASKS
from sim.expert.scripted import run_episode
from sim.spec import ACTION_NAMES, REPO_ROOT

RECORD_SEED_BASE = 0
EVAL_SEED_BASE = 900_000
DEFAULT_ROOT = REPO_ROOT / "outputs" / "datasets"
ROBOT_TYPE = "vla_arm_sim"


def features(image_size: int, cameras=CAMERAS, use_videos: bool = True) -> dict:
    f = {
        "observation.state": {"dtype": "float32", "shape": (6,), "names": list(ACTION_NAMES)},
        "action": {"dtype": "float32", "shape": (6,), "names": list(ACTION_NAMES)},
    }
    for c in cameras:
        f[f"observation.images.{c}"] = {"dtype": "video" if use_videos else "image",
                                        "shape": (image_size, image_size, 3),
                                        "names": ["height", "width", "channels"]}
    return f


def record(task: str, episodes: int, root: Path, image_size: int = 96, seed0: int = RECORD_SEED_BASE,
           overwrite: bool = False, extra_steps: int = 10, verbose: bool = True, randomize=None):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(root)
    if root.exists():
        if not overwrite:
            raise FileExistsError(f"{root} exists (use --overwrite)")
        shutil.rmtree(root)
    ds = LeRobotDataset.create(repo_id=f"local/{root.name}", fps=CONTROL_HZ, features=features(image_size),
                               root=root, robot_type=ROBOT_TYPE, use_videos=True)
    env = TabletopEnv(task, image_size=image_size, randomize=randomize)
    sidecar, skipped = [], []
    seed, t0 = seed0, time.time()
    while len(sidecar) < episodes:
        frames = []

        def on_step(obs, a):
            fr = {"observation.state": obs["observation.state"].astype(np.float32),
                  "action": a.astype(np.float32), "task": obs["task"]}
            for c in CAMERAS:
                fr[f"observation.images.{c}"] = obs[f"observation.images.{c}"]
            frames.append(fr)

        ok, n = run_episode(env, seed, extra_steps=extra_steps, on_step=on_step)
        if ok:
            for fr in frames:
                ds.add_frame(fr)
            ds.save_episode()
            sidecar.append({"episode_index": len(sidecar), "seed": seed, "task_name": task,
                            "instruction": env.instruction, "length": len(frames),
                            "assignment": env.assignment,
                            "layout": {k: list(v) for k, v in env.layout.items()},
                            "domain_randomization": env.dr_params})
            if verbose:
                print(f"[{task}] ep {len(sidecar) - 1:3d} seed {seed} len {len(frames)} '{env.instruction}'")
        else:
            skipped.append(seed)
        seed += 1
    ds.finalize()
    env.close()
    with open(root / "meta" / "sim_episodes.jsonl", "w") as fh:
        for row in sidecar:
            fh.write(json.dumps(row) + "\n")
    size = sum(p.stat().st_size for p in root.rglob("*") if p.is_file())
    summary = {"task": task, "root": str(root), "episodes": len(sidecar), "frames": ds.meta.total_frames,
               "skipped_failed_seeds": skipped, "bytes": size, "seconds": time.time() - t0,
               "image_size": image_size, "randomize": randomize,
               "dr_config": config_dict(env.dr_config)}
    (root / "meta" / "sim_record_summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--task", default="pick_place", choices=list(TASKS) + ["all"])
    ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--image-size", type=int, default=96)
    ap.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    ap.add_argument("--seed0", type=int, default=RECORD_SEED_BASE)
    ap.add_argument("--overwrite", action="store_true")
    ap.add_argument("--randomize", default="none", help="domain randomisation preset: none|visual|default")
    ap.add_argument("--name-suffix", default="", help="dataset dir = vla_arm_sim_<task><suffix>")
    args = ap.parse_args(argv)
    tasks = list(TASKS) if args.task == "all" else [args.task]
    for t in tasks:
        s = record(t, args.episodes, args.root / f"vla_arm_sim_{t}{args.name_suffix}", args.image_size,
                   args.seed0, overwrite=args.overwrite,
                   randomize=None if args.randomize == "none" else args.randomize)
        print(json.dumps(s))


if __name__ == "__main__":
    main()
