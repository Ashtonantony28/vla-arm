"""Success rate of the scripted experts.

    python -m sim.expert.evaluate --episodes 100            # all tasks, no rendering
    python -m sim.expert.evaluate --episodes 100 --randomize default   # with action noise / DR
"""
from __future__ import annotations

import argparse
import json
import time

from sim.envs.tabletop import TabletopEnv
from sim.envs.tasks import TASKS
from sim.expert.scripted import run_episode
from sim.metrics import fmt_rate, wilson_interval


def evaluate(task: str, episodes: int, seed0: int = 0, noise: bool = True, randomize=None) -> dict:
    env = TabletopEnv(task, render_images=False, randomize=randomize)
    ok, lengths, failed = 0, [], []
    t0 = time.time()
    for s in range(seed0, seed0 + episodes):
        succ, n = run_episode(env, s, noise=noise, extra_steps=0)
        ok += succ
        lengths.append(n)
        if not succ:
            failed.append(s)
    lo, hi = wilson_interval(ok, episodes)
    return {"task": task, "successes": ok, "episodes": episodes, "rate": ok / episodes,
            "wilson95": [lo, hi], "mean_steps": sum(lengths) / len(lengths),
            "failed_seeds": failed, "sec_per_episode": (time.time() - t0) / episodes}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--tasks", nargs="+", default=list(TASKS))
    ap.add_argument("--episodes", type=int, default=100)
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--no-noise", action="store_true")
    ap.add_argument("--randomize", default="none", help="domain randomisation preset: none|visual|default")
    args = ap.parse_args(argv)
    results = []
    for t in args.tasks:
        r = evaluate(t, args.episodes, args.seed0, noise=not args.no_noise, randomize=args.randomize)
        results.append(r)
        print(f"{t:11s} {fmt_rate(r['successes'], r['episodes'])}  mean {r['mean_steps']:.0f} steps"
              f"  failed seeds {r['failed_seeds'][:10]}")
    print(json.dumps(results))
    return results


if __name__ == "__main__":
    main()
