"""One controlled experiment = train + held-out sim eval, appended as a row to a results JSONL.

    python -m policies.run_experiment --name E2_smallcnn --results outputs/wave2/results.jsonl \
        --eval-randomize default -- --policy act --datasets ... --backbone small_cnn --steps 3000

Everything after `--` is passed to `policies.train` (its --out is set to <out-root>/<name>).
The eval uses the fixed held-out seeds (sim.record.EVAL_SEED_BASE...), so rows are comparable.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import torch

from policies import eval_sim, train
from sim.metrics import wilson_interval


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", required=True)
    ap.add_argument("--results", type=Path, required=True)
    ap.add_argument("--out-root", type=Path, required=True)
    ap.add_argument("--task", default="pick_place")
    ap.add_argument("--episodes", type=int, default=20)
    ap.add_argument("--eval-randomize", nargs="+", default=["default"], help="one eval per preset")
    ap.add_argument("--instructions", nargs="+", default=["train"])
    ap.add_argument("--note", default="")
    ap.add_argument("--eval-threads", type=int, default=2)
    ap.add_argument("--skip-train", action="store_true", help="evaluate an existing <out-root>/<name>")
    ap.add_argument("train_args", nargs=argparse.REMAINDER)
    a = ap.parse_args(argv)
    targs = [x for x in a.train_args if x != "--"]
    out = a.out_root / a.name
    t0 = time.time()
    if not a.skip_train:
        train.main(targs + ["--out", str(out)])
    meta = json.loads((out / "train_meta.json").read_text())
    torch.set_num_threads(a.eval_threads)
    for rz in a.eval_randomize:
        for ins in a.instructions:
            s = eval_sim.evaluate(out / "pretrained_model", a.task, a.episodes, instructions=ins,
                                  randomize=None if rz == "none" else rz, verbose=True)
            (out / f"eval_{rz}_{ins}.json").write_text(json.dumps(s, indent=2))
            lo, hi = wilson_interval(s["successes"], s["episodes"])
            row = {"name": a.name, "note": a.note, "task": a.task, "policy": meta["policy"],
                   "train_args": targs, "steps": meta["steps"], "train_min": round(meta["train_seconds"] / 60, 1),
                   "n_params_M": round(meta["n_params"] / 1e6, 2), "final_loss": meta.get("final_logged_loss"),
                   "final_val_l1": meta.get("final_val_l1"), "eval_randomize": rz, "instructions": ins,
                   "successes": s["successes"], "episodes": s["episodes"], "wilson95": [round(lo, 3), round(hi, 3)],
                   "categories": s["categories"], "median_first_grasp_offset_mm": s["median_first_grasp_offset_mm"],
                   "eval_min": round(s["seconds"] / 60, 1), "total_min": round((time.time() - t0) / 60, 1)}
            with open(a.results, "a") as fh:
                fh.write(json.dumps(row) + "\n")
            print("[experiment]", json.dumps(row))


if __name__ == "__main__":
    main()
