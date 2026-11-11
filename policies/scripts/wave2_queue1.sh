#!/bin/bash
# Wave-2 experiment queue 1 (results in policies/RESULTS.md): one variable at a time on 100
# domain-randomised pick_place demos. Training uses 2 threads, one run at a time; each finished run
# is evaluated in the background (1 thread) while the next one trains.
# Usage: policies/scripts/wave2_queue1.sh   (from the repo root; W= output root)
set -u
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO"
export MUJOCO_GL=egl PYTHONPATH=$REPO OMP_NUM_THREADS=2
P=${P:-$REPO/sim/.venv/bin/python}
W=${W:-$REPO/outputs/wave2}
D=$W/datasets
DS="$D/vla_arm_sim_pick_place_dr_a $D/vla_arm_sim_pick_place_dr_b"
VAL="$D/vla_arm_sim_pick_place_dr_val"
COMMON="--policy act --datasets $DS --val-datasets $VAL --val-every 500 --steps 3000 --batch-size 16 --threads 2 --save-every 1000"
mkdir -p "$W/runs" "$W/logs"

run() {  # name note train-args...
  name=$1; note=$2; shift 2
  echo "=== $name $(date +%T)"
  $P -m policies.train $COMMON "$@" --out "$W/runs/$name" > "$W/logs/$name.train.log" 2>&1
  OMP_NUM_THREADS=1 nohup $P -m policies.run_experiment --name "$name" --note "$note" --results "$W/results.jsonl" \
      --out-root "$W/runs" --eval-threads 1 --skip-train -- $COMMON "$@" > "$W/logs/$name.eval.log" 2>&1 &
}

run E1_resnet18 "wave-1 recipe (ResNet-18 from scratch), 100 DR demos"
run E2_smallcnn "SmallCNN stride 16 replaces ResNet-18" --backbone small_cnn
run E3_smallcnn_ee "E2 + EE position in state" --backbone small_cnn --state-mode joints_ee
run E4_smallcnn_ee_cj "E3 + colour jitter 0.2" --backbone small_cnn --state-mode joints_ee --color-jitter 0.2
wait
echo "=== queue1 done $(date +%T)"
