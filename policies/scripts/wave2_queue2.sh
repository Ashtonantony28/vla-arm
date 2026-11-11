#!/bin/bash
# Wave-2 experiment queue 2. After queue 1 (E1 ResNet-18 2/20, E2 SmallCNN 0/20) the SmallCNN branch
# was dropped (E3 killed after 11 min); the next levers are applied on top of E1 (ResNet-18).
set -u
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO"
export MUJOCO_GL=egl PYTHONPATH=$REPO OMP_NUM_THREADS=2
P=${P:-$REPO/sim/.venv/bin/python}
W=${W:-$REPO/outputs/wave2}
D=$W/datasets
DS="$D/vla_arm_sim_pick_place_dr_a $D/vla_arm_sim_pick_place_dr_b"
VAL="$D/vla_arm_sim_pick_place_dr_val"
COMMON="--policy act --datasets $DS --val-datasets $VAL --val-every 500 --batch-size 16 --threads 2 --save-every 1000"
mkdir -p "$W/runs" "$W/logs"

run() {  # name note train-args...
  name=$1; note=$2; shift 2
  echo "=== $name $(date +%T)"
  $P -m policies.train $COMMON "$@" --out "$W/runs/$name" > "$W/logs/$name.train.log" 2>&1
  OMP_NUM_THREADS=1 nohup $P -m policies.run_experiment --name "$name" --note "$note" --results "$W/results.jsonl" \
      --out-root "$W/runs" --eval-threads 1 --skip-train -- $COMMON "$@" > "$W/logs/$name.eval.log" 2>&1 &
}

run E3_resnet18_ee "E1 + EE position in state" --steps 3000 --state-mode joints_ee
run E4_resnet18_ee_tiny "E3 with the tiny transformer (dim 128), same 38-min budget -> more steps" \
    --steps 8000 --time-limit-min 38 --state-mode joints_ee --size tiny
wait
echo "=== queue2 done $(date +%T)"
