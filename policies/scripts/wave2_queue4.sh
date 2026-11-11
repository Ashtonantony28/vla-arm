#!/bin/bash
# Wave-2 queue 4: DP1 (40 min, 2638 steps) scored 0/20, so give DP the same total budget as the best
# ACT run (E5: 2 x ~37 min): resume DP1 for another 40 minutes.
set -u
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO"
export MUJOCO_GL=egl PYTHONPATH=$REPO OMP_NUM_THREADS=2
P=${P:-$REPO/sim/.venv/bin/python}
W=${W:-$REPO/outputs/wave2}
D=$W/datasets
DS="$D/vla_arm_sim_pick_place_dr_a $D/vla_arm_sim_pick_place_dr_b"
VAL="$D/vla_arm_sim_pick_place_dr_val"
BASE="--datasets $DS --val-datasets $VAL --val-every 500 --batch-size 16 --threads 2 --save-every 1000"

while [ ! -f "$W/runs/L1_act_lang_sort/train_meta.json" ]; do sleep 30; done
name=DP2_resnet18_80min
echo "=== $name $(date +%T)"
ARGS="--policy diffusion $BASE --steps 20000 --time-limit-min 40 --init-checkpoint $W/runs/DP1_resnet18/pretrained_model"
$P -m policies.train $ARGS --out "$W/runs/$name" > "$W/logs/$name.train.log" 2>&1
$P -m policies.run_experiment --name $name --note "DP1 resumed for 40 more minutes (80 min total)" \
    --results "$W/results.jsonl" --out-root "$W/runs" --eval-threads 2 --skip-train -- $ARGS > "$W/logs/$name.eval.log" 2>&1
echo "=== queue4 done $(date +%T)"
