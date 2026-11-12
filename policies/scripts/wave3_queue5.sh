#!/bin/bash
# Wave-3 queue 5: control. P1-P3 (ImageNet ResNet-18) scored 6, 3 and 4 /40, all below wave 2's E5
# (random init, 10/40). To tell "pretraining doesn't help here" from drift in data/software, train
# the random-init recipe again on the wave-3 datasets, same 6000 steps (one run, not E5's 2x3000).
set -u
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO"
export MUJOCO_GL=egl PYTHONPATH=$REPO
P=${P:-python}
W=${W:-$REPO/outputs/wave3}
D=$W/datasets
DS="$D/vla_arm_sim_pick_place_dr_a $D/vla_arm_sim_pick_place_dr_b"
VAL="$D/vla_arm_sim_pick_place_dr_val"
name=C1_scratch_6k
OMP_NUM_THREADS=4 $P -m policies.train --policy act --datasets $DS --val-datasets $VAL --val-every 500 \
    --batch-size 16 --threads 4 --save-every 1000 --steps 6000 --out "$W/runs/$name" > "$W/logs/$name.train.log" 2>&1
OMP_NUM_THREADS=4 $P -m policies.eval_sim --checkpoint "$W/runs/$name/pretrained_model" --task pick_place \
    --episodes 40 --randomize default --threads 4 --out "$W/evals/$name.json" > "$W/logs/$name.eval.log" 2>&1
echo "=== wave3 queue5 done $(date +%T)"
