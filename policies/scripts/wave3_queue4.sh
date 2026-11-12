#!/bin/bash
# Wave-3 queue 4. The container restarted during queue 3 with P2 and P3 at step ~2100 (side-by-side
# training on 4 cores ran at 2.4 s/step). Both resume from their step-2000 checkpoints for 4000
# more steps (6000 total, like P1; AdamW state restarts, as in wave 2's E5), one at a time.
set -u
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO"
export MUJOCO_GL=egl PYTHONPATH=$REPO
P=${P:-python}
W=${W:-$REPO/outputs/wave3}
D=$W/datasets
DS="$D/vla_arm_sim_pick_place_dr_a $D/vla_arm_sim_pick_place_dr_b"
VAL="$D/vla_arm_sim_pick_place_dr_val"
COMMON="--policy act --datasets $DS --val-datasets $VAL --val-every 500 --batch-size 16 --save-every 1000 --steps 4000 --pretrained-backbone"
mkdir -p "$W/runs" "$W/logs" "$W/evals"
resume() {  # name threads train-args...
  name=$1; t=$2; shift 2
  echo "=== train ${name}_6k $(date +%T)"
  OMP_NUM_THREADS=$t $P -m policies.train $COMMON --threads $t "$@" \
      --init-checkpoint "$W/runs/$name/pretrained_model" --out "$W/runs/${name}_6k" > "$W/logs/${name}_6k.train.log" 2>&1
}
evaluate() {  # name threads
  OMP_NUM_THREADS=$2 $P -m policies.eval_sim --checkpoint "$W/runs/$1/pretrained_model" --task pick_place \
      --episodes 40 --randomize default --threads $2 --out "$W/evals/$1.json" > "$W/logs/$1.eval.log" 2>&1
  echo "=== evaluated $1 $(date +%T)"
}
resume P2_imagenet_lrbb1e-5 4 --lr-backbone 1e-5
evaluate P2_imagenet_lrbb1e-5_6k 1 &
resume P3_imagenet_layer3 3 --backbone resnet18_layer3
evaluate P3_imagenet_layer3_6k 3
wait
echo "=== wave3 queue4 done $(date +%T)"
