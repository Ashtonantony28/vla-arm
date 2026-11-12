#!/bin/bash
# Wave-3 queue 1 (results in policies/RESULTS.md, wave-3 section): the wave-2 best recipe (E5:
# ACT small, 100 DR pick_place demos, 6000 steps) with the backbone started from ImageNet weights
# instead of random init. Install the weights first: python -m policies.pretrained --resnet18 ...
# Usage: policies/scripts/wave3_queue1.sh   (from the repo root; W= output root, T= train threads)
set -u
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO"
export MUJOCO_GL=egl PYTHONPATH=$REPO
P=${P:-python}
W=${W:-$REPO/outputs/wave3}
T=${T:-3}
D=$W/datasets
DS="$D/vla_arm_sim_pick_place_dr_a $D/vla_arm_sim_pick_place_dr_b"
VAL="$D/vla_arm_sim_pick_place_dr_val"
COMMON="--policy act --datasets $DS --val-datasets $VAL --val-every 500 --batch-size 16 --threads $T --save-every 1000"
mkdir -p "$W/runs" "$W/logs" "$W/evals"

train() {  # name train-args...
  name=$1; shift
  echo "=== train $name $(date +%T)"
  OMP_NUM_THREADS=$T $P -m policies.train $COMMON "$@" --out "$W/runs/$name" > "$W/logs/$name.train.log" 2>&1
}
evaluate() {  # name threads  -> 40 held-out DR layouts (seeds 900000-900039, same as wave 2)
  OMP_NUM_THREADS=$2 $P -m policies.eval_sim --checkpoint "$W/runs/$1/pretrained_model" --task pick_place \
      --episodes 40 --randomize default --threads $2 --out "$W/evals/$1.json" > "$W/logs/$1.eval.log" 2>&1
}

train P1_imagenet_6k --steps 6000 --pretrained-backbone
evaluate P1_imagenet_6k 1 &
wait
echo "=== wave3 queue1 done $(date +%T)"
