#!/bin/bash
# Wave-3 queue 3. P1 (ImageNet ResNet-18, wave-2 E5 recipe) scored 6/40 = 15%, not better than E5's
# 10/40 from random init; its median first-grasp offset was 22 mm (E5: 9 mm). Two one-variable
# follow-ups on P1, trained side by side at 2 threads each:
#   P2: backbone lr 1e-5 (the ACT paper's setting for a pretrained backbone; P1 used 1e-4 everywhere)
#   P3: ResNet-18 cut after layer3 (stride 16 -> 6x6 tokens per camera instead of 3x3; finer block
#       localisation; torchvision's BasicBlock can't dilate layer4)
set -u
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO"
export MUJOCO_GL=egl PYTHONPATH=$REPO
P=${P:-python}
W=${W:-$REPO/outputs/wave3}
D=$W/datasets
DS="$D/vla_arm_sim_pick_place_dr_a $D/vla_arm_sim_pick_place_dr_b"
VAL="$D/vla_arm_sim_pick_place_dr_val"
COMMON="--policy act --datasets $DS --val-datasets $VAL --val-every 500 --batch-size 16 --threads 2 --save-every 1000 --steps 6000 --pretrained-backbone"
mkdir -p "$W/runs" "$W/logs" "$W/evals"
# start once queue 2's language run has finished training (its eval may still be running)
while [ ! -f "$W/runs/L2_act_lang_minilm_sort/train_meta.json" ]; do sleep 30; done
one() {  # name train-args...
  name=$1; shift
  echo "=== train $name $(date +%T)"
  OMP_NUM_THREADS=2 $P -m policies.train $COMMON "$@" --out "$W/runs/$name" > "$W/logs/$name.train.log" 2>&1
  OMP_NUM_THREADS=2 $P -m policies.eval_sim --checkpoint "$W/runs/$name/pretrained_model" --task pick_place \
      --episodes 40 --randomize default --threads 2 --out "$W/evals/$name.json" > "$W/logs/$name.eval.log" 2>&1
  echo "=== done $name $(date +%T)"
}
one P2_imagenet_lrbb1e-5 --lr-backbone 1e-5 &
one P3_imagenet_layer3 --backbone resnet18_layer3 &
wait
echo "=== wave3 queue3 done $(date +%T)"
