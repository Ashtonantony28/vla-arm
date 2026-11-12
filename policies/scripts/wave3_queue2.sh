#!/bin/bash
# Wave-3 queue 2: language-conditioned ACT on `sort` with the real MiniLM sentence encoder (wave 2's
# L1 used the hashed bag-of-words stub) and the ImageNet ResNet-18, 6000 steps. Eval: 20 held-out DR
# layouts with seen and held-out paraphrases (same protocol as L1).
# Install weights first: python -m policies.pretrained --resnet18 ... --minilm ...
set -u
REPO=$(cd "$(dirname "$0")/../.." && pwd)
cd "$REPO"
export MUJOCO_GL=egl PYTHONPATH=$REPO
P=${P:-python}
W=${W:-$REPO/outputs/wave3}
T=${T:-3}
D=$W/datasets
mkdir -p "$W/runs" "$W/logs"
name=L2_act_lang_minilm_sort
ARGS="--policy act_lang --datasets $D/vla_arm_sim_sort_dr --batch-size 16 --threads $T --save-every 1000 \
  --text-encoder minilm --pretrained-backbone --steps 6000"
echo "=== train $name $(date +%T)"
OMP_NUM_THREADS=$T $P -m policies.train $ARGS --out "$W/runs/$name" > "$W/logs/$name.train.log" 2>&1
OMP_NUM_THREADS=$T $P -m policies.run_experiment --name $name --task sort --instructions train heldout \
  --note "ACT + MiniLM instruction token + ImageNet ResNet-18 on sort, 100 DR demos, 6000 steps" \
  --results "$W/results.jsonl" --out-root "$W/runs" --eval-threads $T --skip-train -- $ARGS > "$W/logs/$name.eval.log" 2>&1
echo "=== wave3 queue2 done $(date +%T)"
