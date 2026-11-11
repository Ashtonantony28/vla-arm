#!/bin/bash
# Wave-2 experiment queue 3: longer training (E1 resumed), Diffusion Policy on the same data and
# recipe, then language-conditioned ACT on sort (hashed bag-of-words STUB encoder).
# E1 (joint state) and E3 (+EE) were tied (2/20 vs 1/20), so the simpler E1 recipe is the base.
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
mkdir -p "$W/runs" "$W/logs"

# wait for queue 2's last training run to finish (its eval may still be running)
while [ ! -f "$W/runs/E4_resnet18_ee_tiny/train_meta.json" ]; do sleep 30; done

run() {  # name note task eval-args -- train-args...
  name=$1; note=$2; task=$3; evargs=$4; shift 4
  echo "=== $name $(date +%T)"
  $P -m policies.train "$@" --out "$W/runs/$name" > "$W/logs/$name.train.log" 2>&1
  OMP_NUM_THREADS=1 nohup $P -m policies.run_experiment --name "$name" --note "$note" --task "$task" $evargs \
      --results "$W/results.jsonl" --out-root "$W/runs" --eval-threads 1 --skip-train -- "$@" \
      > "$W/logs/$name.eval.log" 2>&1 &
}

run E5_resnet18_6k "E1 resumed for 3000 more steps (6000 total)" pick_place "" \
    --policy act $BASE --steps 3000 --init-checkpoint "$W/runs/E1_resnet18/pretrained_model"
run DP1_resnet18 "Diffusion Policy, same data/recipe as E1, 40-min budget" pick_place "" \
    --policy diffusion $BASE --steps 20000 --time-limit-min 40
run L1_act_lang_sort "ACT + hashed-BoW (STUB) instruction token on sort, 100 DR demos" sort \
    "--instructions train heldout" \
    --policy act_lang --datasets "$D/vla_arm_sim_sort_dr" --batch-size 16 --threads 2 --save-every 1000 \
    --text-encoder hashed_bow --steps 20000 --time-limit-min 40
wait
echo "=== queue3 done $(date +%T)"
