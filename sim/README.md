# sim/ — MuJoCo twin, tasks, scripted experts, LeRobot recording

Everything here is generated from `spec/kinematics.yaml`; nothing in `sim/` hard-codes a link
length, joint limit, mass or servo rating.

## Setup

```bash
cd sim
python3 -m venv .venv                                   # gitignored
.venv/bin/pip install torch torchvision                  # see note on CPU wheels below
.venv/bin/pip install -r requirements.txt                # mujoco, lerobot[dataset,diffusion], pytest, ...
export MUJOCO_GL=egl                                     # offscreen rendering (conftest sets this)
```

What the build machine got (Python 3.13.18, 4 CPU cores, no GPU): mujoco 4.0.0, torch 2.15.0,
torchvision 0.29.2, lerobot 0.7.0 (+ datasets 5.1.0, diffusers 0.39.0, av 15.1.0, torchcodec 0.17.0),
numpy 2.2.6 (lerobot downgraded it from 2.5), gymnasium 1.4.0, pytest 9.2.0.

* `download.pytorch.org` is blocked on that machine, so the CPU-only wheel index was unusable and
  torch came from PyPI (the CUDA build, ~4 GB of `nvidia-*` libraries, runs fine on CPU). On a
  machine with access prefer `--index-url https://download.pytorch.org/whl/cpu`.
* EGL rendering needed `apt-get install libegl1 libegl-mesa0 libgl1-mesa-dri`; OSMesa did not work
  with this mujoco build, so stay with `MUJOCO_GL=egl`.

All commands below run from the repo root with `sim/.venv/bin/python`.

## Model — `sim/model/`

```bash
python -m sim.model.build_mjcf            # regenerate sim/model/vla_arm.xml from the spec
python -m sim.model.build_mjcf --check    # CI: fails if the XML is stale (also a pytest)
```

* Links: capsules/boxes with the spec masses (`masses_kg`; the gripper mass is split palm 70%,
  fingers 15% each). Joint axes, origins, limits from the spec. TCP site at `tool.tcp`.
* Actuators: MuJoCo `position` actuators, `forcerange = ±stall_torque_nm` (2.9 N·m),
  `kp = stall / 0.02 rad` (torque saturates after ~1.1° error, like the servo's P loop),
  critically-damped `kv`, armature 0.01 kg·m² (rotor inertia × gear ratio², ~1:345).
  The env ramps setpoints at sim rate and rate-limits them to the spec `vmax`.
* Gripper: two slide fingers coupled by an equality constraint; the actuator takes the servo angle
  (0 closed … 1.6 rad open) and maps it linearly to 0 … 60 mm jaw gap (`opening_m_at_max`). Its
  force limit is the *continuous* torque (stall × 1.0 A / 2.7 A = 1.07 N·m), on the assumption that
  the firmware limits holding current. TPU-like friction (1.5) on the fingers, elliptic cones.
* Scene: table plane at z = 0 (the base plate stands on it), red/blue 30 mm cubes (30 g, free),
  green/yellow bowls (mocap, repositioned each reset), a cup (free) and a target mark (visual).
  Objects a task doesn't use are parked off-table at x = 3 m.
* Cameras: `wrist` on the gripper body (behind the palm, pitched 40° toward the jaws, fovy 80) and
  a fixed `scene` camera front-right of the arm.
* Tests check: XML up to date, masses, zero-pose TCP = (0.490, 0, 0.100), each actuator moves its
  own joint in the documented positive direction (yaw +x→+y, pitch down, roll right-hand about x,
  gripper positive opens), FK (`sim/kinematics.py`) == MuJoCo, closed-form top-down IK round trip.

## Env — `sim/envs/`

```python
from sim.envs.tabletop import TabletopEnv
env = TabletopEnv("pick_place", image_size=96)          # "stack", "sort"; image_size int or (h, w)
obs, info = env.reset(seed=0)
obs, reward, terminated, truncated, info = env.step(action6)
```

* Observation: `observation.state` (6 joint positions, rad, order
  base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll, gripper),
  `observation.images.wrist|scene` (uint8 HxWx3), `task` (instruction string).
* Action: 6 absolute joint targets at 30 Hz; 20 physics substeps (dt = 1/600 s) in between.
* Tasks (`sim/envs/tasks.py`) with 5 paraphrases each; the first 3 are used for data, the last 2 are
  held out (`instruction_split="heldout"`):
  * `pick_place` — red block into the (green) bowl; blue block is a distractor.
  * `stack` — red block on the blue block.
  * `sort` — red and blue into the green/yellow bowls; the colour→bowl assignment is random and
    only given by the instruction (so the task needs language).
  * 30% of resets also place the cup and/or the target mark as distractors.
* Success: block centre inside the bowl's inner radius and below the rim; or red on blue within
  15 mm xy and 30 ± 6 mm z with blue on the table. In every case the block must not be touching
  the fingers (it was released).

## Scripted experts — `sim/expert/`

Privileged-state experts: Cartesian waypoints (approach → descend → close → lift → transfer → lower
→ open → retreat), min-jerk interpolation, closed-form IK, jaw aligned to the nearest cube face,
noise on waypoints (2–4 mm), segment durations (±15%) and joint targets (σ = 0.002 rad).

```bash
python -m sim.expert.evaluate --episodes 100
```

Result (seeds 0–99): pick_place 100/100, stack 100/100, sort 100/100
(Wilson 95% CI 96–100% each); mean 168 / 168 / 356 steps.

## Recording — `sim/record.py`

```bash
python -m sim.record --task all --episodes 30 --image-size 96 --overwrite
```

Uses LeRobot's own `LeRobotDataset.create / add_frame / save_episode / finalize` (dataset format
**v3.0**: chunked parquet + AV1 MP4 per camera, `meta/info.json`, `meta/stats.json`,
`meta/tasks.parquet`, `meta/episodes/`). Extra sidecars: `meta/sim_episodes.jsonl` (seed, layout,
instruction per episode) and `meta/sim_record_summary.json`. Only successful episodes are kept;
10 frames after success are kept so policies learn to stop. Recording seeds start at 0, eval
seeds at 900 000.

Recorded on the build machine into `outputs/datasets/` (gitignored), 96×96:

| dataset | episodes | frames | size | time |
|---|---|---|---|---|
| vla_arm_sim_pick_place | 30 | 5 333 | 11.4 MB | 4.6 min |
| vla_arm_sim_stack | 30 | 5 332 | 10.1 MB | 4.1 min |
| vla_arm_sim_sort | 30 | 11 014 | 25.1 MB | 10.3 min |

## Tests

```bash
sim/.venv/bin/python -m pytest -q sim/tests policies/tests
```

## Real vs simplified

* Real: geometry, limits, masses, stall torque, vmax from the spec; contact physics; cameras.
* Simplified: link shapes are primitives; servo = ideal saturating PD (no backlash, deadband,
  compliance, temperature); gripper linkage is linear; the expert uses privileged object poses;
  bowls are fixed during an episode; lighting/texture are not randomised yet (PLAN 5 asks for it).
