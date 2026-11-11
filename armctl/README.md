# armctl

Python host driver for the VLA arm controller board (5 arm joints + gripper), implementing
`spec/protocol.md` v1 and the kinematics in `spec/kinematics.yaml`. Runtime deps: numpy,
pyserial, pyyaml. Optional: `lerobot` (extra `[lerobot]`, the LeRobot plugins) and
`pin` + `ikpy` (extra `[crosscheck]`, test-only).

```
armctl/
  protocol.py     CRC-16/CCITT-FALSE, frame encode, streaming decoder with resync, message dataclasses
  kinematics.py   load kinematics.yaml, FK, 6x5 Jacobian, analytic IK (all branches) + DLS refinement, gripper rad<->m
  client.py       ArmClient: reader thread, STATE cache, ACK matching by SEQ, retries, heartbeat
  safety.py       SafetyShim: limits, velocity, workspace + collisions along the board's planned path; check_chunk
  collision.py    CollisionModel: capsules per link vs each other, the base and the table (vectorised)
  trajectory.py   FirmwareTrajectory: Python twin of firmware/src/traj.c (quintic replan, stretch, blend)
  urdf.py         URDF of the chain generated from kinematics.yaml
  data/collision_model.yaml   capsule radii fitted to the CAD (sources cited in the header)
  lerobot/        LeRobot 0.7.0 Robot (follower), Teleoperator (leader), fake camera, CLI wrappers
  calibration.py  calibration YAML, offset/direction procedure, leader->follower mapping, push via CONFIG
  mockboard.py    in-process fake board (spec-faithful) + serial-like MockTransport
  teleop.py       leader-follower loop (unit-testable with two mock boards)
  cli.py          armctl-ping, armctl-teleop
tools/            dump_cad_links.py + fit_capsules.py (regenerate data/collision_model.yaml from the CAD)
tests/            pytest + hypothesis; test_lerobot.py / test_crosscheck.py skip without their extras
../tests/integration/   armctl vs the compiled firmware (libarmfw): protocol, and twin vs STATE cmd_pos_crc
```

## Install and test

```bash
cd armctl
python3.13 -m venv .venv
.venv/bin/pip install -e '.[test]'
.venv/bin/pytest            # ~30 s; the client/teleop tests run the mock board in real time
# optional suites (skipped otherwise):
.venv/bin/pip install -e '.[crosscheck]'   # Pinocchio + ikpy -> tests/test_crosscheck.py
.venv/bin/pip install -e '.[lerobot]'      # or run test_lerobot.py from an env that has lerobot 0.7
```

Firmware cross-tests (from the repo root; need cmake and a C compiler):

```bash
cmake -S firmware -B firmware/build && cmake --build firmware/build
PYTHONPATH=armctl armctl/.venv/bin/pytest tests/integration
```

`kinematics.yaml` is found by walking up from the package to `spec/kinematics.yaml`
(works for the repo checkout / editable install). Elsewhere set `ARMCTL_KINEMATICS=/path/to/kinematics.yaml`.

## Usage

```bash
armctl-ping /dev/ttyACM0          # PONG + one STATE
armctl-ping mock                  # same against the in-process mock
armctl-teleop --leader /dev/ttyACM1 --follower /dev/ttyACM0 \
    --leader-cal leader.yaml --follower-cal follower.yaml --rate 100
```

```python
import numpy as np
from armctl import ArmClient, SafetyShim, ik
from armctl.protocol import Mode

with ArmClient.open_serial("/dev/ttyACM0", safety=SafetyShim()) as arm:   # torque off on exit
    print(arm.ping())
    arm.enable()                                       # torque on, holding the measured pose
    q = ik([0.25, 0.05, 0.08], pitch=np.pi / 2, roll=0.0, q_ref=arm.positions()[:5])
    arm.set_targets(np.append(q, 1.0), move_ms=1500)   # 6 values: 5 joints + gripper (rad)
    arm.wait_for_state(lambda s: s.mode == Mode.ACTIVE)
    arm.estop()                                        # latches ESTOP_SOFT
    arm.clear_faults()                                 # step 1: clear latched bits
    arm.enable()                                       # step 2: torque-enabled SET_TARGETS
```

Mock instead of hardware: `ArmClient(MockTransport(MockBoard()))`. The mock can also be driven
in simulated time without threads: `board.receive(bytes)`, `board.step(dt)`, `board.take_output()`.

### API notes

* All angles are radians at the API; the wire uses int16 mrad (rounded, saturated).
* `set_targets(q, move_ms, wait=True)` waits for the ACK (retries with a fresh SEQ; an ACK for
  any earlier attempt counts). `wait=False` is fire-and-forget with asynchronous NACK
  tracking (`on_nack` callback, `stats`), which the teleop loop uses.
* NACK raises `NackError` (with the code name), no ACK raises `ArmTimeout`, a command the safety
  shim refuses raises `SafetyRejected` and nothing is sent.
* Heartbeat: a background thread sends PING when nothing else was sent for `heartbeat_period`
  (default 40 ms, below the 100 ms HOLD threshold). Because a background heartbeat would keep
  feeding the watchdog even if the application loop hangs, `heartbeat_max_idle=<s>` stops it
  after that long without an application command. `pause_heartbeat()` / `resume_heartbeat()`
  exist for tests.
* The client keeps its own idea of "torque on" (set from the ACK of a torque-enable
  SET_TARGETS, cleared on torque-off/estop or a later STATE in IDLE/FAULT) and re-seeds the
  safety shim from measured positions whenever torque is being (re-)enabled.

### Kinematics

* `fk(q)` → 4x4 TCP pose; `joint_positions(q)` → shoulder/elbow/wrist/roll/tcp points;
  `jacobian(q)` → geometric 6x5 `[v; w]` in the base frame.
* `solve_ik(T)` / `solve_ik_xyz(p, pitch, roll)` → `IKResult` with every in-limit branch
  (`front|back` × `elbow_up|elbow_down`), each polished by DLS; rejected branches and the
  reason for "unreachable" are reported. `pitch` is the tool x-axis angle below horizontal
  (positive = pointing down, same sign as the joint pitches), the tool heading is radially
  outward from the base axis, `roll` is the wrist roll joint. `ik(...)` returns the branch
  closest to `q_ref` or raises `Unreachable`.
* A 5-DOF arm cannot realise an arbitrary orientation: `solve_ik(T)` projects the requested
  rotation onto the arm plane (yaw from the target position) and reports `rot_err`;
  `solve_ik_numeric(T, seed)` / `refine_ik` give a weighted least-squares DLS solution.
* Gripper: `gripper_rad_to_opening` / `gripper_opening_to_rad` (linear, 0.060 m at 1.60 rad).

IK accuracy (random configurations uniformly inside the joint limits, `FK(IK(FK(q)))`, 20 000
samples): 100 % solved, the generating configuration is among the returned branches in 100 %
of cases, max position error 3.8e-16 m, max orientation error 2.8e-13 rad, ~0.7 ms per solve
(all branches). The test suite runs 5 000 samples.

### Safety shim

Applied by `ArmClient.set_targets` (if a shim is configured), the teleop loop and the LeRobot
follower. `filter(q, dt, now=...)` does, in order:

1. clamp every joint to `kinematics.yaml` limits (optional margin);
2. per-joint step from the last sent command ≤ `vmax × vel_scale × dt`;
3. **check the path the board will actually run**: the trajectory twin (below) is advanced from
   the previous command to `now` and replanned to the new target exactly like the firmware
   does (from the current commanded position/velocity/acceleration, with duration stretching
   and blends), and its setpoints are sampled every 10 ms to the end of the move. Every sample
   must pass the point checks (TCP and wrist-pitch axis `z ≥ 0.01 m`, TCP outside the
   `r = 0.07 m` cylinder around the base axis) and the capsule collision check (self, base,
   table; 5 mm link clearance by default). If not, the step is shortened by bisection (12
   iterations, each replanned) to the largest safe fraction; if even holding the previous
   target is unsafe (the board is already committed to an unsafe plan) the command is
   rejected. With no previous command an unsafe target is rejected. If the previous pose was
   itself unsafe (arm slumped on the table at power-up), steps that do not increase the total
   violation depth are allowed so the arm can be brought back.

Why the trajectory matters: a command whose straight joint-space line is safe, sent while the
board is still moving fast from the previous one, makes the firmware replan a quintic from that
(p, v, a) state, which can swing well past the line.
`test_trajectory_check_catches_mid_move_replan_overshoot` has a case where the straight-line
check passes a command whose executed plan goes more than 1 cm into the table; the twin check
scales it back and the resulting plan stays clear. `SafetyConfig(trajectory=False)` restores the
straight-line check, `collision=False` the point-only checks.

`check_chunk(Q)` screens a whole policy action chunk `(T, 6)` in one batched call (limits,
point checks, capsules per step) without touching shim state, and returns per-step `ok`, depth,
clearance, the first bad index and a reason.

Every modification adds a human-readable reason to the returned `SafetyReport`.

Cost per command (teleop-style stream, 4-core box at load ~3-5): median 1.1-1.2 ms (the
point-only shim of the first version took ~0.6 ms).

### Collision model

`CollisionModel` (`collision.py`) represents every moving link as a union of capsules (segment +
radius) in its joint frame: shoulder bracket 3, upper arm 11 (clevis disc, two side plates with
the hollow between them, bridge, counterbalance pin, neck, elbow cradle), forearm 8, wrist 4,
gripper body 9 (incl. the wrist camera), each jaw 3, the jaws sliding ±18.75 mm/rad of gripper
angle. The base is static boxes (plate, clamp tabs, ribs) and vertical cylinders (housing, and
the turntable, which yaws with joint 0). The table is `z = 0`. Checks: all non-adjacent link
pairs, every link vs the base, every link except the yaw bracket vs the table.

The numbers are in `data/collision_model.yaml`, whose header cites the sources: segments laid
out from `hardware/cad/params.py`, radii **fitted** to the CAD (`tools/dump_cad_links.py`
tessellates every assembly item in its link frame; `tools/fit_capsules.py` takes the max
distance of the assigned points, rounded up to 0.1 mm). Every CAD point of every link lies
inside its capsules and all base points inside the static primitives, so the model is
conservative (≈ 2 mm over the clevis plates, up to ≈ 10 mm at box corners).

Distances: capsule–capsule exact (segment–segment); capsule–box/cylinder via a bracketing-grid
lower bound (never overestimates; within 0.16 % of the segment length); bounding spheres cull,
so clearances below `exact_below` (1 cm) are exact and larger ones are lower bounds.
`clearance(Q, details=True)` names every failing pair.

Reproduction of the CAD findings (`hardware/cad/out/cad_report.md`), one joint swept, others 0:

| finding | capsule model | CAD point cloud (exact) | CAD report |
|---|---|---|---|
| elbow folded down, wrist meets the base | 2.366 rad (wrist.disc vs base.plate) | 2.367 | free to 2.363, contact in [2.363, 2.383] |
| shoulder down, upper arm meets the base | 0.797 rad (upper_arm.bridge vs base.plate) | 0.811 | free to 0.802, contact in [0.802, 0.822] |
| arm stretched, shoulder down, gripper meets the table | 0.165 rad | 0.176 | – |

(The elbow *axis* reaches the table at asin(0.1/0.2) = 0.524 rad; the gripper touches it much
earlier.) Over uniformly random in-limit poses, 2.8 % are in self-collision and 32.5 % collide
with something (self, base or table): the joint limits alone are not a safe set.

Speed (numpy, 50-pose chunk, 4-core box at load ~3-5, median of 200): smooth chunk 1.4-1.5 ms,
typical workspace poses ~1.8 ms, uniformly random full-range poses ~2.1-2.3 ms;
`check_chunk` incl. point checks ~1.9 ms. `test_chunk_benchmark` prints the number (loose 6 ms
guard for loaded CI).

### Trajectory twin

`FirmwareTrajectory` (`trajectory.py`) ports `firmware/src/traj.c` to float64 numpy, in mrad:
quintic replan from the current commanded (p, v, a), vmax duration search (secant search,
common duration), constant-jerk blend when stretching cannot bring the peak under vmax,
saturation at the limits with per-joint re-planning, 1 kHz stepping. `tests/test_trajectory.py`
compiles `traj.c` with `cc` and compares 900 random replans (803 stretched, 28 blended, 12 with
overshoot/saturation): worst difference 0.004 mrad, identical flags.
`tests/integration/test_trajectory_vs_firmware.py` runs the real firmware (libarmfw via ctypes,
virtual time) and checks that the CRC of the twin's setpoints equals STATE `cmd_pos_crc` in every
STATE frame through 120 random replans and a 100 Hz teleop stream (setpoints within 0.02 mrad of
an int16 rounding boundary are allowed either way: the firmware computes in float32).

### Kinematics cross-check

`tests/test_crosscheck.py` (needs `pin`, `ikpy` from PyPI; Pinocchio 4.1.0 and ikpy 4.5.0 used):
`fk` and `jacobian` vs Pinocchio on the URDF generated by `armctl.urdf` *and* on the sim's
hand-written MJCF `sim/model/vla_arm.xml`, a `J·dq` vs FK-difference check, ikpy FK on the URDF,
and ikpy IK solutions verified with armctl FK. Worst disagreement over 400 random in-limit poses:
3.3e-16 (position, rotation and Jacobian entries) for both model sources.

### Calibration

Board mapping assumed (see interpretations): `q = direction × (ticks − 2048 − offset_ticks) × 2π/4096`.
Procedure (`calibrate_at_known_pose`): with torque off and the arm placed at a known pose
(jig), push identity (offset 0, dir +1), read positions, compute
`offset = rel_ticks − q_known / (dir·k)`, push, verify. Directions come from assembly or from
two known poses (`compute_directions`). `Calibration.push` refuses while torque is on (a mapping
change under torque would jump the arm). The leader file can carry `leader_to_follower`
(per-joint `scale`/`offset_rad`, gripper handle range → follower 0–1.6 rad); see the docstring
in `calibration.py` for the YAML layout.

### LeRobot

`armctl.lerobot` implements LeRobot **0.7.0**'s `Robot`, `Teleoperator` and `Camera`
interfaces (checked against the installed package source):

* `ArmctlFollower` / `ArmctlFollowerConfig` (`--robot.type=armctl_follower`): observation
  `{"base_yaw.pos", …, "gripper.pos"}` in rad (protocol order; gripper in servo rad 0–1.6) plus
  one `(h, w, 3)` image per configured camera; action: the same six absolute targets.
  `connect()` enables torque holding the measured pose (refuses a board with latched faults);
  `send_action()` runs the safety shim, sends a fire-and-forget SET_TARGETS with
  `move_ms = 2 / control_fps` and returns what was sent; a rejected action keeps the previous
  target (or raises with `raise_on_reject`). `supports_position_hold = True`.
  Options: `port`, `calibration_file` (armctl YAML pushed on connect), `move_ms`, `safety`,
  `vel_scale`, `collision_margin_m`, `max_relative_target`, `cameras`, ….
* `ArmctlLeader` / `ArmctlLeaderConfig` (`--teleop.type=armctl_leader`): the leader board's latest
  STATE mapped through its calibration's `leader_to_follower`; torque off; no feedback.
* `ArmctlFakeCamera` (`type: armctl_fake`): synthetic frames for dry runs.
* Ports: a serial device, `mock` (private in-process mock board) or `mock:<name>` (shared,
  reachable via `armctl.lerobot.mock_board(name)`, e.g. to backdrive a mock leader).

LeRobot only auto-imports plugins from distributions named `lerobot_robot_*` etc., so use the
wrappers, which import `armctl.lerobot` first and then run LeRobot's own CLI:

```bash
armctl-lerobot-record \
  --robot.type=armctl_follower --robot.port=/dev/ttyACM0 \
  --robot.cameras='{wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \
  --teleop.type=armctl_leader --teleop.port=/dev/ttyACM1 \
  --dataset.repo_id=me/vla_arm_pick --dataset.single_task="pick up the red block" \
  --dataset.num_episodes=10 --dataset.push_to_hub=false
armctl-lerobot-teleoperate --robot.type=armctl_follower --robot.port=mock \
  --teleop.type=armctl_leader --teleop.port=mock
```

`tests/test_lerobot.py` runs `armctl-lerobot-record` end to end with two mock boards and two fake
cameras: 2 episodes × 1.2 s at 30 fps into a temp dir, reloaded with `LeRobotDataset`; it checks
the features (names, shapes, image dtype), fps, robot type, episode lengths, 1/30 s timestamps,
item shapes, and that the recorded follower state tracks the recorded leader action.

### Teleop

`TeleopLoop(leader, follower, mapping, safety, rate_hz=100)`: the leader is a second board of the
same type with torque off, streaming STATE. Each `step()` takes the latest leader STATE (skips if
older than `max_leader_age`), maps it, runs the shim and sends a fire-and-forget SET_TARGETS with
`move_ms = 2 × period` so consecutive quintics blend. A follower in FAULT raises
`FollowerFaulted`. With two mock boards and a 0.5 Hz ±0.5 rad leader motion at 100 Hz the follower
tracks within 0.06 rad peak / 0.02 rad mean (pipeline lag ≈ 40 ms).

## Spec interpretations

Places where `spec/protocol.md` / `kinematics.yaml` leave a choice open; the mock and the
client implement these choices (marked `INTERP` in `mockboard.py`). The C firmware should be
checked against this list when cross-testing.

1. **Wrong payload length for a known TYPE.** The spec both says such a frame "is dropped" and
   defines NACK code 1 "bad length". Mock: a CRC-valid frame with a known host TYPE but wrong
   LEN gets NACK 1; bad CRC / bad VER / LEN > 240 are dropped silently. The host decoder drops
   board frames with a wrong length (counted in `stats.decode_errors`).
2. **Board→host TYPE sent to the board** (e.g. 0x82) → NACK 3 (unknown type).
3. **What counts as a "valid host frame" for the watchdog / `last_host_seq`:** every CRC-valid
   frame, including ones that are then NACKed.
4. **Watchdog only armed while torque is on** (ACTIVE/HOLD). An IDLE board (e.g. the limp leader
   arm) never latches WATCHDOG. Thresholds are strict: HOLD when silent `> watchdog_hold_ms`,
   FAULT when silent `> watchdog_off_ms` (measured from the last frame, not from entering HOLD).
5. **HOLD:** the in-progress quintic is abandoned and the current commanded position is frozen
   (velocity/acceleration zeroed). Any valid host frame returns the board to ACTIVE, still
   holding that frozen setpoint until the next SET_TARGETS.
6. **Two-step clear:** after CLEAR_FAULT removes every latched bit the mode byte stays FAULT
   (torque off) with `fault_flags == 0` until a torque-enabled SET_TARGETS → ACTIVE.
   SET_TARGETS with torque *off* is accepted while faulted (mode stays FAULT if latched bits
   remain, else IDLE).
7. **"Rejected while faulted" (NACK 4)** applies only to torque-enabled SET_TARGETS. CONFIG,
   CLEAR_FAULT, SET_STREAM, ESTOP, PING are accepted in FAULT.
8. **ESTOP_HW** is treated as latched (it needs a host clear). CLEAR_FAULT while the loop is
   still open is ACKed but leaves the bit set.
9. **BUS_UNDERVOLT** is not marked latched: the mock sets/clears it with the bus voltage and it
   does not by itself force FAULT or torque off. CLEAR_FAULT cannot clear it.
10. **Torque enable from limp** (IDLE/FAULT → ACTIVE): the commanded state starts at the measured
    position with zero velocity/acceleration (the spec's "current commanded state" is undefined
    while limp).
11. **vmax stretching** uses one common duration for all joints. With a mid-move replan
    (non-zero initial velocity/acceleration) stretching does not always bring the peak under
    vmax (and the starting speed itself may exceed the new joint's vmax); the mock searches a
    bounded set of durations and takes the first that satisfies vmax, else the lowest peak.
12. **LIMIT_CLAMP self-clear:** cleared by the next SET_TARGETS that needed neither a position
    clamp nor a duration stretch.
13. **Offset/direction formula** (not in the spec):
    `joint_rad = direction × (ticks − center_tick − offset_ticks) × 2π / ticks_per_rev`,
    `center_tick`/`ticks_per_rev` from `kinematics.yaml`.
14. **`joint_status` bits** are not defined. Mock: bit0 = torque on, others 0. The host does not
    interpret them.
15. **`cmd_pos_crc`:** CRC-16/CCITT-FALSE over the six current trajectory setpoints (this
    cycle's commanded position, not the final target) as int16 LE mrad.
16. **CONFIG validation:** global keys (0x10/0x11) require joint 0xFF and per-joint keys require
    0–5 (no 0xFF broadcast), else NACK 2. NACK 5 is checked before the joint index. Ranges:
    direction ±1; `|offset| < ticks_per_rev`; `limit_min < limit_max`; vmax/current > 0;
    `1 ≤ hold_ms < off_ms ≤ 60000`.
17. **SET_STREAM** `1..999` → NACK 2. **SET_TARGETS** `move_ms` outside 1–5000 or reserved flag
    bits set → NACK 2, also when torque is off.
18. **CLEAR_FAULT** mask bits 8–15 are ignored.
19. **PONG has no SEQ echo**, so the client matches a PONG to the oldest outstanding `ping()`.
20. **Leader arm** is assumed to be a second controller board running the same firmware with
    torque off (PLAN §5 does not say what reads the leader servos).
21. **Tool pitch for IK** is the angle of the tool x-axis below horizontal (sum of the three
    pitch joints in the front branch), heading radially outward from the base axis.

### Possible issues in the spec (not edited)

* Items 1, 3, 5, 6 and 11 above are genuine ambiguities the firmware author should settle;
  item 1 is a direct contradiction between the framing paragraph and the NACK code table.
* PONG cannot be matched to its PING. Adding `u8 ping_seq` (or answering PING with an ACK plus
  PONG) would fix it.
* `t_us` in STATE is u32 µs and wraps every ~71.6 min; consumers must unwrap it.
* At the default 200 Hz, STATE is 74 bytes × 200 = 14.8 kB/s. Over a plain UART at 115 200 baud
  ("the same framing works over a plain UART for bring-up") that is ~128 % of the link; bring-up
  over UART needs ≥ 230 400 baud or a slower stream.
* Found while building the trajectory twin and the collision model (spec v1.1, not edited):
  * Only `firmware/src/traj.c` fully defines the trajectory: the duration search (secant, 8
    iterations, 0.999 goal, at most 8× per step), how blend durations are chosen, and that a
    saturated joint is replanned on its own from the limit (rest-to-rest quintic, at least the
    vmax-feasible duration) are not in spec §9. A host that predicts the board's motion (as the
    safety shim does) has to follow `traj.c`, so changes there should be treated as protocol
    changes or written into the spec.
  * `cmd_pos_crc` does not say how setpoints are rounded to int16 mrad, or at what precision. The
    firmware rounds float32 values, so a float64 host twin disagrees in the last mrad when a
    setpoint is within ~0.02 mrad of a rounding boundary. Specifying round-half-away-from-zero
    on float32 (or streaming the setpoints) would make the check exact.
  * The joint limits are not a safe set: 32.5 % of uniformly random in-limit poses hit the
    table, the base or the arm itself, and stretched out the gripper reaches the table at
    shoulder +0.165 rad, far inside the +0.75 limit. The spec might say that a host-side
    collision check is required, not optional.

## Known gaps

* Protocol behaviour is cross-tested against the compiled firmware (`tests/integration`), but
  not on real hardware; the mock implements the interpretations above.
* Collision model: only the arm, the base and the table. No other obstacles (bowls, cups, the
  operator), nothing held in the gripper, jaws as rigid boxes. Capsules over-approximate box
  corners by up to ~10 mm. The CAD servo envelope it was fitted to is itself an approximation
  (hardware/README.md); rerun `tools/` when the CAD changes.
* The safety shim needs `now` to model mid-move replans; without it the previous move is
  assumed finished. Long gaps (> 50 ms) inside a move skip the saturation emulation.
* LeRobot: tested with mock boards and fake cameras only (no OpenCV camera, and no video
  encoding in the test: `--dataset.video=false`). `lerobot-record` stores the *teleop* action,
  not the action the shim actually sent; when the shim modifies a command the dataset holds the
  leader's command. Feature names carry LeRobot's `.pos` suffix (`base_yaw.pos`), while the sim
  datasets (`sim/record.py`) use bare joint names; order and units are the same.
* Mock physics: first-order servo lag, no gravity, backlash, thermal or real current model;
  OVERCURRENT/OVERTEMP/SERVO_COMMS only via `inject_fault`.
* `SerialTransport` is tested over a pseudo-terminal, not a real USB-CDC device.
* The client/teleop/LeRobot tests use wall-clock timing. On an oversubscribed box (8 busy
  processes on 4 cores) `test_teleop_follower_tracks_leader` can miss its 0.06 rad / 150-command
  bounds; the first version misses them too under that load, somewhat less often (its shim took
  ~0.6 ms per command, now ~1.1 ms).
