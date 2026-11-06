# armctl

Python host driver for the VLA arm controller board (5 arm joints + gripper), implementing
`spec/protocol.md` v1 and the kinematics in `spec/kinematics.yaml`. Runtime deps: numpy,
pyserial, pyyaml.

```
armctl/
  protocol.py     CRC-16/CCITT-FALSE, frame encode, streaming decoder with resync, message dataclasses
  kinematics.py   load kinematics.yaml, FK, 6x5 Jacobian, analytic IK (all branches) + DLS refinement, gripper rad<->m
  client.py       ArmClient: reader thread, STATE cache, ACK matching by SEQ, retries, heartbeat
  safety.py       SafetyShim: joint-limit clamp, per-step velocity limit, table plane / base keep-out
  calibration.py  calibration YAML, offset/direction procedure, leader->follower mapping, push via CONFIG
  mockboard.py    in-process fake board (spec-faithful) + serial-like MockTransport
  teleop.py       leader-follower loop (unit-testable with two mock boards)
  cli.py          armctl-ping, armctl-teleop
tests/            pytest + hypothesis (protocol, kinematics, safety, mock, client, calibration, teleop)
```

## Install and test

```bash
cd armctl
python3.13 -m venv .venv
.venv/bin/pip install -e '.[test]'
.venv/bin/pytest            # ~15 s; the client/teleop tests run the mock board in real time
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

Applied by `ArmClient.set_targets` (if a shim is configured) and by the teleop loop:

1. clamp every joint to `kinematics.yaml` limits (optional margin);
2. per-joint step from the last sent command ≤ `vmax × vel_scale × move_ms`;
3. TCP and wrist (wrist-pitch axis) `z ≥ 0.01 m`, TCP outside the `r = 0.07 m` cylinder around
   the base axis. The joint-space segment from the last command is sampled (default 5 points);
   if it leaves the workspace the step is shortened by bisection to the last safe fraction.
   With no previous command an unsafe target is rejected. If the previous pose was itself
   unsafe (e.g. the arm slumped on the table at power-up), steps that do not increase the total
   violation depth are allowed so the arm can be brought back.

Every modification adds a human-readable reason to the returned `SafetyReport`.

### Calibration

Board mapping assumed (see interpretations): `q = direction × (ticks − 2048 − offset_ticks) × 2π/4096`.
Procedure (`calibrate_at_known_pose`): with torque off and the arm placed at a known pose
(jig), push identity (offset 0, dir +1), read positions, compute
`offset = rel_ticks − q_known / (dir·k)`, push, verify. Directions come from assembly or from
two known poses (`compute_directions`). `Calibration.push` refuses while torque is on (a mapping
change under torque would jump the arm). The leader file can carry `leader_to_follower`
(per-joint `scale`/`offset_rad`, gripper handle range → follower 0–1.6 rad); see the docstring
in `calibration.py` for the YAML layout.

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

## Known gaps

* Not yet cross-tested against the C firmware; the mock implements the interpretations above.
* No Pinocchio/ikpy cross-check (PLAN §3.4); FK/IK are checked against hand-derived poses,
  finite-difference Jacobians and round trips.
* The safety shim checks only the TCP and wrist points (as specified), not the elbow, fingers or
  self-collision; the path check samples the joint-space line, not the board's quintic.
* Mock physics: first-order servo lag, no gravity, backlash, thermal or real current model;
  OVERCURRENT/OVERTEMP/SERVO_COMMS only via `inject_fault`.
* `SerialTransport` is tested over a pseudo-terminal, not a real USB-CDC device.
* The client/teleop tests use wall-clock timing (tolerances are generous; they passed 4×
  in parallel on a loaded 4-core box).
* LeRobot robot-class integration is out of scope here.
