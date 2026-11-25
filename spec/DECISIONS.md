# Decisions (locked 2026-11-06 from the plan's recommendations; user approved the plan)

- **D1 Joint count:** 5 arm joints + gripper. See `kinematics.yaml`.
- **D2 Hardware depth v1:** STS3215-class bus servos + own STM32G474 controller board. Custom FOC joints are v2.
- **D3 VLA latency fallback:** keep everything on-edge; if SmolVLA misses 10 Hz on the Orin Nano after INT8, ship text-conditioned ACT as the on-edge language policy and report SmolVLA numbers as measured.
- **D4 Visibility:** private by default until the user says otherwise.
- **D5 Owned equipment:** unknown; BOM assumes nothing is owned.

## Spec v1.1 changes (2026-11-06, from wave 1 findings)
- shoulder_pitch max +1.30 → **+0.75 rad**: CAD shows the upper arm hits the base at ~+0.80.
- elbow_pitch max +2.70 → **+2.30 rad**: the wrist reaches the base beyond ~+2.36 with the shoulder level.
- payload 0.25 → **0.20 kg**: the elbow's continuous limit at full reach. The shoulder counterbalance spring is **required**, not optional (without it even an empty arm exceeds the shoulder's continuous rating).
- gripper current limit **1000 mA** (continuous rating), not stall.
- PSU: **12 V ≥ 12.5 A** (was 10 A).

## Spec v1.2 change (2026-11-24, from the CAD polish and full clash audit)
- shoulder_pitch max +0.75 → **+0.60 rad**. The CAD now uses the STS3215 datasheet geometry: its output is
  12.5 mm off the case centre, so the yaw servo case reaches r = 37.5 mm and no longer fits inside the
  61812's 60 mm bore. The base now carries a 61816 (80 x 100 x 10); the wider housing and turntable meet the
  upper arm at +0.627 rad. Every recorded sim dataset keeps the shoulder below -0.64 rad, so no data or
  policy is affected. The alternative that keeps +0.75 (yaw servo mounted inverted in the shoulder bracket,
  small bearing around a base hub) is a larger redesign, listed in hardware/README.md.

## Spec v1.3 change (2026-11-25, user decision after the counterbalance packaging check)
- **Shoulder servo STS3215 → STS3250** (`servo_overrides.shoulder_pitch`): same case and bus protocol,
  4.9 N*m stall (was 2.9), 74.5 g. **The counterbalance spring, its post, pulley and pin are removed.**
  Why: the spring that balances the arm stores ~1.9 J. Every music-wire spring that does this is 110-125 mm
  long at full extension, and the bracket offers ~80 mm. Taller posts, a moved pin, 2:1 reeving, parallel
  springs and smaller spring loads were all checked, and none fits. The other options were a ~150 mm rear
  spring tail sweeping the desk, or no spring with an STS3215, which cannot hold even the empty arm
  continuously at full reach. The user picked the STS3250.
- Consequence: at full reach the shoulder holds about 0.12 kg continuously (40 % of stall). 0.20 kg is an
  intermittent load at 48 % of stall, the same class as the elbow. The STS3250's stall and rated currents
  (4.2 A / 1.4 A) and its rated torque come from retailer listings and must be checked on the datasheet.
