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
