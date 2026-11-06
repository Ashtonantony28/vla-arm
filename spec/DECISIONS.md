# Decisions (locked 2026-11-06 from the plan's recommendations; user approved the plan)

- **D1 Joint count:** 5 arm joints + gripper. See `kinematics.yaml`.
- **D2 Hardware depth v1:** STS3215-class bus servos + own STM32G474 controller board. Custom FOC joints are v2.
- **D3 VLA latency fallback:** keep everything on-edge; if SmolVLA misses 10 Hz on the Orin Nano after INT8, ship text-conditioned ACT as the on-edge language policy and report SmolVLA numbers as measured.
- **D4 Visibility:** private by default until the user says otherwise.
- **D5 Owned equipment:** unknown; BOM assumes nothing is owned.
