# VLA Arm — a low-cost 6-DOF arm driven by a vision-language-action policy

**Status:** PLAN, awaiting review. Nothing has been designed, ordered or coded.
**Drafted:** 2026-11-06
**One line:** build a ~$700 desktop 6-DOF arm with your own motor-controller PCB and firmware, collect teleoperated demonstrations with a leader arm, train ACT and diffusion policies, then fine-tune a small vision-language-action model so you can type "put the red block in the bowl" and the arm does it, with all inference running on a Jetson Orin Nano at the desk.

---

## 1. Why this project

It joins the threads your other projects already run on: low-level hardware (tiny-gpu-extended's RTL, the int8 inference path), from-scratch ML (nanoGPT-extended), and agent systems that turn language into actions (Praxis, AgentOps). Here the "action" is physical, so every layer has to work for real: PCB, firmware timing, kinematics, data quality, model size versus edge latency.

It also produces things that read well to a robotics or ML-systems reviewer: a board you designed, a firmware control loop with measured jitter, a public LeRobot-format dataset, trained policies with success-rate tables, and an edge-inference latency budget with numbers.

## 2. Goals

### Must reach (v1)
1. **Arm hardware:** 6 DOF (5 arm joints + gripper), ~45 cm reach, 250 g payload at full reach, repeatability within ±2 mm.
2. **Own controller board:** an STM32G4-based board that runs the servo bus, measures per-joint current, owns the safety logic (e-stop, current and position limits, watchdog) and talks to the host over USB at 500 Hz–1 kHz.
3. **Firmware:** deterministic control loop with logged jitter (<50 µs target at 1 kHz), trajectory interpolation between policy actions, safe-stop on host timeout.
4. **Teleop + data:** leader-follower teleop, 2 cameras (wrist + scene), recording to the LeRobot dataset format; ≥300 demos across 5 tasks.
5. **Policies:** ACT and Diffusion Policy trained on that data, each scored on ≥3 tasks with 20 trials per task.
6. **Language:** one language-conditioned policy (SmolVLA fine-tune is the default; see §6.4) that picks the right task from a typed instruction across ≥5 tasks.
7. **Edge:** the deployed policy runs on the Jetson at ≥10 Hz action-chunk rate with the whole loop on the desk, no cloud.

### Stretch (v2)
- **Custom joint actuators:** replace the shoulder and elbow bus servos with your own FOC BLDC modules (gimbal motor + magnetic encoder + 3D-printed cycloidal reducer) on CAN-FD, driven by a second board you design. This is the biggest jump in hardware depth and the biggest risk, so it sits after v1 works.
- Voice input (local Whisper-small on the Jetson).
- Force-aware grasping from joint current (contact detection, slip retry).
- Sim twin in MuJoCo for pre-training and regression tests of new policies.

### Out of scope
Mobile base, bimanual, ROS 2 as a hard dependency (a thin ROS 2 bridge can be added later), and commercial safety certification.

## 3. System architecture

```
 typed / spoken instruction
            │
 ┌──────────▼────────────────────── Jetson Orin Nano Super (8 GB) ──────────────────────┐
 │  camera capture (wrist + scene, 30 fps)                                              │
 │        │                                                                             │
 │  policy server ── SmolVLA (TensorRT, FP16/INT8) or ACT/DP ── action chunk (50 × 6)   │
 │        │                                                                             │
 │  chunk scheduler (temporal ensembling, 30–50 Hz setpoints) ── safety shim (limits)   │
 └────────┼─────────────────────────────────────────────────────────────────────────────┘
          │ USB-CDC, binary framed protocol, CRC16, 500 Hz–1 kHz
 ┌────────▼───────────── Arm controller PCB (STM32G474) ───────────────┐
 │  1 kHz loop: setpoint interpolation → servo bus write → read back   │
 │  pos / vel / current / temp; INA240 bus-current sense; e-stop input │
 │  watchdog: host silent > 100 ms → hold, > 500 ms → torque off       │
 └────────┬──────────────────────────────────────────────────┬─────────┘
          │ half-duplex TTL bus @ 1 Mbps                     │ (v2) CAN-FD @ 5 Mbps
     6 × serial bus servos (follower arm)          custom FOC joint modules
```

### 3.1 Mechanical
- Own CAD (Onshape or FreeCAD), 3D-printed PLA+/PETG links, sized around 12 V serial bus servos (Feetech STS3215-class, ~30 kg·cm stall). The open SO-100/SO-101 arms prove this servo class works for tabletop manipulation; we use them as a reference for proportions only and design our own links, wrist and gripper.
- Joint layout: base yaw, shoulder pitch, elbow pitch, wrist pitch, wrist roll, gripper. That is 5 DOF + gripper; "6-DOF" counts the gripper, as is usual in this class. A true 6-axis arm (extra wrist yaw) is possible but adds weight at the end of the chain and makes the leader arm harder; I recommend 5+1 (decision D1).
- Gripper: parallel jaw with TPU fingertips, wrist camera mounted on the gripper body.
- Shoulder gets a spring counterbalance to cut holding torque, which matters for servo heat during long data sessions.

### 3.2 Electronics (the custom board)
| Block | Choice | Why |
|---|---|---|
| MCU | STM32G474 (170 MHz Cortex-M4F, CORDIC, FDCAN) | Fast enough for 1 kHz plus math; FDCAN ready for v2 modules |
| Servo bus | UART half-duplex via 74LVC1G126 buffer / SN74LVC2G241, 1 Mbps | Bus servo protocol, DMA-driven |
| Host link | USB-FS CDC (later USB-HS via external PHY only if needed) | Simple, ~1 MB/s is plenty |
| Power | 12 V 10 A input, ideal-diode reverse protection, TVS, bulk caps, 5 V/3.3 V buck | Servo stall surges are the main failure source |
| Sensing | INA240 high-side current shunt on the servo rail, NTC on board | Independent check of what servos report |
| Safety | Hardware e-stop loop that cuts servo power through a MOSFET, independent of firmware | A firmware bug must not be able to keep the arm powered |
| Debug | SWD header, UART log, 2 status LEDs, test points | Bring-up |
| v2 | CAN-FD transceiver (TCAN1462), termination jumper | For custom joint modules |

4-layer board, KiCad 8, fabbed and part-assembled at JLCPCB/PCBWay. Two spins budgeted.

### 3.3 Firmware (C, STM32 HAL/LL + FreeRTOS or bare-metal superloop)
- 1 kHz timer-driven control task; servo reads/writes via DMA so the CPU is never blocked on the bus.
- Host protocol: versioned binary frames (`SET_TARGETS`, `STATE`, `CONFIG`, `ESTOP`, `PING`), CRC16, sequence numbers so dropped frames are visible.
- Interpolation: cubic or minimum-jerk between policy setpoints so a 30 Hz policy doesn't produce a 30 Hz jerk.
- Safety: joint position and velocity limits, current limit per joint, self-collision box check on the host side, watchdog tiers as in the diagram.
- Tests: protocol and interpolation code built for host too and unit-tested in CI (Unity or GoogleTest); hardware-in-the-loop script that logs jitter and round-trip latency.

### 3.4 Host software (Python on the Jetson and on a dev PC)
- `armctl`: driver for the board protocol, calibration (joint offsets, leader↔follower mapping), forward/inverse kinematics (own implementation, checked against Pinocchio or ikpy).
- Integration with Hugging Face **LeRobot** as a custom robot class, so recording, training, and evaluation use its tooling instead of a home-grown pipeline. The novel work is the hardware, data and edge deployment, not re-implementing ACT.
- Eval harness: scripted reset positions, success marked by a key press and optionally checked by an AprilTag on the target object, results written to a CSV per run.

## 4. Bill of materials (v1)

Prices are typical retail in late 2026 terms, from memory, not quotes. I'd confirm each before ordering.

| Item | Qty | Est. unit | Est. total |
|---|---|---|---|
| Serial bus servo, 12 V ~30 kg·cm (STS3215-class), follower arm | 6 | $18 | $108 |
| Same servo, 7.4 V version, leader arm (back-driven, low torque fine) | 6 | $14 | $84 |
| Jetson Orin Nano Super dev kit (8 GB) | 1 | $249 | $249 |
| NVMe SSD 256 GB for the Jetson | 1 | $30 | $30 |
| USB cameras (1 wide scene cam, 1 small wrist cam, 640×480 @ 30 fps, global shutter preferred for wrist) | 2 | $30 | $60 |
| Controller PCB, 2 spins × 5 boards, part-assembled | 2 | $60 | $120 |
| Off-board parts for the PCB (connectors, MCU if not stocked by assembler) | – | – | $40 |
| 12 V 10 A PSU + e-stop mushroom button + wiring | – | – | $45 |
| Filament PLA+/PETG/TPU (~2 kg) | – | – | $50 |
| Bearings, fasteners, heat-set inserts, springs, AprilTags, task objects | – | – | $55 |
| **v1 hardware total** | | | **≈ $840** |

If you already own a Jetson, a 3D printer, or a USB-C bench supply, subtract those. Without the Jetson the arm itself is ≈ $590.

**Compute for training:** ACT and Diffusion Policy train in a few hours on one 24 GB GPU. SmolVLA fine-tuning is also a single-GPU job (hours on an A100/H100-class card). Budget **$60–150** of rented GPU time for the whole project, or $0 if you have a desktop RTX 3090/4090.

**v2 stretch (custom FOC joints, 2 joints):** gimbal BLDC motors ×2 (~$35 each), AS5047P encoder boards ×2 (~$15), FOC driver board design + fab (~$90), bearings for cycloidal reducers (~$30). ≈ **$220** more.

**Project total:** ≈ $900–1,000 for v1 with training compute, ≈ $1,200 with v2.

## 5. Data collection plan
- **Teleop:** leader arm with the same kinematics, servos in torque-off mode read at 200 Hz; follower mirrors it. Leader is the cheapest and highest-quality teleop for this class; a phone/VR teleop is not worth the latency.
- **Tasks (5), easiest first:** (1) pick a block and place in a bowl, (2) stack two blocks, (3) push a cup to a marked target, (4) open a drawer of a small printed box, (5) sort two colours into two bowls (needs language to pick which).
- **Volume:** 50 demos per task for ACT baselines, then 80–100 per task for the language model, ≈ 300–500 total. At ~30 s per demo plus reset, about 6–8 hours of recording spread across sessions.
- **Variation:** randomise object pose each demo, vary lighting between sessions, keep 10% of demos with a distractor object.
- **Language labels:** 3–5 paraphrases per task written up front ("put the red block in the bowl", "drop the red cube into the bowl"...); held-out paraphrases for testing.
- **Format:** LeRobot dataset (parquet + MP4), pushed to a Hugging Face dataset repo (private or public, your choice; decision D4).

## 6. ML pipeline

### 6.1 Baseline: ACT (Action Chunking Transformer)
CVAE-transformer predicting 50-step action chunks from 2 images + joint state. Small (~80 M parameters with a ResNet-18 backbone), trains fast and runs easily on the Jetson. This is the "does the whole system work" milestone.

### 6.2 Diffusion Policy
U-Net diffusion over action chunks, trained on the same data. Typically smoother and better on multimodal demos but slower to infer; use DDIM with ~10 denoising steps on the Jetson. Comparing it with ACT on identical data and identical eval protocol is one of the project's results.

### 6.3 Language conditioning without a VLA (control experiment)
ACT with a frozen small text encoder embedding added as a token. Cheap, and tells us how much the full VLA actually buys over a simple conditioned policy. This is the honest baseline most hobby VLA projects skip.

### 6.4 Vision-language-action model
- **Default: SmolVLA** (~450 M params, SmolVLM backbone + flow-matching action expert, built for LeRobot-format data and consumer GPUs). Fine-tune on our 5 tasks. It is small enough that an FP16/INT8 TensorRT build on an 8 GB Orin Nano is plausible, which is the deciding factor.
- **Considered and parked:** OpenVLA (7 B) and π0 (~3 B). Both are too large to meet the 10 Hz budget on an Orin Nano; they would need a remote GPU, which breaks the "all on the desk" goal. If SmolVLA can't hit the latency target, the fallback is decision D3.
- **Deployment:** export vision encoder and action expert separately to ONNX → TensorRT; keep the language prefix cached per instruction (it only changes when you type a new command), so per-step cost is image encoding + action expert.

### 6.5 Edge latency budget (target)
| Stage | Target |
|---|---|
| Capture + resize 2 frames | 8 ms |
| Vision encoder (TensorRT FP16) | 35 ms |
| Action expert / flow steps | 40 ms |
| Overhead, IPC, USB to board | 7 ms |
| **Per chunk** | **≤ 90 ms → ≥ 11 Hz chunk rate** |
With 50-step chunks and temporal ensembling the arm gets fresh setpoints at 30–50 Hz regardless.

### 6.6 Evaluation protocol
Fixed 20 start poses per task (printed mat with marked positions), success criteria written before testing, every policy evaluated on the same poses. Report success rate with a Wilson 95% interval, mean completion time, and failure categories (missed grasp, wrong object, collision, timeout). Language eval: in-distribution phrasing vs held-out paraphrases.

## 7. Milestones

Effort assumes ~10–12 hours/week of your time; I can do the software, firmware, CAD scripting and analysis in threads, but printing, soldering, wiring and teleop need your hands.

| # | Milestone | Done when | Est. weeks |
|---|---|---|---|
| M0 | Decisions + ordering | D1–D5 settled, parts ordered, PCB v1 sent to fab | 1 |
| M1 | Arm on a dev board | Printed arm moves under a Nucleo-G474 + breadboard bus buffer; FK/IK verified against measured poses | 2–3 |
| M2 | Own PCB bring-up | PCB v1 runs the 1 kHz loop; jitter, latency and e-stop measured; rev list for v2 | 2 |
| M3 | Teleop + recording | Leader-follower at ≥100 Hz, 2 cameras synced, LeRobot dataset recording works end to end | 1–2 |
| M4 | ACT works | ≥50 demos on task 1, ACT ≥70% success on the Jetson | 2 |
| M5 | Full dataset + DP | 5 tasks recorded; ACT vs Diffusion Policy comparison table | 3 |
| M6 | Language | Text-conditioned ACT baseline, SmolVLA fine-tune, both evaluated incl. held-out phrasing | 3 |
| M7 | Edge optimisation | TensorRT build meets ≥10 Hz; latency budget measured stage by stage | 2 |
| M8 | Write-up | Repo README, demo video, dataset card, results report | 1 |
| v2 | Custom FOC joints | Two joints swapped; same tasks re-evaluated | 6–8 |

**v1 total ≈ 17–20 weeks** at that pace. M1 and M2 overlap with PCB fab lead time.

## 8. Risks

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Servo overheating / stall during long sessions | High | Medium | Shoulder counterbalance, current limits in firmware, session duty cycle, thermal logging |
| PCB v1 has a fatal error | Medium | Medium | Budget for 2 spins; M1 proves firmware on a Nucleo first so the board is not on the critical path |
| Servo backlash limits precision | High | Low–Med | Policies are closed-loop on vision, so ±2 mm is fine for these tasks; v2 FOC joints address it |
| SmolVLA too slow on Orin Nano | Medium | High | INT8 + fewer flow steps; reduce image size; fallback D3 (remote GPU or text-conditioned ACT) |
| Too little data for VLA to generalise | Medium | Medium | Start from the pretrained community checkpoint; add demos where failures cluster |
| Teleop demos inconsistent | Medium | Medium | Short practice sessions first; drop bad episodes with a review tool; one operator |
| Camera calibration drift (wrist cam mount flexes) | Medium | Low | Rigid printed mount, re-calibrate per session, augmentations in training |
| Safety: pinches, tipped arm | Low | High | Hardware e-stop, low-torque servos, clamp base to desk, velocity limits, no hands in workspace during autonomous runs |
| Scope creep into v2 before v1 works | Medium | High | v2 gated on M7 done |
| Jetson software stack friction (JetPack, TensorRT versions) | High | Low–Med | Pin JetPack and container versions in M0; use NVIDIA's L4T PyTorch containers |

## 9. Deliverables
1. Repository (`vla-arm`, new; you'd need to create it on GitHub): `hardware/` (KiCad + CAD), `firmware/`, `armctl/`, `policies/`, `eval/`, `docs/`.
2. PCB design files, BOM, bring-up report with scope captures.
3. Hugging Face dataset (≥300 episodes, 5 tasks, language labels).
4. Trained checkpoints: ACT, Diffusion Policy, text-ACT, SmolVLA fine-tune.
5. Results report: per-task success tables, ACT vs DP vs text-ACT vs VLA, latency budget measured on the Jetson.
6. 2–3 minute demo video of typed-instruction tasks.

## 10. Decisions needed from you before M0

- **D1 — Joint count:** 5 joints + gripper (recommended: lighter, easier leader arm, standard in this class) or a true 6 axes + gripper.
- **D2 — Hardware depth for v1:** bus servos + own controller board (recommended), or go straight to custom FOC joints for every joint (much harder, +2–3 months, +$400).
- **D3 — If the VLA can't run at 10 Hz on the Jetson:** fall back to a desktop/remote GPU for the VLA while ACT stays on-edge, or keep everything on-edge and accept text-conditioned ACT as the language model.
- **D4 — Dataset and repo visibility:** public (better portfolio value) or private.
- **D5 — What you already own:** 3D printer? Jetson? Bench supply / soldering setup / oscilloscope? GPU for training? This changes the BOM and whether I plan any training on rented GPUs.

## 11. How I'd split the work once you say proceed
- **I can do:** CAD (scripted in CadQuery so it lives in git), KiCad schematic and layout, firmware, host driver, LeRobot integration, training scripts and configs, TensorRT export, eval tooling, analysis and write-up.
- **Needs you:** ordering, printing, PCB assembly touch-up and wiring, flashing, running teleop sessions, and running eval trials (or supervising them), plus running anything that needs the Jetson, via your device.
