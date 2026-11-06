# firmware/ — arm controller board firmware (STM32G474)

Firmware for the arm controller board: 5 arm joints + gripper on a 1 Mbps half-duplex
STS3215-class servo bus, host link per `spec/protocol.md` (v1). It runs a 1 kHz control loop:
host frames in, quintic trajectories, servo SYNC_WRITE out, round-robin feedback READ, safety
state machine, STATE frames out.

No hardware exists yet. All of the logic lives in a portable C11 core with no HAL calls, no
`malloc` and no libm. It is built and tested on Linux against simulated servos. It also
cross-compiles for the Cortex-M4F target with `-Werror`.

## Layout

```
firmware/
  core/include/armfw/   public headers of the portable core
    common.h              fixed-size helpers, LE access, N = 6
    crc16.h  proto.h      CRC-16/CCITT-FALSE; frames, streaming parser, pack/unpack of every message
    config.h              per-joint calibration + limits (defaults = spec/kinematics.yaml), CONFIG keys
    traj.h                quintic trajectories, replanning, vmax stretching, blend fallback
    safety.h              IDLE/ACTIVE/HOLD/FAULT, fault flags, watchdog tiers, fault checks
    servo_bus.h           Feetech STS/SCS packets, status parser, feedback decode, read scheduler
    control.h             the 1 kHz step tying everything together
    hal_port.h            the HAL interface a target implements (9 functions)
  core/src/             implementation (≈ 1.4 k lines)
  sim/                  fake servo model + fake HAL + virtual-time simulator (sim_t)
  lib/armfw_api.[ch]    libarmfw.so: stable C API (framing, CRC, payloads, in-process sim)
  tools/fw_sim.c        CLI: firmware + sim, host frames on stdin, board frames on stdout
  tools/m4bench/        instruction-count benchmark of control_step on a Cortex-M4 model (QEMU)
  targets/stm32g474/    target layer: startup, clocks, linker script, register-level HAL, main
  cmake/arm-none-eabi.cmake  cross toolchain file
  tests/                tt.h (vendored minimal test framework), unit + closed-loop + timing tests
```

## Build and test

```sh
# host build + all tests (each TT_TEST is its own ctest case)
cmake -S firmware -B firmware/build && cmake --build firmware/build -j
(cd firmware/build && ctest -j)

# same under AddressSanitizer + UndefinedBehaviorSanitizer
cmake -S firmware -B firmware/build-asan -DARMFW_SANITIZE=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build firmware/build-asan -j && (cd firmware/build-asan && ctest -j)

# cross build for the STM32G474 (needs arm-none-eabi-gcc; apt install gcc-arm-none-eabi)
cmake -S firmware/targets/stm32g474 -B firmware/build-arm \
      -DCMAKE_TOOLCHAIN_FILE=$PWD/firmware/cmake/arm-none-eabi.cmake
cmake --build firmware/build-arm      # prints flash/RAM usage; .elf, .bin, .map in build-arm/

# Cortex-M4 instruction counts per control step (needs qemu-system-arm)
firmware/tools/m4bench/run.sh
```

Results at the time of writing: **96/96 ctest cases pass**, the same 96/96 under ASan+UBSan, and
the cross build is clean with `-Wall -Wextra -Werror` (core also `-pedantic -Wconversion`).

| Image (MinSizeRel, arm-none-eabi-gcc 13.2) | |
|---|---|
| Flash | 11 936 B of 512 KB (text 11 352, vectors 472, rodata 104) |
| RAM | 4 652 B static (`control_t` 1 136 B, host/servo DMA buffers 3.3 KB) + 4 KB reserved stack |
| Stack | `control_step` frame 536 B; deepest path (frame → replan → extrema) about 1 KB (`-fstack-usage`) |

### fw_sim

```sh
firmware/build/fw_sim [--fast] [--steps N] [--linger MS] [--echo] [--estop-open-at MS] [--log]
# serial-port style access for a driver that opens a tty:
socat PTY,link=/tmp/armfw,raw,echo=0 EXEC:"firmware/build/fw_sim"
```

By default it runs in real time at 1 kHz, reads host frames from stdin, writes raw board frames to
stdout and exits on stdin EOF. `tests/fw_sim_pipe_test.py` drives it end to end. That test also
cross-checks libarmfw.so against an independent Python CRC and frame encoder on 500 random frames.

## libarmfw.so — exported C API (ARMFW_API_VERSION 1, names frozen)

The header is `lib/armfw_api.h` and the library is `build/libarmfw.so` (SONAME `libarmfw.so.1`).
Only `armfw_*` symbols are exported (`-fvisibility=hidden`). The library never allocates:
objects live in caller memory of `armfw_*_sizeof()` bytes, aligned to 16. Sizes return 0 on
error; `int` results are 1 = ok, 0 = failure.

| Function | Purpose |
|---|---|
| `uint32_t armfw_api_version(void)` / `uint16_t armfw_fw_version(void)` | 1 / 0x0100 |
| `uint16_t armfw_crc16(const uint8_t*, size_t)` | CRC-16/CCITT-FALSE |
| `size_t armfw_encode_frame(type, seq, payload, len, out, cap)` | full frame, returns len+8 |
| `int armfw_expected_len(uint8_t type)` | payload length per TYPE, -1 unknown |
| `size_t armfw_parser_sizeof(void)`, `void armfw_parser_init(void*)` | streaming parser in caller memory |
| `size_t armfw_parser_feed(p, data, len, armfw_frame_t *out, max_frames, size_t *consumed)` | feed bytes, get frames (`armfw_frame_t` = ver,type,seq,len,payload[240]; 244 B) |
| `void armfw_parser_stats(p, uint32_t out[6])` | frames_ok, crc_errors, ver_errors, len_errors, bytes_skipped, overflows |
| `armfw_pack_*` / `armfw_unpack_*` for `set_targets`, `config`, `clear_fault`, `set_stream`, `pong`, `ack`, `nack`, `state` | payload layout of every message; scalar arguments |
| `size_t armfw_state_sizeof(void)` | 68; `armfw_state_t` offsets are documented in the header |
| `armfw_sim_sizeof/init/step/time_us/host_write/host_read` | the real firmware in-process, virtual time |
| `armfw_sim_set_estop_closed/set_supply_mv/set_servo_silent/set_servo_temp/inject_current/set_echo` | fault injection |
| `armfw_sim_mode/faults/servo_pos_mrad` | introspection |

## What is verified on the host, and what needs hardware

**Verified on the host** (unit, property and closed-loop tests, also under sanitizers):
- CRC check value 0x29B1, plus agreement with a bitwise reference and with a pure-Python reference.
- Round trip of every message type.
- Parser recovery from random garbage, split frames, bad CRC, bad VER, LEN > 240, frames hidden
  inside a corrupt frame, and random bit flips (never outputs a wrong frame).
- Servo packets against published example packets: AX-12 manual READ, broadcast WRITE,
  SYNC_WRITE and status packets, plus the Feetech STS goal-position WRITE.
- Quintic boundary conditions, and C2 continuity at replans and at the blend junction.
- vmax and joint limits checked at every 1 kHz step by randomized property tests (about 7 700
  replans, 0 vmax-infeasible, worst peak/vmax 1.0000).
- Every safety transition and fault rule, including the watchdog (HOLD after 101 ms of silence,
  FAULT/WATCHDOG after 501 ms) and the two-step clear.
- A 5 s closed loop with a 30 Hz policy streaming targets: joints converge to within 2 mrad, and
  exactly 1000 STATE frames arrive at 5-step spacing (200 Hz). `cmd_pos_crc` is checked.
- Half-duplex echo tolerance, jitter and overrun reporting, and servo bus bytes per cycle (34 B,
  or 54 B with the 100 ms torque refresh; ≤ 800 µs at 1 Mbps including a 50 µs turnaround).

**Timing.** Host cost of `control_step`: mean 0.8 µs, replan steps p50 5 µs. The host number is
only a regression guard. The QEMU Cortex-M4 model (`tools/m4bench`) counts instructions per
step:

| Step kind | mean insn | max insn |
|---|---|---|
| idle (torque off) | 4.4 k | 9.2 k |
| active, no new frame | 8.1 k | 13 k |
| active + SET_TARGETS replan, 30 Hz policy | 33 k | 80 k |
| adversarial full-range replans mid-motion | 42 k | 87 k |

At 170 MHz, assuming cycles ≈ 1.0–1.5 × instructions, the worst replan step is about 0.5–0.8 ms,
inside the 1 ms period but without much margin. A normal step is about 50–70 µs. The cycle count
must be measured on the board (`g_step_cycles_max` in `main.c` via the DWT counter). If it is
too tight, two mitigations are prepared but not done: run the core from CCM SRAM, or move
`traj_replan` out of the tick ISR into the background loop.

**Needs real hardware:**
- The register-level HAL in `targets/stm32g474/`. It compiles but has never run. The register
  addresses, DMAMUX request IDs and IRQ numbers in `stm32g4_min.h` were written from memory of
  RM0440 because ST's CMSIS headers were not downloadable here; verify them before flashing.
- Clock bring-up to 170 MHz.
- Servo bus turnaround, plus the STS3215 Return Delay register (must be set near 0).
- The real servo sign conventions for speed, load and current.
- The ADC (INA240 + divider), which is a TODO stub returning 12.0 V / 0 mA.
- USB CDC, which is a stub with a TinyUSB integration plan.
- The e-stop EXTI path.
- Real loop jitter and cycle counts.

## Spec interpretations

These are aligned with `armctl/README.md` § "Spec interpretations" (numbers in brackets refer to
that list). The two places that differ are marked **DIFFERS**.

1. [1] A CRC-valid frame with a known host TYPE but the wrong LEN gets NACK 1. Bad CRC, bad VER
   and LEN > 240 are dropped silently.
2. [2] A board→host TYPE (or any unknown TYPE) sent to the board gets NACK 3.
3. [3] Every CRC-valid frame, including NACKed ones, feeds the watchdog and sets `last_host_seq`.
   Corrupt frames do not (tested).
4. [4] The watchdog is armed only while torque is on (ACTIVE/HOLD). Thresholds are strict (`>`)
   and measured from the last frame.
5. [5] HOLD freezes the current commanded position immediately (v = a = 0). Any valid frame
   returns to ACTIVE, still holding until the next SET_TARGETS. A gentler stop would be a later
   option.
6. [6] After CLEAR_FAULT removes every latched bit, the mode stays FAULT with `fault_flags == 0`.
   A torque-enabled SET_TARGETS then goes to ACTIVE; a torque-off SET_TARGETS goes to IDLE.
7. [7] NACK 4 applies only to torque-enabled SET_TARGETS while latched bits remain. One addition:
   a torque-enable from limp before every servo has been read once (first ~6 ms after boot) also
   gets NACK 4.
8. [8, 18] CLEAR_FAULT is always ACKed. A bit whose condition still persists stays set: e-stop
   loop open, temperature > 70 °C, current over the limit, servo still silent. Bits 8–15 are
   ignored.
9. [9] BUS_UNDERVOLT is a live, informational flag with no effect on the mode, and CLEAR_FAULT
   cannot clear it. Firmware choices: it sets after 5 consecutive ms below 10.5 V and clears
   after 50 ms at or above 10.7 V. Note that a hardware e-stop also cuts servo power, but that
   case is covered by the latched ESTOP_HW.
10. [10] On torque enable from limp, the commanded state starts at the measured position with
    v = a = 0. While limp, the command follows the measured position every cycle.
11. [11] vmax stretching uses one common duration for all joints. The duration search is bounded
    to at most 21 evaluations; the tests saw at most 15. Where the spec cannot be met (see the
    issues below), the board uses the blend fallback, which is stricter than the armctl mock's
    "lowest peak" choice.
12. [12] LIMIT_CLAMP is set when a target was clamped, the duration was stretched, or the
    commanded path had to be saturated at a limit. The next torque-enabled SET_TARGETS that
    needs none of these clears it.
13. [13] Calibration: `mrad = direction × (ticks − 2048 − offset_ticks) × 2π·1000/4096`, and the
    inverse for goals. Goals are clamped to ticks 0..4095.
14. [14] `joint_status` bits (firmware-defined): bit0 torque on, bit1 feedback OK, bit2 target
    clamped, bit3 over current, bit4 over temp, bit5 comms fault, bit6 moving, bit7 servo error
    byte ≠ 0. The host ignores them.
15. [15] `cmd_pos_crc` is CRC-16/CCITT-FALSE over this cycle's six commanded setpoints, as int16
    LE mrad, rounded to nearest. While limp, those setpoints are the measured positions.
16. [16] CONFIG: global keys need joint 0xFF and per-joint keys need 0–5, else NACK 2. NACK 5 is
    checked first. Ranges: direction ±1; |offset| ≤ 4095; limits within ±32767 and min < max;
    vmax and current limit 1..32767; `1 ≤ hold_ms < off_ms ≤ 60000`. Extra rule (no armctl
    counterpart): offset_ticks and direction are refused with NACK 2 while torque is on, because
    the arm would jump. The spec has no "busy" code.
17. [17] SET_STREAM 1..999 gets NACK 2; 0 turns streaming off. The stream phase restarts on
    every SET_STREAM, and STATE timing follows the control-step count. **DIFFERS:** a SET_TARGETS
    with torque off and a bad `move_ms` or reserved flag bits is NACKed with code 2 (same reply
    as armctl), but the board **still turns torque off**. A torque-off request is never refused.
18. [19] PONG `uptime_ms` is milliseconds since boot (step count). `fw_version` is
    `major << 8 | minor` = 0x0100.
19. SERVO_COMMS "didn't answer 3 cycles in a row" is read as 3 consecutive polls of that servo.
    Each servo is polled every 6 ms, so the fault latches about 13–18 ms after a servo goes
    silent. 3 control cycles would be meaningless with a 6-cycle round robin.
20. OVERCURRENT thresholds are firmware choices, since the spec gives none. The per-joint limit
    defaults to 2000 mA and trips after 8 consecutive over-limit reads (~48 ms). The bus shunt
    limit is 8 A sustained for 50 ms. The gripper squeezing an object draws holding current, so
    the host should set its `current_limit_ma` deliberately.
21. `loop_jitter_max_us` is the maximum |period − 1000 µs| since the previous STATE frame (reset
    after each STATE). `loop_overruns` is cumulative, wraps at 2^16, and counts ticks ≥ 2 ms
    apart plus steps whose execution took more than 1 ms.
22. STATE `pos_mrad` and `vel_mrad_s` are the servos' measured values (latest round-robin read).
    Before a joint's first read, `pos_mrad` falls back to the commanded value.
23. Servo IDs are fixed at joint i → bus ID i+1. Each cycle sends the goal SYNC_WRITE (2-byte
    position) plus one READ of 15 bytes at 0x38 (position..current). The torque state is re-sent
    every 100 ms and on every change: torque-off before the goals, torque-on after them.

## Possible issues in spec/protocol.md (not edited)

- **The single-quintic replan cannot always respect vmax.** Replanning from a state with large
  acceleration a0: the a0 term of the quintic grows linearly with the duration T, so no
  stretched T keeps peak |v| ≤ vmax. Before the fallback, this happened in about 17 % of
  aggressive random replans, with worst overshoot 3.3× vmax. The firmware's fallback:
  - only when no single quintic is feasible, prepend a per-joint constant-jerk blend that ramps
    acceleration to 0 within ≤ 20 ms;
  - then plan the stretched quintic, still C2-continuous, to a common end time.
  
  This deviates from "the quintic move" in exactly those cases. Suggest the spec allow it.
- **Position limits can be overshot by a quintic that starts with velocity toward a nearby
  limit.** Example: moving fast toward the limit when the new target sits at or behind it. No
  duration avoids this. The firmware saturates the command at the limit, then continues
  rest-to-rest to the target, and sets LIMIT_CLAMP. This is the only remaining velocity
  discontinuity.
- **The spec contradicts itself on frames with the wrong LEN for their TYPE.** One sentence says
  they are dropped; the NACK code 1 "bad length" implies they are answered.
- **The spec has no "rejected while torque on" NACK code.** It would be the right reply for
  calibration CONFIG keys while torque is on.
- **PONG carries no PING sequence number,** so the host cannot match replies to requests.
- **`joint_status` bits are not defined.**
- **`t_us` wraps every 71.6 min.**
- **STATE at 200 Hz needs ≥ 230 400 baud on a plain UART;** bring-up uses 1 Mbaud.

## Known gaps

- The target HAL is a sketch that has never run. Register definitions are unverified (see
  above), the ADC is a stub, USB CDC is a stub, and there is no flash-stored calibration: CONFIG
  is RAM only and resets at boot.
- Real M4 cycle counts are estimated from QEMU instruction counts. The worst-case replan step
  (about 0.5–0.8 ms) needs measuring on silicon.
- The fake servo is first-order with a simple current model. It has no backlash, gravity
  dynamics, servo-side PID or reply latency variation.
- HOLD stops abruptly (spec-literal), with no deceleration ramp.
- There is no per-servo torque-limit or EEPROM configuration of the servos: return delay, angle
  limits and baud are assumed pre-configured.
- The core is not hardened against a host that changes joint limits mid-move beyond saturation.
