# firmware/ — arm controller board firmware (STM32G474)

Firmware for the arm controller board: 5 arm joints + gripper on a 1 Mbps half-duplex
STS3215-class servo bus, host link per `spec/protocol.md` (v1). It runs a 1 kHz control loop:
host frames in, quintic trajectories, servo SYNC_WRITE out, round-robin feedback READ, safety
state machine, STATE frames out.

No hardware exists yet. All of the logic lives in a portable C11 core with no HAL calls, no
`malloc` and no libm. It is built and tested on Linux against simulated servos, a simulated ADC
and a fault-injecting fake flash. It also cross-compiles for the Cortex-M4F target with
`-Werror`, with the host link on USB CDC (TinyUSB) by default.

Wave 2 added: the board ADC path (rail voltage, INA240 current, NTC) feeding the safety rules,
calibration persisted in flash (A/B pages, power-loss safe), the USB CDC host link, and a
machine check of the target's register definitions against ST's CMSIS header.

## Layout

```
firmware/
  core/include/armfw/   public headers of the portable core
    common.h              fixed-size helpers, LE access, N = 6
    crc16.h  proto.h      CRC-16/CCITT-FALSE; frames, streaming parser, pack/unpack of every message
    config.h              per-joint calibration + limits (defaults = spec/kinematics.yaml), CONFIG keys
    analog.h              ADC scaling (dividers, INA240, VREFINT), IIR filters, NTC Beta model
    calstore.h            calibration flash store: A/B page log, CRC-32, incremental save
    host_link.h           lock-free SPSC byte rings between the control ISR and USB CDC
    traj.h                quintic trajectories, replanning, vmax stretching, blend fallback
    safety.h              IDLE/ACTIVE/HOLD/FAULT, fault flags, watchdog tiers, fault checks
    servo_bus.h           Feetech STS/SCS packets, status parser, feedback decode, read scheduler
    control.h             the 1 kHz step tying everything together
    hal_port.h            the HAL interface a target implements (8 functions + optional flash port)
  core/src/             implementation (≈ 2.0 k lines)
  sim/                  fake servo model, fake flash (fault injection), fake HAL + ADC, simulator
  lib/armfw_api.[ch]    libarmfw.so: stable C API (framing, CRC, payloads, in-process sim)
  tools/fw_sim.c        CLI: firmware + sim, host frames on stdin, board frames on stdout
  tools/m4bench/        instruction-count benchmark of control_step on a Cortex-M4 model (QEMU)
  tools/check_regs.py   checks targets/stm32g474/stm32g4_min.h against ST's CMSIS header
  targets/stm32g474/    target layer: startup, clocks, linker script, register-level HAL, main,
                        adc_stm32g4.c, flash_stm32g4.c, usb_cdc_glue.c + usb_descriptors.c
  third_party/          TinyUSB 0.21.0, ST cmsis_device_g4 v1.2.6, CMSIS-Core 5.9.0 (trimmed,
                        licences kept; fetch.sh re-creates them from the pinned tags)
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
# same with the host link on LPUART1 (NUCLEO ST-LINK VCP) instead of USB CDC
cmake -S firmware/targets/stm32g474 -B firmware/build-arm-uart -DARMFW_HOST_LINK_USB=OFF \
      -DCMAKE_TOOLCHAIN_FILE=$PWD/firmware/cmake/arm-none-eabi.cmake && cmake --build firmware/build-arm-uart

# Cortex-M4 instruction counts per control step (needs qemu-system-arm)
firmware/tools/m4bench/run.sh

# register definitions of the target layer vs. ST's header (also the ctest case tools.check_regs)
python3 firmware/tools/check_regs.py
```

Results at the time of writing: **136/136 ctest cases pass**, the same 136/136 under ASan+UBSan
(the host-link SPSC ring test was also run under TSan, clean), and both cross builds are clean
with `-Wall -Wextra -Werror` (core also `-pedantic -Wconversion`). The two wall-clock tests
(`cli.fw_sim_pipe`, `timing.control_step_cost`) are marked `RUN_SERIAL`: next to CPU-heavy tests
on a loaded 4-core machine the real-time pipe test missed its 100 ms windows now and then.

| Image (MinSizeRel, arm-none-eabi-gcc 13.2) | USB CDC (default) | LPUART1 |
|---|---|---|
| Flash | 25 992 B of 508 KB (text 25 060, vectors 472, rodata 392, data 60) | 16 096 B |
| RAM | 10.4 KB static + 4 KB reserved stack | 5.0 KB + 4 KB |

In the USB build the static RAM breaks down as: `control_t` 1 408 B, USB rings 3 KB, TinyUSB CDC
state 1.6 KB, and the LPUART buffers 3 KB (still allocated in the USB build).

Stack: the `control_step` frame is 544 B. The deepest path (frame → replan → extrema) is about
1 KB, and `calstore_init` uses 256 B (`-fstack-usage`).

The last 4 KB of flash (bank 2 pages 126/127) are reserved for the calibration store.

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
| `armfw_sim_set_estop_closed/set_supply_mv/set_servo_silent/set_servo_temp/inject_current/set_echo` | fault injection (supply voltage now reaches the firmware as ADC codes) |
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
- (wave 2) ADC scaling over 0–24 V and 0–20 A to within 1 LSB, VREFINT correction from 3.0 to
  3.4 V, the NTC Beta model against libm (±0.02 °C), filter noise and step response, the stale-ADC
  path, and each threshold driven by synthetic code sequences.
- (wave 2) The calibration store under power cuts at every byte and every erase position (see
  "Calibration in flash"); CONFIG 0x20/0x21 end to end, with values surviving `sim_reboot`.
- (wave 2) The USB host-link rings against a fake CDC and two real threads.

**Timing.** Host cost of `control_step`: mean 0.8 µs, replan steps p50 5 µs. The host number is
only a regression guard. The QEMU Cortex-M4 model (`tools/m4bench`) counts instructions per
step:

In wave 2, every step also scales the ADC readings and runs the NTC model.

| Step kind | mean insn | max insn |
|---|---|---|
| idle (torque off) | 4.8 k | 9.5 k |
| active, no new frame | 8.5 k | 13.2 k |
| active + SET_TARGETS replan, 30 Hz policy | 33.6 k | 80.1 k |
| adversarial full-range replans mid-motion | 42.4 k | 82.5 k |
| step servicing a calibration save (erase start / program / read-back verify) | 8.6 k | 11.2 k |
| step receiving CONFIG 0x20 (save start, blank-slot check) | | 12.0 k |
| **replan + save service in the same step (worst case seen)** | | **84.6 k** |
| boot `calstore_init` over two full pages (once, before the tick) | | 65.4 k |
| `host_link_write` (74 B frame, ISR side) / `host_link_pump` (≤ 64 B) | | 147 / 173 |

The sim's fake ADC and fake servos run inside the measured window, so these numbers include some
simulator cost (the real `adc_read` is a 6-word copy). A save adds at most ~2.5 k instructions to
a step, because the store starts or polls at most one flash operation per step.

At 170 MHz, assuming cycles ≈ 1.0–1.5 × instructions, the worst replan step is about 0.5–0.8 ms,
inside the 1 ms period but without much margin. A normal step is about 50–70 µs. The cycle count
must be measured on the board (`g_step_cycles_max` in `main.c` via the DWT counter). If it is
too tight, two mitigations are prepared but not done: run the core from CCM SRAM, or move
`traj_replan` out of the tick ISR into the background loop.

**Needs real hardware:**
- The register-level HAL in `targets/stm32g474/`. It compiles but has never run.
  `stm32g4_min.h` (written from memory in wave 1) is now checked by `tools/check_regs.py`
  against ST's `stm32g474xx.h`: 171 definitions match, 0 differ. The new ADC, flash and USB code
  uses ST's header directly. What is still from memory is listed under "Unverified register
  details" below.
- Clock bring-up to 170 MHz.
- Servo bus turnaround, plus the STS3215 Return Delay register (must be set near 0).
- The real servo sign conventions for speed, load and current.
- The ADC bring-up (calibration, sampling times, oversampling) and the analog accuracy: divider
  tolerances, INA240 offset (`isense_offset_mv` exists for a zero-current calibration), NTC
  placement.
- Flash programming/erase on silicon, the ECC-NMI recovery path, and real erase times.
- USB enumeration, CRS trimming and CDC throughput.
- The e-stop EXTI path.
- Real loop jitter and cycle counts.

## ADC path (wave 2)

`hal_port.adc_read()` returns raw 12-bit codes; `core/src/analog.c` does everything else.

| Signal | Pin / channel (mcu_pins.md) | Conversion |
|---|---|---|
| servo rail `bus_mv` | PA1 = ADC1_IN2 | divider 100k/15k: mV = code · VDDA/4095 · 115/15 (6.2 mV/LSB, 25.3 V full scale) |
| rail current `bus_ma` | PA0 = ADC1_IN1 | INA240A2 (×50) on 3 mΩ, REF = GND: 0.15 V/A, 5.4 mA/LSB, ~22 A full scale, ≥ 0 only |
| protected input `vin_mv` | PC1 = ADC1_IN7 | same divider (diagnostic, not in STATE) |
| board temperature | PC0 = ADC1_IN6 | 10k pull-up to VREF+, NTC 10k B3380 to GND: R = 10k·c/(4095−c), Beta equation, `ln` without libm (error < 2e-6) |
| VDDA | VREFINT (ADC1_IN18) | VDDA = 3.0 V · VREFINT_CAL / code (nominal 3.3 V when not supplied) |

- **Target:** continuous scan, 16× hardware oversampling, DMA circular; one sequence takes about
  0.34 ms.
- **Filters** (per 1 kHz step): rail V and I `y += (x−y)/2`, NTC `/64` (τ ≈ 64 ms), VREFINT
  `/16`. The first sample primes the filters.
- **Stale ADC:** with no conversion for 10 ms the readings are invalid and
  `bus_mv = bus_ma = 0`. That raises BUS_UNDERVOLT (a visible sensing failure) and can never trip
  the bus overcurrent on stale data.
- **Safety thresholds:** BUS_UNDERVOLT and the 8 A / 50 ms bus overcurrent use these values.
  Measured with synthetic code sequences:
  - a drop to 10.2 V sets the flag after 7–9 ms;
  - a 3 A → 8.5 A step latches OVERCURRENT after 53 ms;
  - 7.8 A forever, single-sample 20 A spikes and 40 ms bursts of 8.5 A do not trip.
- **Board overtemperature** (firmware choice; the spec only defines OVERTEMP > 70 °C for servos):
  - NTC > 85.0 °C for 1 s latches OVERTEMP;
  - CLEAR_FAULT is refused while the board is still ≥ 80.0 °C;
  - an open or shorted NTC (outside −40…150 °C) is reported invalid and never trips.

## Calibration in flash (wave 2)

`core/src/calstore.c`, driven by the control step; target driver `targets/stm32g474/flash_stm32g4.c`.

- **Layout.** Two 2 KB pages (A, B), each an append-only log of 21 fixed 96-byte records. A
  record holds: magic `ACAL`, format version 1, payload length 76, a 32-bit sequence number, the
  payload (all per-joint calibration and limits + watchdog times), and a CRC-32 last. A record
  counts only if magic, version, length and CRC match **and** every field passes the CONFIG range
  checks.
- **Boot.** Both pages are scanned and the valid record with the highest sequence wins; if there
  is none, the built-in defaults are used. Unreadable (ECC) or garbage slots are skipped.
- **Save.**
  - The new record goes into the next blank slot after the newest record, in the same page.
  - When that page is full, the other page is erased and written from slot 0.
  - The page that holds the newest valid record is never erased or overwritten, so a power cut
    at any moment leaves it readable.
  - Every record is read back and compared. A failing slot is skipped and the next one tried (3
    retries).
  - The save is a state machine that starts or polls at most one flash operation per control
    step: about 14 steps per save, plus about 22 ms on the silicon when a page erase is due.
- **Wear.** One page erase per 21 saves, alternating pages: 10 k cycles ≈ 420 k saves.
- **Tested against the fake flash** (`sim/fake_flash.c`, STM32G4 rules: program only erased
  double words, PROGERR otherwise):
  - a power cut at every byte offset of the save, in each of three torn-double-word behaviours:
    partial bits, an ECC error on read, or still looking blank;
  - five starting states for those cuts: first save ever, append, last slot of a page, first
    page rollover, second page rollover;
  - a torn erase at every double word of the page;
  - an ECC error or bit rot in the newest record;
  - PROGERR, silent corruption caught by the read-back, and a flash that never finishes;
  - 3 000 random saves with random cuts, several in a row.

  After every reboot the loaded configuration was the previous one or the new one. It was never
  garbage, and never the defaults once something had been saved. A completed save was never
  lost, and the next save always worked.

### Proposed protocol additions (not in spec/protocol.md)

Both are CONFIG messages with joint `0xFF` and value `0x5AFE`.

**Key `0x20`, save to flash.** The current configuration is snapshotted and written as a new
record.
- ACK is sent only after the record is written and verified: about 14 ms, or about 40 ms when a
  page erase is due.
- NACK 4: there is no flash, a save is already running, or the write failed.
- NACK 2: the value is not `0x5AFE`, or the joint is not `0xFF`.

**Key `0x21`, restore defaults.** The RAM configuration goes back to the built-in defaults. Flash
is not touched, so send `0x20` afterwards to persist the defaults.
- ACK on success.
- NACK 2 while torque is on (the same code as offset/direction), because new offsets would make
  the arm jump.

Saving is allowed with torque on. The flash work runs in the background on bank 2 while the code
runs from bank 1, and the closed-loop test sees no overrun. Replies to frames sent after the
`0x20` frame can arrive before its deferred ACK. An older host that never sends these keys is
unaffected. Without them, CONFIG stays RAM-only as before.

## USB CDC host link (wave 2)

TinyUSB 0.21.0 (MIT) was cloned and vendored (trimmed) in `third_party/`. `usb_cdc_glue.c` and
`usb_descriptors.c` integrate it in the target build, which is the default build.

- **Descriptors.** One CDC-ACM function. VID/PID 0x1209/0x0001 is the pid.codes **test** PID;
  replace it before boards are distributed. The serial number string comes from the 96-bit
  unique ID.
- **Where the code runs.** `tud_task()` and `host_link_pump()` run in the background loop, never
  in the 1 kHz ISR. The ISR only reads and writes two lock-free SPSC rings
  (`core/src/host_link.c`, RX 1 KB, TX 2 KB).
- **Ring behaviour.**
  - `host_write` takes whole frames or nothing.
  - RX is flow-controlled: bytes that do not fit stay in TinyUSB's FIFO and the host is NAKed.
  - TX bytes queued while no host has the port open (DTR low) are dropped, so a newly opened port
    does not start with stale frames.
- **Interrupts and clock.** USB IRQs (HP, LP, wakeup) have priority 3, below the control tick
  (1). HSI48 is the 48 MHz clock, trimmed by the CRS from USB SOF.
- **Host tests** (`tests/test_host_link.c`, fake CDC):
  - byte-exact transport of random frame streams under random packetisation and back-pressure;
  - a 40 kB RX burst through the 1 KB ring with no loss;
  - whole-frame refusal when the ring is full;
  - the disconnect discard;
  - 4 MB through the ring with a real producer thread and a real consumer thread (also clean
    under TSan).
- **Bring-up option.** `-DARMFW_HOST_LINK_USB=OFF` builds the LPUART1 link for the NUCLEO.

## Unverified register details

`tools/check_regs.py` checks `stm32g4_min.h`, and the new modules use ST's names. Neither covers
the items below, which were written from memory. Verify them in RM0440 / DS12288 before flashing:

- DMAMUX request IDs: USART1 RX/TX = 24/25, LPUART1 RX/TX = 34/35, ADC1 = 5. ST keeps these in
  the LL/HAL headers, not in CMSIS.
- VREFINT_CAL is at address 0x1FFF75AA and was measured at VDDA = 3.0 V.
- ADC clocking: `CKMODE = 11` (HCLK/4, synchronous) makes `RCC_CCIPR.ADC12SEL` irrelevant.
- ADC sampling: VREFINT needs ≥ 4 µs, so it uses 640.5 cycles at 42.5 MHz.
- Flash keys: 0x45670123 / 0xCDEF89AB.
- Flash banks: with DBANK = 1, bank 2 starts at 0x08040000 with 128 pages of 2 KB. DBANK = 1 is
  assumed to be the factory default; the driver checks `OPTR.DBANK` and disables persistence
  otherwise.
- Cache flush after an erase or program: DCEN/ICEN off, a DCRST/ICRST pulse, then the enables
  restored.
- **ECC NMI:** the driver assumes that returning from the ECCD NMI resumes the interrupted flash
  read with garbage data, and uses this to report a torn double word as unreadable. If the part
  re-faults instead, `flash_read` needs another strategy.
- USB: CRS `SYNCSRC` resets to USB SOF, and PA11/PA12 need no MODER/AF setting for the USB FS PHY.
- Pins: `hal_stm32g474.c` still follows the NUCLEO bring-up map for the servo bus, e-stop, LEDs
  and log, not `mcu_pins.md`. Only the ADC channels and USB follow the custom board.

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

24. (wave 2) BUS_UNDERVOLT is evaluated on the **servo rail** (after the e-stop/servo power
    switch), so it is also set whenever servo power is off: e-stop open, or a dead ADC. The
    protected input voltage is measured too (`analog.vin_mv`), but it is not in STATE.
25. (wave 2) `bus_ma` in STATE is ≥ 0, because the INA240 is wired unidirectional (REF = GND).
26. (wave 2) Board overtemperature reuses OVERTEMP (bit 4): > 85 °C for 1 s, as described in "ADC
    path". `joint_status` bit 4 stays per servo.
27. (wave 2) CONFIG keys 0x20/0x21 are firmware additions, see "Proposed protocol additions".

## Possible issues in spec/protocol.md (not edited)

- **v1.1 clarification 5 says calibration CONFIG keys while torque is on get NACK 4. The firmware
  answers NACK 2** (interpretation 16), and so does `tests/integration/test_armctl_vs_firmware.py`
  ("NACK 2 (firmware choice)"). The firmware and the integration test agree with each other but
  not with the spec text. `CFG_ERR_TORQUE_ON` in `config.h` switches every such reply, including
  key 0x21, in one place. The spec and the integration test should settle on one code.
- **The spec does not say whose temperature OVERTEMP covers.** It only gives "> 70 °C". The
  firmware applies 70 °C to the servos and 85 °C to the board NTC.
- **CONFIG has no persistence.** Keys 0x20/0x21 are proposed for v1.2 or v2. The spec has no way
  to report that a CONFIG value was restored from flash at boot; the firmware tracks it
  internally (`cal_from_flash`).

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

- The target HAL is a sketch that has never run. Register definitions are now checked against ST's
  header; the items in "Unverified register details" are not. The ADC, flash and USB drivers
  compile and link but have never run.
- The target still uses the NUCLEO pin map for the servo bus, e-stop and LEDs; the custom board
  differs (`board.h` note). SERVO_PWR_EN (PC6) is not driven.
- The INA240 zero-current offset is not calibrated automatically. `isense_offset_mv` defaults to
  0; at boot with servo power off, the reading could be captured as the offset.
- A torn double word that reads back as still erased (marginal cells) is handled in the fake
  flash, where the read-back verify catches it. Whether re-programming such a double word on the
  real part is benign is unknown.
- The LPUART buffers (3 KB) are still allocated in the USB build.
- libarmfw.so does not yet expose the new fault injection (board temperature, dead ADC, flash
  power cuts) or `analog`/`calstore` state. The C tests reach them through `sim_t`.
- Real M4 cycle counts are estimated from QEMU instruction counts. The worst-case replan step
  (about 0.5–0.8 ms) needs measuring on silicon.
- The fake servo is first-order with a simple current model. It has no backlash, gravity
  dynamics, servo-side PID or reply latency variation.
- HOLD stops abruptly (spec-literal), with no deceleration ramp.
- There is no per-servo torque-limit or EEPROM configuration of the servos: return delay, angle
  limits and baud are assumed pre-configured.
- The core is not hardened against a host that changes joint limits mid-move beyond saturation.
