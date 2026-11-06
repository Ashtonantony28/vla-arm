# Host ↔ arm controller board protocol, version 1

Transport: USB-CDC (virtual serial). The same framing works over a plain UART for bring-up.
All multi-byte integers are **little-endian**. Signed values are two's complement.

## Frame

| Field | Size | Notes |
|---|---|---|
| SYNC0 | 1 | `0xA5` |
| SYNC1 | 1 | `0x5A` |
| VER | 1 | `0x01` |
| TYPE | 1 | message type, below |
| SEQ | 1 | sender's sequence number, wraps 255→0 |
| LEN | 1 | payload length, 0–240 |
| PAYLOAD | LEN | |
| CRC | 2 | CRC-16/CCITT-FALSE over VER..PAYLOAD (not the sync bytes), little-endian |

CRC-16/CCITT-FALSE: poly `0x1021`, init `0xFFFF`, no input/output reflection, xorout `0x0000`.
Check value: CRC of ASCII `"123456789"` = `0x29B1`.

Receivers resynchronise by scanning for `A5 5A`. A frame with bad CRC, unknown VER, LEN > 240,
or a payload length that does not match its TYPE is dropped; the board answers a well-framed but
invalid host frame with `NACK`.

Joint order everywhere: `base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll, gripper` (N = 6).
Angle unit: **mrad** as int16 (±32.767 rad). Velocity: mrad/s int16. Current: mA int16.

## Host → board

| TYPE | Name | Payload |
|---|---|---|
| `0x01` | PING | none |
| `0x02` | SET_TARGETS | `u16 move_ms`, `i16 pos[6]` (mrad), `u8 flags` → 15 bytes |
| `0x03` | CONFIG | `u8 joint` (0–5, or `0xFF` = global), `u8 key`, `i32 value` → 6 bytes |
| `0x04` | ESTOP | none. Latches the soft e-stop fault: torque off. |
| `0x05` | CLEAR_FAULT | `u16 mask` of fault bits to clear |
| `0x06` | SET_STREAM | `u16 period_us` for STATE frames (0 = off; min 1000; default 5000 = 200 Hz) |

SET_TARGETS `flags`: bit0 = torque enable (0 → torque off, targets ignored). Other bits reserved, must be 0.
`move_ms`: duration of the quintic move from the board's current commanded state to the target
(1–5000). A new SET_TARGETS mid-move replans from the current commanded position, velocity and
acceleration (continuous to 2nd order), ending at the target with zero velocity and acceleration.
The board clamps targets to joint limits and the resulting motion to per-joint vmax (by stretching
the duration, not by clipping the trajectory) and sets `FAULT_LIMIT_CLAMP` (informational) when it does.

CONFIG keys (per joint unless noted):
`0x01` offset_ticks, `0x02` direction (+1/-1), `0x03` limit_min_mrad, `0x04` limit_max_mrad,
`0x05` vmax_mrad_s, `0x06` current_limit_ma, `0x10` (global) watchdog_hold_ms (default 100),
`0x11` (global) watchdog_off_ms (default 500).

## Board → host

| TYPE | Name | Payload |
|---|---|---|
| `0x81` | PONG | `u16 fw_version`, `u32 uptime_ms`, `u8 proto_ver` → 7 bytes |
| `0x82` | STATE | see below → 66 bytes |
| `0x83` | ACK | `u8 acked_seq`, `u8 acked_type` → 2 bytes |
| `0x8F` | NACK | `u8 seq`, `u8 type`, `u8 error` → 3 bytes |

STATE payload (66 bytes):
`u32 t_us`, `u16 fault_flags`, `u8 mode`, `u8 last_host_seq`,
then for each of 6 joints: `i16 pos_mrad`, `i16 vel_mrad_s`, `i16 current_ma`, `u8 temp_c`, `u8 joint_status` (6 × 8 = 48),
then `u16 bus_mv`, `i16 bus_ma`, `u16 loop_jitter_max_us`, `u16 loop_overruns`, `u16 cmd_pos_crc` (CRC-16 of the 12 bytes of current commanded positions, for debugging).
Size check: 4+2+1+1 + 48 + 2+2+2+2+2 = 66.

SET_TARGETS, CONFIG, ESTOP, CLEAR_FAULT, SET_STREAM are answered with ACK (or NACK). PING with PONG.

NACK error codes: `1` bad length, `2` bad value / out of range, `3` unknown type, `4` rejected while faulted, `5` unknown config key.

## Modes (`mode` byte)

| Value | Mode | Meaning |
|---|---|---|
| 0 | IDLE | torque off, no fault |
| 1 | ACTIVE | tracking targets |
| 2 | HOLD | host silent > watchdog_hold_ms: hold the current commanded position, torque on |
| 3 | FAULT | torque off until faults are cleared |

## Fault flags (`u16`)

bit0 ESTOP_SOFT (latched), bit1 ESTOP_HW (hardware loop open; clears when loop closes and host clears),
bit2 WATCHDOG (host silent > watchdog_off_ms, latched), bit3 OVERCURRENT (latched),
bit4 OVERTEMP (latched, >70 °C), bit5 SERVO_COMMS (servo didn't answer 3 cycles in a row, latched),
bit6 BUS_UNDERVOLT (<10.5 V), bit7 LIMIT_CLAMP (informational, self-clears on next in-range target).

Latched faults put the board in FAULT and torque off. Leaving FAULT needs CLEAR_FAULT for every
latched bit **and** then a SET_TARGETS with torque enable.

## Timing

- Control loop: 1 kHz. Servo bus: 1 Mbps half-duplex. Each cycle writes all 6 goal positions with
  one SYNC_WRITE and reads back one servo in round-robin (each joint refreshed at ~166 Hz), so the
  bus load per cycle stays under 1 ms.
- Watchdog: any valid host frame resets it.
