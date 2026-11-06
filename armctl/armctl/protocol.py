"""Wire protocol v1 between the host and the arm controller board.

Authoritative reference: ``spec/protocol.md``. Everything here is little-endian.

Frame layout::

    A5 5A | VER | TYPE | SEQ | LEN | PAYLOAD[LEN] | CRC16 (LE, over VER..PAYLOAD)
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
from typing import ClassVar, Iterator

SYNC0 = 0xA5
SYNC1 = 0x5A
PROTO_VER = 0x01
MAX_PAYLOAD = 240
HEADER_SIZE = 6  # SYNC0 SYNC1 VER TYPE SEQ LEN
CRC_SIZE = 2
N_JOINTS = 6
JOINT_NAMES = (
    "base_yaw",
    "shoulder_pitch",
    "elbow_pitch",
    "wrist_pitch",
    "wrist_roll",
    "gripper",
)
GLOBAL_JOINT = 0xFF


# --------------------------------------------------------------------------- CRC
def _make_crc_table() -> tuple[int, ...]:
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
        table.append(crc)
    return tuple(table)


_CRC_TABLE = _make_crc_table()


def crc16_ccitt_false(data: bytes | bytearray | memoryview, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, xorout 0."""
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]
    return crc


# --------------------------------------------------------------------------- enums
class MsgType(IntEnum):
    PING = 0x01
    SET_TARGETS = 0x02
    CONFIG = 0x03
    ESTOP = 0x04
    CLEAR_FAULT = 0x05
    SET_STREAM = 0x06
    PONG = 0x81
    STATE = 0x82
    ACK = 0x83
    NACK = 0x8F


class Mode(IntEnum):
    IDLE = 0
    ACTIVE = 1
    HOLD = 2
    FAULT = 3


class Fault(IntFlag):
    NONE = 0
    ESTOP_SOFT = 1 << 0
    ESTOP_HW = 1 << 1
    WATCHDOG = 1 << 2
    OVERCURRENT = 1 << 3
    OVERTEMP = 1 << 4
    SERVO_COMMS = 1 << 5
    BUS_UNDERVOLT = 1 << 6
    LIMIT_CLAMP = 1 << 7


#: Bits that put the board in FAULT and must be cleared with CLEAR_FAULT.
#: ESTOP_HW is not called "latched" in the spec but needs a host clear, so it is
#: treated as latched here (see README "Spec interpretations").
LATCHED_FAULTS = (
    Fault.ESTOP_SOFT
    | Fault.ESTOP_HW
    | Fault.WATCHDOG
    | Fault.OVERCURRENT
    | Fault.OVERTEMP
    | Fault.SERVO_COMMS
)


class NackError(IntEnum):
    BAD_LENGTH = 1
    BAD_VALUE = 2
    UNKNOWN_TYPE = 3
    REJECTED_FAULTED = 4
    UNKNOWN_CONFIG_KEY = 5


class ConfigKey(IntEnum):
    OFFSET_TICKS = 0x01
    DIRECTION = 0x02
    LIMIT_MIN_MRAD = 0x03
    LIMIT_MAX_MRAD = 0x04
    VMAX_MRAD_S = 0x05
    CURRENT_LIMIT_MA = 0x06
    WATCHDOG_HOLD_MS = 0x10  # global
    WATCHDOG_OFF_MS = 0x11  # global


GLOBAL_CONFIG_KEYS = frozenset({ConfigKey.WATCHDOG_HOLD_MS, ConfigKey.WATCHDOG_OFF_MS})

FLAG_TORQUE_ENABLE = 0x01


# --------------------------------------------------------------------------- unit helpers
def rad_to_mrad(x: float) -> int:
    """Round radians to int16 mrad, saturating at the int16 range."""
    v = int(round(float(x) * 1000.0))
    return max(-32768, min(32767, v))


def mrad_to_rad(v: int) -> float:
    return v / 1000.0


def commanded_pos_crc(pos_mrad: "list[int] | tuple[int, ...]") -> int:
    """CRC-16/CCITT-FALSE of the 12 bytes of commanded positions (6 x i16 LE)."""
    return crc16_ccitt_false(struct.pack("<6h", *pos_mrad))


# --------------------------------------------------------------------------- messages
class ProtocolError(ValueError):
    pass


@dataclass
class Message:
    """Base class. Subclasses define TYPE, SIZE and a struct ``_FMT``."""

    TYPE: ClassVar[MsgType]
    SIZE: ClassVar[int]

    def pack(self) -> bytes:  # pragma: no cover - overridden
        raise NotImplementedError

    @classmethod
    def unpack(cls, payload: bytes) -> "Message":  # pragma: no cover - overridden
        raise NotImplementedError

    @classmethod
    def _check_len(cls, payload: bytes) -> None:
        if len(payload) != cls.SIZE:
            raise ProtocolError(
                f"{cls.__name__}: payload is {len(payload)} bytes, expected {cls.SIZE}"
            )


@dataclass
class Ping(Message):
    TYPE: ClassVar[MsgType] = MsgType.PING
    SIZE: ClassVar[int] = 0

    def pack(self) -> bytes:
        return b""

    @classmethod
    def unpack(cls, payload: bytes) -> "Ping":
        cls._check_len(payload)
        return cls()


@dataclass
class SetTargets(Message):
    TYPE: ClassVar[MsgType] = MsgType.SET_TARGETS
    SIZE: ClassVar[int] = 15
    _FMT: ClassVar[str] = "<H6hB"

    move_ms: int
    pos_mrad: tuple[int, ...]
    flags: int = FLAG_TORQUE_ENABLE

    @property
    def torque_enable(self) -> bool:
        return bool(self.flags & FLAG_TORQUE_ENABLE)

    def pack(self) -> bytes:
        if len(self.pos_mrad) != N_JOINTS:
            raise ProtocolError("SET_TARGETS needs 6 positions")
        return struct.pack(self._FMT, self.move_ms, *self.pos_mrad, self.flags)

    @classmethod
    def unpack(cls, payload: bytes) -> "SetTargets":
        cls._check_len(payload)
        f = struct.unpack(cls._FMT, payload)
        return cls(move_ms=f[0], pos_mrad=tuple(f[1:7]), flags=f[7])


@dataclass
class Config(Message):
    TYPE: ClassVar[MsgType] = MsgType.CONFIG
    SIZE: ClassVar[int] = 6
    _FMT: ClassVar[str] = "<BBi"

    joint: int
    key: int
    value: int

    def pack(self) -> bytes:
        return struct.pack(self._FMT, self.joint, int(self.key), self.value)

    @classmethod
    def unpack(cls, payload: bytes) -> "Config":
        cls._check_len(payload)
        j, k, v = struct.unpack(cls._FMT, payload)
        return cls(joint=j, key=k, value=v)


@dataclass
class EStop(Message):
    TYPE: ClassVar[MsgType] = MsgType.ESTOP
    SIZE: ClassVar[int] = 0

    def pack(self) -> bytes:
        return b""

    @classmethod
    def unpack(cls, payload: bytes) -> "EStop":
        cls._check_len(payload)
        return cls()


@dataclass
class ClearFault(Message):
    TYPE: ClassVar[MsgType] = MsgType.CLEAR_FAULT
    SIZE: ClassVar[int] = 2

    mask: int

    def pack(self) -> bytes:
        return struct.pack("<H", int(self.mask))

    @classmethod
    def unpack(cls, payload: bytes) -> "ClearFault":
        cls._check_len(payload)
        return cls(mask=struct.unpack("<H", payload)[0])


@dataclass
class SetStream(Message):
    TYPE: ClassVar[MsgType] = MsgType.SET_STREAM
    SIZE: ClassVar[int] = 2

    period_us: int

    def pack(self) -> bytes:
        return struct.pack("<H", self.period_us)

    @classmethod
    def unpack(cls, payload: bytes) -> "SetStream":
        cls._check_len(payload)
        return cls(period_us=struct.unpack("<H", payload)[0])


@dataclass
class Pong(Message):
    TYPE: ClassVar[MsgType] = MsgType.PONG
    SIZE: ClassVar[int] = 7
    _FMT: ClassVar[str] = "<HIB"

    fw_version: int
    uptime_ms: int
    proto_ver: int = PROTO_VER

    def pack(self) -> bytes:
        return struct.pack(self._FMT, self.fw_version, self.uptime_ms, self.proto_ver)

    @classmethod
    def unpack(cls, payload: bytes) -> "Pong":
        cls._check_len(payload)
        return cls(*struct.unpack(cls._FMT, payload))


@dataclass
class JointState:
    pos_mrad: int = 0
    vel_mrad_s: int = 0
    current_ma: int = 0
    temp_c: int = 0
    status: int = 0

    _FMT: ClassVar[str] = "<hhhBB"
    SIZE: ClassVar[int] = 8


@dataclass
class State(Message):
    TYPE: ClassVar[MsgType] = MsgType.STATE
    SIZE: ClassVar[int] = 66
    _HEAD: ClassVar[str] = "<IHBB"  # 8
    _TAIL: ClassVar[str] = "<HhHHH"  # 10

    t_us: int = 0
    fault_flags: int = 0
    mode: int = Mode.IDLE
    last_host_seq: int = 0
    joints: list[JointState] = field(default_factory=lambda: [JointState() for _ in range(N_JOINTS)])
    bus_mv: int = 0
    bus_ma: int = 0
    loop_jitter_max_us: int = 0
    loop_overruns: int = 0
    cmd_pos_crc: int = 0

    def pack(self) -> bytes:
        if len(self.joints) != N_JOINTS:
            raise ProtocolError("STATE needs 6 joints")
        out = bytearray(
            struct.pack(self._HEAD, self.t_us, self.fault_flags, int(self.mode), self.last_host_seq)
        )
        for j in self.joints:
            out += struct.pack(JointState._FMT, j.pos_mrad, j.vel_mrad_s, j.current_ma, j.temp_c, j.status)
        out += struct.pack(
            self._TAIL,
            self.bus_mv,
            self.bus_ma,
            self.loop_jitter_max_us,
            self.loop_overruns,
            self.cmd_pos_crc,
        )
        assert len(out) == self.SIZE
        return bytes(out)

    @classmethod
    def unpack(cls, payload: bytes) -> "State":
        cls._check_len(payload)
        t_us, ff, mode, lhs = struct.unpack_from(cls._HEAD, payload, 0)
        joints = [
            JointState(*struct.unpack_from(JointState._FMT, payload, 8 + 8 * i)) for i in range(N_JOINTS)
        ]
        tail = struct.unpack_from(cls._TAIL, payload, 56)
        return cls(t_us, ff, mode, lhs, joints, *tail)

    # convenience views (radians, SI)
    @property
    def positions_rad(self) -> list[float]:
        return [j.pos_mrad / 1000.0 for j in self.joints]

    @property
    def velocities_rad_s(self) -> list[float]:
        return [j.vel_mrad_s / 1000.0 for j in self.joints]

    @property
    def faults(self) -> Fault:
        return Fault(self.fault_flags)


@dataclass
class Ack(Message):
    TYPE: ClassVar[MsgType] = MsgType.ACK
    SIZE: ClassVar[int] = 2

    acked_seq: int
    acked_type: int

    def pack(self) -> bytes:
        return struct.pack("<BB", self.acked_seq, int(self.acked_type))

    @classmethod
    def unpack(cls, payload: bytes) -> "Ack":
        cls._check_len(payload)
        return cls(*struct.unpack("<BB", payload))


@dataclass
class Nack(Message):
    TYPE: ClassVar[MsgType] = MsgType.NACK
    SIZE: ClassVar[int] = 3

    seq: int
    type: int
    error: int

    def pack(self) -> bytes:
        return struct.pack("<BBB", self.seq, int(self.type), int(self.error))

    @classmethod
    def unpack(cls, payload: bytes) -> "Nack":
        cls._check_len(payload)
        return cls(*struct.unpack("<BBB", payload))


MESSAGE_CLASSES: dict[int, type[Message]] = {
    c.TYPE: c
    for c in (Ping, SetTargets, Config, EStop, ClearFault, SetStream, Pong, State, Ack, Nack)
}
HOST_TO_BOARD = frozenset(t for t in MESSAGE_CLASSES if t < 0x80)
BOARD_TO_HOST = frozenset(t for t in MESSAGE_CLASSES if t >= 0x80)


# --------------------------------------------------------------------------- framing
@dataclass(frozen=True)
class Frame:
    type: int
    seq: int
    payload: bytes
    ver: int = PROTO_VER

    def decode(self) -> Message:
        """Parse the payload into its message dataclass (raises ProtocolError)."""
        cls = MESSAGE_CLASSES.get(self.type)
        if cls is None:
            raise ProtocolError(f"unknown message type 0x{self.type:02X}")
        return cls.unpack(self.payload)


def encode_frame(msg_type: int, seq: int, payload: bytes = b"", ver: int = PROTO_VER) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f"payload too long ({len(payload)} > {MAX_PAYLOAD})")
    body = bytes((ver & 0xFF, int(msg_type) & 0xFF, seq & 0xFF, len(payload))) + bytes(payload)
    crc = crc16_ccitt_false(body)
    return bytes((SYNC0, SYNC1)) + body + struct.pack("<H", crc)


def encode(msg: Message, seq: int) -> bytes:
    return encode_frame(msg.TYPE, seq, msg.pack())


@dataclass
class DecoderStats:
    frames: int = 0
    crc_errors: int = 0
    bad_version: int = 0
    bad_length: int = 0
    bytes_skipped: int = 0


class FrameDecoder:
    """Incremental decoder. Feed arbitrary byte chunks; get whole, CRC-valid frames.

    Resync rule: scan for ``A5 5A``. A header with unknown VER or LEN > 240 is rejected
    and the scan resumes at the byte after SYNC0. A frame with a bad CRC is rejected the
    same way (the scan restarts one byte after its SYNC0, not after its claimed end), so
    a real frame hidden inside a corrupt one is still found.

    Payload length vs TYPE is *not* checked here (that is a message-layer concern, see
    :meth:`Frame.decode`), so the board can tell "well-framed but invalid" apart.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.stats = DecoderStats()

    def reset(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> list[Frame]:
        self._buf += data
        return list(self._drain())

    def _skip(self, n: int) -> None:
        del self._buf[:n]
        self.stats.bytes_skipped += n

    def _drain(self) -> Iterator[Frame]:
        buf = self._buf
        while True:
            # find sync
            idx = buf.find(b"\xa5\x5a")
            if idx < 0:
                # keep a trailing A5 that may be the first half of a sync pair
                keep = 1 if buf and buf[-1] == SYNC0 else 0
                if len(buf) - keep:
                    self._skip(len(buf) - keep)
                return
            if idx:
                self._skip(idx)
            if len(buf) < HEADER_SIZE:
                return
            ver, length = buf[2], buf[5]
            if ver != PROTO_VER:
                self.stats.bad_version += 1
                self._skip(1)
                continue
            if length > MAX_PAYLOAD:
                self.stats.bad_length += 1
                self._skip(1)
                continue
            total = HEADER_SIZE + length + CRC_SIZE
            if len(buf) < total:
                return
            crc_rx = buf[total - 2] | (buf[total - 1] << 8)
            if crc16_ccitt_false(bytes(buf[2 : HEADER_SIZE + length])) != crc_rx:
                self.stats.crc_errors += 1
                self._skip(1)
                continue
            frame = Frame(type=buf[3], seq=buf[4], payload=bytes(buf[HEADER_SIZE : HEADER_SIZE + length]), ver=ver)
            del buf[:total]
            self.stats.frames += 1
            yield frame
