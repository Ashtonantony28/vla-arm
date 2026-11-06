import random
import struct

import pytest
from hypothesis import given, settings, strategies as st

from armctl import protocol as P


def crc_bitwise(data: bytes) -> int:
    """Independent reference: bit-by-bit CRC-16/CCITT-FALSE."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def test_crc_check_value():
    assert P.crc16_ccitt_false(b"123456789") == 0x29B1
    assert crc_bitwise(b"123456789") == 0x29B1


@given(st.binary(max_size=300))
def test_crc_matches_reference(data):
    assert P.crc16_ccitt_false(data) == crc_bitwise(data)


def test_message_sizes_match_spec():
    sizes = {
        P.MsgType.PING: 0, P.MsgType.SET_TARGETS: 15, P.MsgType.CONFIG: 6, P.MsgType.ESTOP: 0,
        P.MsgType.CLEAR_FAULT: 2, P.MsgType.SET_STREAM: 2, P.MsgType.PONG: 7, P.MsgType.STATE: 66,
        P.MsgType.ACK: 2, P.MsgType.NACK: 3,
    }
    assert set(P.MESSAGE_CLASSES) == set(sizes)
    for t, n in sizes.items():
        assert P.MESSAGE_CLASSES[t].SIZE == n


# ---------------------------------------------------------------- hand-built frames
def test_ping_frame_bytes():
    raw = P.encode(P.Ping(), seq=0)
    body = bytes([0x01, 0x01, 0x00, 0x00])
    crc = crc_bitwise(body)
    assert raw == bytes([0xA5, 0x5A]) + body + bytes([crc & 0xFF, crc >> 8])
    assert raw.hex() == "a55a0101000044c5"


def test_set_targets_frame_bytes():
    msg = P.SetTargets(move_ms=500, pos_mrad=(1000, -1000, 0, 32767, -32768, 1600), flags=1)
    raw = P.encode(msg, seq=7)
    payload = bytes.fromhex("f401" "e803" "18fc" "0000" "ff7f" "0080" "4006" "01")
    assert len(payload) == 15
    body = bytes([0x01, 0x02, 0x07, 15]) + payload
    crc = crc_bitwise(body)
    assert raw == b"\xa5\x5a" + body + struct.pack("<H", crc)


def test_config_and_clear_fault_bytes():
    assert P.Config(0xFF, P.ConfigKey.WATCHDOG_HOLD_MS, 100).pack() == bytes.fromhex("ff1064000000")
    assert P.Config(2, P.ConfigKey.OFFSET_TICKS, -5).pack() == bytes.fromhex("0201fbffffff")
    assert P.ClearFault(0x0105).pack() == bytes.fromhex("0501")
    assert P.SetStream(5000).pack() == bytes.fromhex("8813")
    assert P.Ack(9, 0x02).pack() == bytes.fromhex("0902")
    assert P.Nack(9, 0x02, 4).pack() == bytes.fromhex("090204")
    assert P.Pong(0x0102, 0x11223344, 1).pack() == bytes.fromhex("0201" "44332211" "01")


def test_state_layout_offsets():
    joints = [P.JointState(100 * i + 1, -i, 10 * i, 30 + i, i) for i in range(6)]
    s = P.State(
        t_us=0x01020304, fault_flags=0x0081, mode=P.Mode.HOLD, last_host_seq=0xAB, joints=joints,
        bus_mv=12000, bus_ma=-5, loop_jitter_max_us=7, loop_overruns=8, cmd_pos_crc=0xBEEF,
    )
    b = s.pack()
    assert len(b) == 66
    assert b[0:4] == bytes.fromhex("04030201")
    assert b[4:6] == bytes.fromhex("8100")
    assert b[6] == 2 and b[7] == 0xAB
    # joint 5 block starts at 8 + 5*8 = 48: pos, vel, cur, temp, status
    assert struct.unpack_from("<hhhBB", b, 48) == (501, -5, 50, 35, 5)
    assert struct.unpack_from("<HhHHH", b, 56) == (12000, -5, 7, 8, 0xBEEF)
    assert P.State.unpack(b) == s


# ---------------------------------------------------------------- round trips
i16 = st.integers(-32768, 32767)
u8 = st.integers(0, 255)
u16 = st.integers(0, 0xFFFF)
u32 = st.integers(0, 0xFFFFFFFF)

msg_strategy = st.one_of(
    st.just(P.Ping()),
    st.just(P.EStop()),
    st.builds(P.SetTargets, u16, st.tuples(i16, i16, i16, i16, i16, i16), u8),
    st.builds(P.Config, u8, u8, st.integers(-(2**31), 2**31 - 1)),
    st.builds(P.ClearFault, u16),
    st.builds(P.SetStream, u16),
    st.builds(P.Pong, u16, u32, u8),
    st.builds(P.Ack, u8, u8),
    st.builds(P.Nack, u8, u8, u8),
    st.builds(
        P.State, u32, u16, u8, u8,
        st.lists(st.builds(P.JointState, i16, i16, i16, u8, u8), min_size=6, max_size=6),
        u16, i16, u16, u16, u16,
    ),
)


@given(msg_strategy, u8)
def test_roundtrip_all_messages(msg, seq):
    payload = msg.pack()
    assert len(payload) == msg.SIZE
    frames = P.FrameDecoder().feed(P.encode(msg, seq))
    assert len(frames) == 1
    fr = frames[0]
    assert (fr.type, fr.seq, fr.ver) == (msg.TYPE, seq, 1)
    assert fr.decode() == msg


def test_wrong_length_payload_raises():
    with pytest.raises(P.ProtocolError):
        P.Frame(type=P.MsgType.ACK, seq=0, payload=b"\x01").decode()
    with pytest.raises(P.ProtocolError):
        P.Frame(type=0x55, seq=0, payload=b"").decode()


def test_encode_rejects_long_payload():
    with pytest.raises(P.ProtocolError):
        P.encode_frame(0x01, 0, b"\x00" * 241)


def test_rad_mrad_saturates():
    assert P.rad_to_mrad(1.2345) == 1234 or P.rad_to_mrad(1.2345) == 1235
    assert P.rad_to_mrad(100.0) == 32767
    assert P.rad_to_mrad(-100.0) == -32768


# ---------------------------------------------------------------- decoder robustness
def test_decoder_drops_bad_crc_bad_ver_bad_len():
    good = P.encode(P.Ack(1, 2), 3)
    bad_crc = bytearray(good)
    bad_crc[-1] ^= 0xFF
    bad_ver = P.encode_frame(0x83, 3, b"\x01\x02", ver=2)
    bad_len = bytes([0xA5, 0x5A, 0x01, 0x83, 0x00, 241])
    d = P.FrameDecoder()
    frames = d.feed(bytes(bad_crc) + bad_ver + bad_len + good)
    assert [f.decode() for f in frames] == [P.Ack(1, 2)]
    assert d.stats.crc_errors == 1 and d.stats.bad_version == 1 and d.stats.bad_length == 1


def test_decoder_finds_frame_hidden_inside_false_header():
    good = P.encode(P.Ack(4, 5), 6)
    # a false header that claims a 20-byte payload which swallows the real frame
    false = bytes([0xA5, 0x5A, 0x01, 0x82, 0x00, 20])
    d = P.FrameDecoder()
    frames = d.feed(false + good + b"\x00" * 20)
    assert [f.decode() for f in frames] == [P.Ack(4, 5)]


def test_decoder_sync_split_across_chunks():
    good = P.encode(P.Pong(1, 2, 1), 0)
    d = P.FrameDecoder()
    out = []
    for b in b"\x00\xa5" + good:
        out += d.feed(bytes([b]))
    assert [f.decode() for f in out] == [P.Pong(1, 2, 1)]


@settings(max_examples=300, deadline=None)
@given(
    st.lists(st.tuples(st.binary(max_size=40), msg_strategy, u8), min_size=1, max_size=8),
    st.binary(max_size=40),
    st.randoms(use_true_random=False),
)
def test_decoder_resync_with_garbage_and_random_splits(chunks, tail, rnd):
    stream = bytearray()
    expected = []
    for garbage, msg, seq in chunks:
        stream += garbage
        stream += P.encode(msg, seq)
        expected.append((msg.TYPE, seq, msg.pack()))
    # trailing garbage + padding so a false header in the garbage cannot hold back the last frame
    stream += tail + b"\x00" * 260
    d = P.FrameDecoder()
    got = []
    i = 0
    while i < len(stream):
        n = rnd.randint(1, 64)
        got += d.feed(bytes(stream[i : i + n]))
        i += n
    got_t = [(f.type, f.seq, f.payload) for f in got]
    # every real frame is recovered, in order (garbage may in theory decode as extra frames)
    it = iter(got_t)
    assert all(any(e == g for g in it) for e in expected), (expected, got_t)


def test_decoder_random_bit_flips_never_crash():
    rnd = random.Random(1)
    good = b"".join(P.encode(P.Ack(i, 2), i) for i in range(50))
    for _ in range(200):
        b = bytearray(good)
        for _ in range(rnd.randint(1, 10)):
            b[rnd.randrange(len(b))] ^= 1 << rnd.randrange(8)
        frames = P.FrameDecoder().feed(bytes(b) + b"\x00" * 260)
        assert len(frames) <= 50
        for f in frames:
            f.decode()  # all CRC-valid ACKs still parse


def test_commanded_pos_crc():
    q = (1, -2, 3, -4, 5, -6)
    assert P.commanded_pos_crc(q) == crc_bitwise(struct.pack("<6h", *q))
