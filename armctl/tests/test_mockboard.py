"""Deterministic (simulated-time) tests of the mock board against the spec."""

import numpy as np
import pytest

from armctl import protocol as P
from armctl.mockboard import MockBoard, quintic_coeffs, quintic_eval


class Host:
    """Tiny synchronous host for driving a MockBoard in simulated time."""

    def __init__(self, board=None):
        self.b = board or MockBoard()
        self.dec = P.FrameDecoder()
        self.seq = 0
        self.msgs = []

    def send(self, msg, raw=None):
        s = self.seq
        self.seq = (self.seq + 1) & 0xFF
        self.b.receive(raw if raw is not None else P.encode(msg, s))
        return s

    def collect(self):
        new = [f.decode() for f in self.dec.feed(self.b.take_output())]
        self.msgs += new
        return new

    def req(self, msg):
        s = self.send(msg)
        out = [m for m in self.collect() if isinstance(m, (P.Ack, P.Nack, P.Pong))]
        assert out, "no reply"
        r = out[-1]
        if isinstance(r, P.Ack):
            assert (r.acked_seq, r.acked_type) == (s, msg.TYPE)
        elif isinstance(r, P.Nack):
            assert (r.seq, r.type) == (s, msg.TYPE)
        return r

    def run(self, seconds, feed_every=0.05):
        """Advance time, PINGing every ``feed_every`` s (None = host silent)."""
        t = 0.0
        while t < seconds - 1e-9:
            dt = min(feed_every or seconds, seconds - t)
            self.b.step(dt)
            t += dt
            if feed_every:
                self.send(P.Ping())
        return self.collect()

    def state(self):
        st = [m for m in self.collect() if isinstance(m, P.State)]
        return st[-1] if st else self.b.make_state()


def targets(q, ms=300, flags=1):
    return P.SetTargets(ms, tuple(P.rad_to_mrad(x) for x in q), flags)


def test_ping_pong_and_boot_state():
    h = Host()
    r = h.req(P.Ping())
    assert isinstance(r, P.Pong) and r.proto_ver == 1
    h.run(0.05)
    st = h.state()
    assert st.mode == P.Mode.IDLE and st.fault_flags == 0


def test_default_stream_200hz_and_set_stream():
    h = Host()
    msgs = h.run(1.0, feed_every=0.05)
    n = sum(isinstance(m, P.State) for m in msgs)
    assert 195 <= n <= 205
    assert isinstance(h.req(P.SetStream(10000)), P.Ack)
    n = sum(isinstance(m, P.State) for m in h.run(1.0))
    assert 98 <= n <= 102
    assert isinstance(h.req(P.SetStream(0)), P.Ack)
    assert not any(isinstance(m, P.State) for m in h.run(0.3))
    r = h.req(P.SetStream(999))
    assert isinstance(r, P.Nack) and r.error == P.NackError.BAD_VALUE


def test_set_targets_converges_and_reports():
    h = Host()
    q = [0.3, -0.5, 0.8, 0.2, -0.4, 1.0]
    assert isinstance(h.req(targets(q, 400)), P.Ack)
    h.run(1.0)
    st = h.state()
    assert st.mode == P.Mode.ACTIVE
    np.testing.assert_allclose(st.positions_rad, q, atol=0.002)
    assert st.cmd_pos_crc == P.commanded_pos_crc([P.rad_to_mrad(x) for x in q])
    assert st.last_host_seq == (h.seq - 1) & 0xFF


def test_nack_codes():
    h = Host()
    bad_len = P.encode_frame(P.MsgType.CLEAR_FAULT, 0, b"\x01")
    h.send(None, raw=bad_len)
    assert h.collect()[-1] == P.Nack(0, P.MsgType.CLEAR_FAULT, P.NackError.BAD_LENGTH)
    h.send(None, raw=P.encode_frame(0x42, 9, b""))
    assert h.collect()[-1] == P.Nack(9, 0x42, P.NackError.UNKNOWN_TYPE)
    h.send(None, raw=P.encode_frame(P.MsgType.STATE, 10, bytes(66)))  # board->host type
    assert h.collect()[-1].error == P.NackError.UNKNOWN_TYPE
    assert h.req(targets([0] * 6, flags=0x03)).error == P.NackError.BAD_VALUE
    assert h.req(targets([0] * 6, ms=0)).error == P.NackError.BAD_VALUE
    assert h.req(targets([0] * 6, ms=5001)).error == P.NackError.BAD_VALUE
    assert h.req(P.Config(0, 0x7E, 1)).error == P.NackError.UNKNOWN_CONFIG_KEY
    assert h.req(P.Config(0, P.ConfigKey.WATCHDOG_HOLD_MS, 50)).error == P.NackError.BAD_VALUE
    assert h.req(P.Config(0xFF, P.ConfigKey.VMAX_MRAD_S, 50)).error == P.NackError.BAD_VALUE
    assert h.req(P.Config(6, P.ConfigKey.VMAX_MRAD_S, 50)).error == P.NackError.BAD_VALUE
    assert h.req(P.Config(1, P.ConfigKey.DIRECTION, 2)).error == P.NackError.BAD_VALUE
    assert isinstance(h.req(P.Config(1, P.ConfigKey.DIRECTION, -1)), P.Ack)


def test_bad_crc_frame_dropped_silently():
    h = Host()
    raw = bytearray(P.encode(P.Ping(), 0))
    raw[-1] ^= 1
    h.send(None, raw=bytes(raw))
    assert h.collect() == []


def test_replan_is_continuous():
    b = MockBoard()
    h = Host(b)
    h.req(targets([1.0, 0, 0, 0, 0, 0], 1000))
    hist = []
    for k in range(600):
        if k == 300:
            h.req(targets([-0.5, 0, 0, 0, 0, 0], 800))
        b.step(0.001)
        if k % 50 == 0:
            h.send(P.Ping())
        hist.append((b.cmd_p[0], b.cmd_v[0], b.cmd_a[0]))
    hist = np.array(hist)
    # velocity and acceleration have no jumps across the replan at k=300
    assert np.max(np.abs(np.diff(hist[:, 1]))) < 0.05
    assert np.max(np.abs(np.diff(hist[:, 2]))) < 1.0


def test_quintic_boundary_conditions():
    c = quintic_coeffs([0.1], [0.5], [-2.0], [1.0], 0.7)
    p, v, a = quintic_eval(c, 0.0)
    assert (p[0], v[0], a[0]) == pytest.approx((0.1, 0.5, -2.0))
    p, v, a = quintic_eval(c, 0.7)
    assert (p[0], v[0], a[0]) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)


def test_vmax_stretch_and_limit_clamp_flag():
    b = MockBoard()
    h = Host(b)
    h.req(targets([2.0, 0, 0, 0, 0, 0], 100))  # 2 rad in 0.1 s -> far above 3 rad/s
    assert b.faults & P.Fault.LIMIT_CLAMP
    assert b.traj_T > 0.1
    peak = 0.0
    for _ in range(2000):
        b.step(0.001)
        peak = max(peak, abs(b.cmd_v[0]))
        if _ % 50 == 0:
            h.send(P.Ping())
    assert peak <= 3.0 + 1e-3
    h.req(targets([2.0, 0, 0, 0, 0, 0], 1000))  # in range, slow: flag self-clears
    assert not b.faults & P.Fault.LIMIT_CLAMP
    h.req(targets([9.0, 0, 0, 0, 0, 0], 5000))  # beyond the 2.6 limit
    assert b.faults & P.Fault.LIMIT_CLAMP
    h.run(5.5)
    assert h.state().positions_rad[0] == pytest.approx(2.6, abs=0.002)
    assert h.state().mode == P.Mode.ACTIVE  # informational only


def test_watchdog_hold_then_fault():
    b = MockBoard()
    h = Host(b)
    h.req(targets([0.5, 0, 0, 0, 0, 0], 2000))
    h.run(0.2)
    pos_before = b.cmd_p.copy()
    b.step(0.090)
    assert b.mode == P.Mode.ACTIVE
    b.step(0.020)  # > 100 ms silent
    assert b.mode == P.Mode.HOLD and b.torque
    held = b.cmd_p.copy()
    assert held[0] > pos_before[0]  # move was in progress, now frozen
    b.step(0.300)
    np.testing.assert_allclose(b.cmd_p, held)
    assert b.mode == P.Mode.HOLD
    b.step(0.100)  # > 500 ms silent
    assert b.mode == P.Mode.FAULT and not b.torque
    assert b.faults & P.Fault.WATCHDOG
    # host back: SET_TARGETS rejected until the latched bit is cleared
    assert h.req(targets([0] * 6)).error == P.NackError.REJECTED_FAULTED


def test_hold_exits_on_host_frame():
    b = MockBoard()
    h = Host(b)
    h.req(targets([0.2, 0, 0, 0, 0, 0], 100))
    b.step(0.15)
    assert b.mode == P.Mode.HOLD
    h.req(P.Ping())
    assert b.mode == P.Mode.ACTIVE


def test_watchdog_not_armed_when_idle():
    b = MockBoard()
    b.step(2.0)
    assert b.mode == P.Mode.IDLE and b.faults == 0


def test_watchdog_config():
    b = MockBoard()
    h = Host(b)
    assert isinstance(h.req(P.Config(0xFF, P.ConfigKey.WATCHDOG_HOLD_MS, 30)), P.Ack)
    assert isinstance(h.req(P.Config(0xFF, P.ConfigKey.WATCHDOG_OFF_MS, 60)), P.Ack)
    assert h.req(P.Config(0xFF, P.ConfigKey.WATCHDOG_OFF_MS, 20)).error == P.NackError.BAD_VALUE
    h.req(targets([0] * 6))
    b.step(0.035)
    assert b.mode == P.Mode.HOLD
    b.step(0.030)
    assert b.mode == P.Mode.FAULT


def test_estop_and_two_step_clear():
    b = MockBoard()
    h = Host(b)
    h.req(targets([0.3] * 6, 300))
    h.run(0.5)
    assert isinstance(h.req(P.EStop()), P.Ack)
    assert b.mode == P.Mode.FAULT and not b.torque and b.faults & P.Fault.ESTOP_SOFT
    assert h.req(targets([0.3] * 6)).error == P.NackError.REJECTED_FAULTED
    assert isinstance(h.req(P.ClearFault(int(P.Fault.ESTOP_SOFT))), P.Ack)
    assert b.faults == 0
    h.run(0.2)
    assert b.mode == P.Mode.FAULT and not b.torque  # still torque off after clear
    assert isinstance(h.req(targets([0.3] * 6)), P.Ack)
    assert b.mode == P.Mode.ACTIVE and b.torque


def test_partial_clear_keeps_fault():
    b = MockBoard()
    h = Host(b)
    b.inject_fault(P.Fault.OVERTEMP | P.Fault.SERVO_COMMS)
    h.req(P.ClearFault(int(P.Fault.OVERTEMP)))
    assert b.faults == P.Fault.SERVO_COMMS
    assert h.req(targets([0] * 6)).error == P.NackError.REJECTED_FAULTED
    h.req(P.ClearFault(0xFFFF))
    assert isinstance(h.req(targets([0] * 6)), P.Ack)


def test_hw_estop_needs_loop_closed():
    b = MockBoard()
    h = Host(b)
    h.req(targets([0] * 6))
    b.set_hw_estop(True)
    assert b.mode == P.Mode.FAULT and not b.torque
    h.req(P.ClearFault(int(P.Fault.ESTOP_HW)))
    assert b.faults & P.Fault.ESTOP_HW
    b.set_hw_estop(False)
    assert b.faults & P.Fault.ESTOP_HW  # still latched until the host clears it
    h.req(P.ClearFault(int(P.Fault.ESTOP_HW)))
    assert not b.faults
    assert isinstance(h.req(targets([0] * 6)), P.Ack)


def test_undervolt_flag_tracks_voltage():
    b = MockBoard()
    h = Host(b)
    h.req(targets([0] * 6))
    b.set_bus_voltage(10000)
    h.run(0.05)
    st = h.state()
    assert st.fault_flags & P.Fault.BUS_UNDERVOLT and st.mode == P.Mode.ACTIVE
    b.set_bus_voltage(12000)
    h.run(0.05)
    assert not h.state().fault_flags & P.Fault.BUS_UNDERVOLT


def test_torque_off_from_active():
    b = MockBoard()
    h = Host(b)
    h.req(targets([0.2] * 6))
    h.run(0.5)
    h.req(targets([0] * 6, flags=0))
    assert b.mode == P.Mode.IDLE and not b.torque
    q = b.measured_q().copy()
    h.run(0.5)
    np.testing.assert_allclose(b.measured_q(), q)  # limp, no motion


def test_config_offset_and_direction_mapping():
    b = MockBoard(mount_offset_ticks=[100, 0, 0, 0, 0, 0], initial_q=[0.2, 0, 0, 0, 0, 0])
    h = Host(b)
    # identity mapping: reported = (ticks - 2048) * k = true + 100 ticks
    k = 2 * np.pi / 4096
    assert b.measured_q()[0] == pytest.approx(0.2 + 100 * k)
    h.req(P.Config(0, P.ConfigKey.OFFSET_TICKS, 100))
    assert b.measured_q()[0] == pytest.approx(0.2)
    h.req(P.Config(0, P.ConfigKey.DIRECTION, -1))
    assert b.measured_q()[0] == pytest.approx(-0.2)
