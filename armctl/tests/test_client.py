"""ArmClient against the real-time MockBoard."""

import threading
import time

import numpy as np
import pytest

from armctl import protocol as P
from armctl.client import ArmClient, ArmTimeout, NackError, SafetyRejected
from armctl.mockboard import MockBoard, MockTransport
from armctl.safety import SafetyShim

SAFE = np.array([0.0, -1.2, 1.4, 0.6, 0.0, 0.5])


@pytest.fixture
def rig():
    board = MockBoard()
    tr = MockTransport(board)
    cli = ArmClient(tr)
    yield board, tr, cli
    cli.close()


def test_ping(rig):
    _, _, cli = rig
    pong = cli.ping()
    assert pong.proto_ver == 1 and pong.fw_version == MockBoard.FW_VERSION


def test_state_cache(rig):
    _, _, cli = rig
    st = cli.wait_for_state(timeout=0.5)
    assert st.mode == P.Mode.IDLE
    assert cli.state_age() < 0.1


def test_set_targets_converges(rig):
    board, _, cli = rig
    q = np.array([0.4, -0.6, 0.9, 0.3, -0.5, 1.2])
    cli.set_targets(q, 300)
    st = cli.wait_for_state(lambda s: np.allclose(s.positions_rad, q, atol=0.003), timeout=2.0)
    assert st.mode == P.Mode.ACTIVE


def test_ack_matching_concurrent(rig):
    board, _, cli = rig
    errors = []

    def worker(j):
        try:
            for v in range(10):
                cli.configure(j, P.ConfigKey.VMAX_MRAD_S, 1000 + 10 * j + v)
        except Exception as e:  # pragma: no cover
            errors.append(e)

    ts = [threading.Thread(target=worker, args=(j,)) for j in range(6)]
    [t.start() for t in ts]
    [t.join() for t in ts]
    assert not errors
    assert [c.vmax_mrad_s for c in board.jcfg] == [1009 + 10 * j for j in range(6)]
    assert cli.stats.timeouts == 0 and cli.stats.retries == 0


def test_nack_raises(rig):
    _, _, cli = rig
    with pytest.raises(NackError, match="UNKNOWN_CONFIG_KEY"):
        cli.configure(0, 0x7E, 1)
    with pytest.raises(NackError, match="BAD_VALUE"):
        cli.set_stream(10)


def test_retry_after_dropped_frame(rig):
    board, tr, cli = rig
    tr.drop_host_frames = 1  # first attempt never arrives
    cli.configure(0, P.ConfigKey.CURRENT_LIMIT_MA, 900)
    assert board.jcfg[0].current_limit_ma == 900
    assert cli.stats.retries == 1


def test_retry_after_dropped_ack(rig):
    board, _, cli = rig
    board.drop_acks = 1
    cli.configure(1, P.ConfigKey.CURRENT_LIMIT_MA, 800)
    assert cli.stats.retries == 1 and board.stats.acks_dropped == 1


def test_timeout_when_board_silent(rig):
    _, tr, cli = rig
    tr.drop_host_frames = 100
    t0 = time.monotonic()
    with pytest.raises(ArmTimeout):
        cli.request(P.ClearFault(0), timeout=0.05, retries=2)
    assert 0.14 < time.monotonic() - t0 < 0.5
    with pytest.raises(ArmTimeout):
        cli.ping(timeout=0.05, retries=0)
    tr.drop_host_frames = 0


def test_heartbeat_keeps_board_active(rig):
    board, _, cli = rig
    cli.set_targets(SAFE, 200)
    time.sleep(0.8)  # idle well past watchdog_off_ms
    st = cli.wait_for_state()
    assert st.mode == P.Mode.ACTIVE and st.fault_flags & P.LATCHED_FAULTS == 0
    assert cli.stats.heartbeats > 5


def test_watchdog_hold_then_fault_when_heartbeat_stops(rig):
    board, _, cli = rig
    cli.set_targets(SAFE, 200)
    cli.wait_for_state(lambda s: s.mode == P.Mode.ACTIVE)
    cli.pause_heartbeat()
    t0 = time.monotonic()
    st = cli.wait_for_state(lambda s: s.mode == P.Mode.HOLD, timeout=1.0)
    t_hold = time.monotonic() - t0
    assert st.fault_flags & P.LATCHED_FAULTS == 0
    st = cli.wait_for_state(lambda s: s.mode == P.Mode.FAULT, timeout=1.5)
    t_fault = time.monotonic() - t0
    assert st.fault_flags & P.Fault.WATCHDOG
    assert 0.08 < t_hold < 0.3 and 0.45 < t_fault < 0.8, (t_hold, t_fault)
    # recovery: clear, then torque-enable
    cli.resume_heartbeat()
    with pytest.raises(NackError, match="REJECTED_FAULTED"):
        cli.set_targets(SAFE, 200)
    cli.clear_faults()
    cli.wait_for_state(lambda s: s.fault_flags & P.LATCHED_FAULTS == 0)
    cli.enable()
    cli.wait_for_state(lambda s: s.mode == P.Mode.ACTIVE)


def test_heartbeat_max_idle_lets_watchdog_trip():
    board = MockBoard()
    with ArmClient(MockTransport(board), heartbeat_max_idle=0.2) as cli:
        cli.set_targets(SAFE, 100)
        t0 = time.monotonic()
        st = cli.wait_for_state(lambda s: s.mode == P.Mode.FAULT, timeout=2.0)
        assert st.fault_flags & P.Fault.WATCHDOG
        assert 0.6 < time.monotonic() - t0 < 1.2


def test_estop_and_two_step_clear(rig):
    board, _, cli = rig
    cli.set_targets(SAFE, 200)
    cli.estop()
    st = cli.wait_for_state(lambda s: s.mode == P.Mode.FAULT)
    assert st.fault_flags & P.Fault.ESTOP_SOFT
    assert all(j.status == 0 for j in st.joints)  # torque off
    with pytest.raises(NackError):
        cli.set_targets(SAFE, 200)
    cli.clear_faults()
    st = cli.wait_for_state(lambda s: s.fault_flags == 0)
    time.sleep(0.05)
    assert cli.wait_for_state().mode == P.Mode.FAULT  # step 1 done, still torque off
    cli.enable()  # step 2
    assert cli.wait_for_state(lambda s: s.mode == P.Mode.ACTIVE).joints[0].status == 1


def test_context_manager_torques_off():
    board = MockBoard()
    with ArmClient(MockTransport(board)) as cli:
        cli.set_targets(SAFE, 100)
        cli.wait_for_state(lambda s: s.mode == P.Mode.ACTIVE)
    assert not board.torque and board.mode == P.Mode.IDLE


def test_context_manager_torques_off_on_exception():
    board = MockBoard()
    with pytest.raises(RuntimeError):
        with ArmClient(MockTransport(board)) as cli:
            cli.set_targets(SAFE, 100)
            raise RuntimeError("boom")
    assert not board.torque


def test_client_safety_shim_applied():
    board = MockBoard(initial_q=SAFE)
    with ArmClient(MockTransport(board), safety=SafetyShim()) as cli:
        cli.wait_for_state()
        q = SAFE.copy()
        q[0] = 5.0  # beyond limit and too fast
        rep = cli.set_targets(q, 100)
        assert rep.modified and rep.q_out[0] == pytest.approx(SAFE[0] + 0.3)
        # immediately after enabling, the (still stale IDLE) cached state must not re-seed
        rep = cli.set_targets(q, 100)
        assert rep.q_out[0] == pytest.approx(SAFE[0] + 0.6)
        # from an unsafe previous pose, a command that makes it worse is rejected, unsent
        bad = np.array([0.0, 0.6, 0.0, -1.2, 0.0, 0.0])
        cli.safety.reset(bad)
        worse = bad.copy()
        worse[1] += 0.3
        n_st = sum(f.type == P.MsgType.SET_TARGETS for f in board.received)
        with pytest.raises(SafetyRejected):
            cli.set_targets(worse, 100)
        time.sleep(0.02)
        assert sum(f.type == P.MsgType.SET_TARGETS for f in board.received) == n_st


def test_open_serial_with_loopback_pty():
    """SerialTransport over a pseudo-terminal pair: the mock board serves the other end."""
    import os
    import pty


    master, slave = pty.openpty()
    import tty

    tty.setraw(slave)
    name = os.ttyname(slave)
    board = MockBoard()
    stop = threading.Event()

    def serve():
        os.set_blocking(master, False)
        last = time.monotonic()
        while not stop.is_set():
            try:
                data = os.read(master, 4096)
                board.receive(data)
            except BlockingIOError:
                pass
            now = time.monotonic()
            if now - last >= 0.005:
                board.step(now - last)
                last = now
            out = board.take_output()
            if out:
                os.write(master, out)
            time.sleep(0.001)

    th = threading.Thread(target=serve, daemon=True)
    th.start()
    try:
        cli = ArmClient.open_serial(name)
        try:
            assert cli.ping(timeout=0.3).proto_ver == 1
            assert cli.wait_for_state(timeout=1.0).mode == P.Mode.IDLE
        finally:
            cli.close()
    finally:
        stop.set()
        th.join(1)
        os.close(master)
        os.close(slave)
