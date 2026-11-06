"""Cross-check: the real armctl ArmClient driving the compiled C firmware (fw_sim) over a pipe.

armctl was tested against its own Python mock board, and the firmware against its own C tests.
This is the first test where the two independent implementations of spec/protocol.md talk.
Build first:  cmake -S firmware -B firmware/build && cmake --build firmware/build
"""
import os
import select
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest

from armctl import protocol as P
from armctl.client import ArmClient, ArmTimeout, NackError

ROOT = Path(__file__).resolve().parents[2]
FW_SIM = ROOT / "firmware" / "build" / "fw_sim"
pytestmark = pytest.mark.skipif(not FW_SIM.exists(), reason="fw_sim not built")


class PipeTransport:
    def __init__(self, argv):
        self.p = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        self.fd = self.p.stdout.fileno()

    def write(self, data: bytes) -> int:
        self.p.stdin.write(data)
        return len(data)

    def read(self, size: int = 4096) -> bytes:
        r, _, _ = select.select([self.fd], [], [], 0.01)
        return os.read(self.fd, size) if r else b""

    def close(self) -> None:
        try:
            self.p.stdin.close()
        except OSError:
            pass
        try:
            self.p.wait(timeout=2)
        except subprocess.TimeoutExpired:
            self.p.kill()


@pytest.fixture
def client():
    tr = PipeTransport([str(FW_SIM)])
    c = ArmClient(tr, ack_timeout=0.2, retries=2)
    yield c
    c.close()


def test_ping_and_state_stream(client):
    pong = client.ping()
    assert pong.proto_ver == 1
    t0 = time.monotonic()
    n0 = client._state_count
    time.sleep(1.0)
    rate = (client._state_count - n0) / (time.monotonic() - t0)
    assert 170 < rate < 230, rate  # default 200 Hz
    st = client.state
    assert st.mode == P.Mode.IDLE and st.fault_flags == 0


def test_move_converges(client):
    client.enable(move_ms=200)
    target = np.array([0.3, -0.4, 0.8, -0.3, 0.5, 0.8])
    client.set_targets(target, move_ms=600)
    st = client.wait_for_state(lambda s: np.max(np.abs(np.array(s.positions_rad) - target)) < 0.01, timeout=3.0)
    assert st.mode == P.Mode.ACTIVE


def test_config_rejected_while_torque_on_then_accepted(client):
    client.enable(move_ms=100)
    with pytest.raises(NackError):
        client.configure(0, 0x01, 10)  # offset while torque on -> NACK 2 (firmware choice)
    client.torque_off()
    client.configure(0, 0x05, 2000)     # vmax, torque off -> ACK


def test_watchdog_hold_then_fault(client):
    client.enable(move_ms=100)
    client.pause_heartbeat()
    client.wait_for_state(lambda s: s.mode == P.Mode.HOLD, timeout=0.4)
    st = client.wait_for_state(lambda s: s.mode == P.Mode.FAULT, timeout=1.0)
    assert st.fault_flags & (1 << 2)  # WATCHDOG
    client.resume_heartbeat()
    client.clear_faults()
    st = client.wait_for_state(timeout=0.5)
    assert st.mode == P.Mode.FAULT and st.fault_flags == 0  # two-step clear
    client.enable(move_ms=100)
    client.wait_for_state(lambda s: s.mode == P.Mode.ACTIVE, timeout=0.5)


def test_estop_and_clear(client):
    client.enable(move_ms=100)
    client.estop()
    st = client.wait_for_state(lambda s: s.mode == P.Mode.FAULT, timeout=0.5)
    assert st.fault_flags & 1
    with pytest.raises(NackError):
        client.set_targets(client.positions(), move_ms=100, apply_safety=False)  # NACK 4 while faulted
    client.clear_faults()
    client.enable(move_ms=100)
    client.wait_for_state(lambda s: s.mode == P.Mode.ACTIVE, timeout=0.5)


def test_limit_clamp_flag(client):
    client.enable(move_ms=100)
    q = client.positions()
    q[0] = 3.0  # beyond base_yaw limit 2.60
    client.set_targets(q, move_ms=500, apply_safety=False)
    st = client.wait_for_state(lambda s: s.fault_flags & (1 << 7), timeout=0.5)
    st = client.wait_for_state(lambda s: abs(s.positions_rad[0] - 2.60) < 0.01, timeout=3.0)
    assert st.positions_rad[0] <= 2.601
