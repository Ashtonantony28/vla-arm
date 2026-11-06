import math
import threading
import time

import numpy as np
import pytest

from armctl import protocol as P
from armctl.calibration import GripperMap, LeaderMap, LinearMap
from armctl.client import ArmClient
from armctl.mockboard import MockBoard, MockTransport
from armctl.safety import SafetyShim
from armctl.teleop import FollowerFaulted, TeleopLoop

HOME = np.array([0.0, -1.2, 1.4, 0.6, 0.0, 0.5])


@pytest.fixture
def pair():
    lb = MockBoard(initial_q=HOME)
    fb = MockBoard(initial_q=HOME)
    leader = ArmClient(MockTransport(lb))
    follower = ArmClient(MockTransport(fb))
    leader.wait_for_state()
    follower.wait_for_state()
    yield lb, fb, leader, follower
    leader.close()
    follower.close()


def leader_traj(t):
    q = HOME.copy()
    q[0] = 0.5 * math.sin(2 * math.pi * 0.5 * t)
    q[1] = -1.2 + 0.2 * math.sin(2 * math.pi * 0.3 * t)
    q[5] = 0.5 + 0.4 * math.sin(2 * math.pi * 0.7 * t)
    return q


def test_teleop_follower_tracks_leader(pair):
    lb, fb, leader, follower = pair
    loop = TeleopLoop(leader, follower, LeaderMap(), SafetyShim(), rate_hz=100)
    stop = threading.Event()
    errs = []

    def operator():
        t0 = time.monotonic()
        while not stop.is_set():
            t = time.monotonic() - t0
            lb.backdrive(leader_traj(t))
            if t > 0.5:  # after start-up transient
                errs.append(np.abs(fb.true_q() - leader_traj(t - 0.04)))  # ~40 ms pipeline lag
            time.sleep(0.005)

    th = threading.Thread(target=operator)
    th.start()
    stats = loop.run(duration=2.0)
    stop.set()
    th.join()
    assert fb.mode == P.Mode.ACTIVE and not fb.faults & P.LATCHED_FAULTS
    assert lb.mode == P.Mode.IDLE  # leader stays limp
    assert stats.sent > 150 and stats.safety_rejected == 0
    errs = np.array(errs)
    assert np.max(errs) < 0.06, np.max(errs, axis=0)
    assert np.mean(errs) < 0.02


def test_teleop_step_with_mapping_and_gripper_scaling(pair):
    lb, fb, leader, follower = pair
    mapping = LeaderMap(
        joints=[LinearMap(-1.0, 0.0)] + [LinearMap() for _ in range(4)],  # mirrored base
        gripper=GripperMap(leader_closed_rad=0.2, leader_open_rad=0.6, follower_closed_rad=0.0, follower_open_rad=1.6),
    )
    loop = TeleopLoop(leader, follower, mapping, SafetyShim(), rate_hz=50)
    q_lead = HOME.copy()
    q_lead[0] = 0.3
    q_lead[5] = 0.4  # half-open on the leader handle
    lb.backdrive(q_lead)
    leader.wait_for_state(lambda s: abs(s.positions_rad[0] - 0.3) < 0.002)
    for _ in range(100):
        sent = loop.step()
        time.sleep(0.02)
    assert sent[0] == pytest.approx(-0.3, abs=0.003)
    assert sent[5] == pytest.approx(0.8, abs=0.01)
    follower.wait_for_state(lambda s: abs(s.positions_rad[0] + 0.3) < 0.005, timeout=1.0)


def test_teleop_safety_limits_jump_and_table(pair):
    lb, fb, leader, follower = pair
    loop = TeleopLoop(leader, follower, LeaderMap(), SafetyShim(), rate_hz=100)
    loop.start()
    jump = HOME.copy()
    jump[0] = 2.0  # sudden 2 rad leader jump
    lb.backdrive(jump)
    leader.wait_for_state(lambda s: abs(s.positions_rad[0] - 2.0) < 0.002)
    sent = loop.step()
    assert sent[0] <= HOME[0] + 3.0 * loop.move_ms / 1000 + 1e-9
    assert loop.stats.safety_modified == 1
    # leader driven into the table: follower command never goes below z_min
    low = np.array([0.0, 0.6, 0.0, -1.2, 0.0, 0.0])
    lb.backdrive(low)
    leader.wait_for_state(lambda s: abs(s.positions_rad[1] - 0.6) < 0.002)
    from armctl.kinematics import joint_positions

    for _ in range(200):
        sent = loop.step()
        if sent is not None:
            pts = joint_positions(sent)
            assert pts["tcp"][2] >= 0.01 - 1e-6 and pts["wrist"][2] >= 0.01 - 1e-6
        time.sleep(0.002)


def test_teleop_stale_leader_skips(pair):
    lb, fb, leader, follower = pair
    loop = TeleopLoop(leader, follower, rate_hz=100, max_leader_age=0.05)
    loop.start()
    leader.set_stream(0)  # leader stops streaming
    time.sleep(0.1)
    assert loop.step() is None
    assert loop.stats.stale_leader == 1


def test_teleop_stops_on_follower_fault(pair):
    lb, fb, leader, follower = pair
    loop = TeleopLoop(leader, follower, rate_hz=100)
    loop.start()
    follower.estop()
    follower.wait_for_state(lambda s: s.mode == P.Mode.FAULT)
    with pytest.raises(FollowerFaulted):
        loop.step()
    with pytest.raises(FollowerFaulted):
        TeleopLoop(leader, follower).start()
