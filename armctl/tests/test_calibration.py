import time

import numpy as np
import pytest

from armctl import protocol as P
from armctl.calibration import (
    Calibration,
    GripperMap,
    JointCal,
    LeaderMap,
    LinearMap,
    calibrate_at_known_pose,
    compute_directions,
    compute_offsets,
)
from armctl.client import ArmClient
from armctl.mockboard import MockBoard, MockTransport

K = 2 * np.pi / 4096


def test_yaml_roundtrip(tmp_path):
    cal = Calibration(
        joints=[JointCal(10 * i - 20, 1 if i % 2 else -1) for i in range(6)],
        leader_to_follower=LeaderMap(
            joints=[LinearMap(1.0, 0.01 * i) for i in range(5)],
            gripper=GripperMap(0.1, 0.9, 0.0, 1.6),
        ),
        arm="leader",
    )
    p = tmp_path / "cal.yaml"
    cal.save(p)
    text = p.read_text()
    assert "base_yaw" in text and "leader_to_follower" in text
    assert Calibration.load(p) == cal


def test_yaml_validation(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("version: 1\njoints: {base_yaw: {offset_ticks: 1}}\n")
    with pytest.raises(ValueError, match="missing"):
        Calibration.load(p)
    d = Calibration().to_dict()
    d["joints"]["gripper"]["direction"] = 2
    with pytest.raises(ValueError, match="direction"):
        Calibration.from_dict(d)


def test_compute_offsets_math():
    true_off = np.array([100, -37, 0, 512, -2000, 7])
    dirs = [1, -1, 1, -1, 1, -1]
    known = np.array([0.0, -1.5, 1.2, 0.3, 0.0, 0.0])
    # simulate: ticks_rel = known/(dir*k) + off ; identity read = ticks_rel * k
    raw = (known / (np.array(dirs) * K) + true_off) * K
    assert compute_offsets(raw, known, dirs) == list(true_off)


def test_compute_directions():
    pa = np.array([0, 0, 0, 0, 0, 0.0])
    pb = np.array([0.5, -0.5, 0.5, 0.5, 0.5, 0.5])
    dirs = np.array([1, -1, -1, 1, 1, -1])
    raw_a = np.zeros(6) + 0.1
    raw_b = raw_a + dirs * pb * np.array([1, 1, -1, 1, 1, 1]) * np.sign(pb)
    got = compute_directions(raw_a, raw_b, pa, pb)
    expected = [int(np.sign((rb - ra) * (b - a))) for ra, rb, a, b in zip(raw_a, raw_b, pa, pb)]
    assert got == expected
    with pytest.raises(ValueError):
        compute_directions(raw_a, raw_b, pa, pa)


def test_gripper_and_leader_map():
    g = GripperMap(leader_closed_rad=0.2, leader_open_rad=1.0, follower_closed_rad=0.0, follower_open_rad=1.6)
    assert g(0.2) == pytest.approx(0.0)
    assert g(0.6) == pytest.approx(0.8)
    assert g(1.0) == pytest.approx(1.6)
    assert g(2.0) == pytest.approx(1.6) and g(-1.0) == pytest.approx(0.0)
    # inverted handle (leader closes when angle increases)
    gi = GripperMap(1.0, 0.2, 0.0, 1.6)
    assert gi(1.0) == pytest.approx(0.0) and gi(0.2) == pytest.approx(1.6)
    lm = LeaderMap(joints=[LinearMap(-1.0, 0.1)] + [LinearMap()] * 4, gripper=g)
    out = lm([0.5, 0.1, 0.2, 0.3, 0.4, 0.6])
    np.testing.assert_allclose(out, [-0.4, 0.1, 0.2, 0.3, 0.4, 0.8])


def test_calibrate_against_mock_with_hidden_mount_offsets():
    mount_off = [123, -456, 789, -10, 2000, -300]
    mount_dir = [1, -1, 1, -1, 1, -1]
    known = np.array([0.0, -1.57, 1.57, 0.0, 0.0, 0.0])  # e.g. a printed calibration jig pose
    board = MockBoard(mount_offset_ticks=mount_off, mount_direction=mount_dir, initial_q=known)
    with ArmClient(MockTransport(board)) as cli:
        cal = calibrate_at_known_pose(cli, known, mount_dir)
        assert [j.offset_ticks for j in cal.joints] == [
            # board formula: q = dir*(ticks-2048-off)*k ; mount: q = mdir*(ticks-2048-moff)*k
            # with dir == mdir this gives off == moff
            o for o in mount_off
        ]
        # the board now reports true physical angles everywhere, not only at the jig pose
        board.backdrive([0.3, -0.8, 0.5, 0.2, -0.4, 0.9])
        st = cli.wait_for_state(timeout=0.5)
        np.testing.assert_allclose(st.positions_rad, [0.3, -0.8, 0.5, 0.2, -0.4, 0.9], atol=0.002)
        # and commanded targets land at the right physical angles
        cli.set_targets([0.1, -1.0, 1.0, 0.3, 0.0, 0.5], 200)
        cli.wait_for_state(lambda s: np.allclose(s.positions_rad, [0.1, -1.0, 1.0, 0.3, 0.0, 0.5], atol=0.003), 2.0)
        time.sleep(0.2)  # let the first-order servo settle
        # residual = offset rounding (<= 1 tick = 1.5 mrad) + mrad quantisation of the reading
        np.testing.assert_allclose(board.true_q(), [0.1, -1.0, 1.0, 0.3, 0.0, 0.5], atol=0.0035)


def test_push_refuses_under_torque():
    board = MockBoard()
    with ArmClient(MockTransport(board)) as cli:
        cli.set_targets([0, -1.2, 1.4, 0.6, 0, 0.5], 100)
        cli.wait_for_state(lambda s: s.mode == P.Mode.ACTIVE)
        with pytest.raises(RuntimeError, match="torque"):
            Calibration().push(cli)
