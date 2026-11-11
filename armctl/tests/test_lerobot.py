"""LeRobot integration: API conformance and an end-to-end ``lerobot-record`` run with two mock
boards and fake cameras. Skipped when lerobot is not installed (it is an optional extra); run with
the sim venv:

    PYTHONPATH=armctl /home/claude/vla-arm/sim/.venv/bin/python -m pytest armctl/tests/test_lerobot.py
"""

import inspect
import math
import sys
import threading
import time

import numpy as np
import pytest

lerobot = pytest.importorskip("lerobot")

import armctl.lerobot as AL  # noqa: E402
from armctl import protocol as P  # noqa: E402
from armctl.lerobot import scripts  # noqa: E402
from lerobot.robots.robot import Robot  # noqa: E402
from lerobot.teleoperators.teleoperator import Teleoperator  # noqa: E402

HOME = np.array(AL.MOCK_HOME)
FPS = 30
CAMS = {"wrist": (48, 64), "scene": (40, 56)}


def leader_traj(t):
    q = HOME.copy()
    q[0] = 0.4 * math.sin(2 * math.pi * 0.5 * t)
    q[1] = -1.2 + 0.15 * math.sin(2 * math.pi * 0.4 * t)
    q[5] = 0.5 + 0.4 * math.sin(2 * math.pi * 0.7 * t)
    return q


@pytest.fixture(autouse=True)
def fresh_mocks():
    AL.forget_mock_board()
    yield
    AL.forget_mock_board()


def test_api_matches_lerobot():
    assert lerobot.__version__ == "0.7.0"
    for cls, base in ((AL.ArmctlFollower, Robot), (AL.ArmctlLeader, Teleoperator)):
        assert issubclass(cls, base) and not inspect.isabstract(cls)
        assert cls.config_class.__name__ == cls.__name__ + "Config"
    assert AL.ArmctlFollowerConfig().type == "armctl_follower"
    assert AL.ArmctlLeaderConfig().type == "armctl_leader"
    assert AL.ArmctlFakeCameraConfig().type == "armctl_fake"
    assert AL.ArmctlFollower.supports_position_hold


def test_factories_and_features(tmp_path):
    from lerobot.robots.utils import make_robot_from_config
    from lerobot.teleoperators.utils import make_teleoperator_from_config

    cams = {k: AL.ArmctlFakeCameraConfig(height=h, width=w, fps=FPS) for k, (h, w) in CAMS.items()}
    robot = make_robot_from_config(AL.ArmctlFollowerConfig(port="mock", cameras=cams))
    teleop = make_teleoperator_from_config(AL.ArmctlLeaderConfig(port="mock", calibration_dir=tmp_path))
    assert isinstance(robot, AL.ArmctlFollower) and isinstance(teleop, AL.ArmctlLeader)
    keys = [f"{n}.pos" for n in P.JOINT_NAMES]
    assert list(robot.action_features) == keys == list(teleop.action_features)
    assert robot.observation_features == {**{k: float for k in keys}, "wrist": (48, 64, 3), "scene": (40, 56, 3)}
    assert teleop.feedback_features == {}
    robot.connect()
    teleop.connect()
    try:
        obs = robot.get_observation()
        assert set(obs) == set(robot.observation_features)
        assert obs["wrist"].shape == (48, 64, 3) and obs["wrist"].dtype == np.uint8
        np.testing.assert_allclose([obs[k] for k in keys], HOME, atol=2e-3)
        np.testing.assert_allclose(list(teleop.get_action().values()), HOME, atol=2e-3)
    finally:
        robot.disconnect()
        teleop.disconnect()
    assert not robot.is_connected and not teleop.is_connected


def test_send_action_goes_through_safety():
    robot = AL.ArmctlFollower(AL.ArmctlFollowerConfig(port="mock:f"))
    robot.connect()
    try:
        board = AL.mock_board("f")
        assert board.mode == P.Mode.ACTIVE
        # a far target is velocity-limited: what was sent (and is returned) differs
        far = dict(zip(AL.JOINT_KEYS, HOME + np.array([1.0, 0, 0, 0, 0, 0])))
        sent = robot.send_action(far)
        assert 0 < sent["base_yaw.pos"] < 0.5 and robot.safety_modified == 1
        # a partial action keeps the other joints at their last target
        sent2 = robot.send_action({"gripper.pos": 0.55})
        assert sent2["gripper.pos"] == pytest.approx(0.55) and sent2["base_yaw.pos"] == pytest.approx(sent["base_yaw.pos"])
        # driving the gripper into the table: never sent as is
        down = dict(zip(AL.JOINT_KEYS, np.array([0.0, 0.5, 0.5, 0.5, 0.0, 0.0])))
        for _ in range(60):
            robot.send_action(down)
            time.sleep(0.005)
        assert robot.shim.violations(robot._last_sent) == []
        # resending the last applied targets (LeRobot position hold) is accepted untouched
        last = robot._to_dict(robot._last_sent)
        assert robot.send_action(last) == pytest.approx(last)
    finally:
        robot.disconnect()
    assert board.mode == P.Mode.IDLE  # torque off on disconnect


def test_follower_refuses_faulted_board():
    board = AL.mock_board("bad")
    board.inject_fault(P.Fault.OVERTEMP)
    robot = AL.ArmctlFollower(AL.ArmctlFollowerConfig(port="mock:bad"))
    with pytest.raises(AL.FollowerFault):
        robot.connect()
    assert robot.client is None


def test_record_two_episodes_end_to_end(tmp_path, monkeypatch):
    """`armctl-lerobot-record` (= lerobot-record + armctl registration), driven by CLI args."""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    lead = AL.mock_board("lead")
    stop = threading.Event()

    def operator():
        t0 = time.monotonic()
        while not stop.is_set():
            lead.backdrive(leader_traj(time.monotonic() - t0))
            time.sleep(0.005)

    root = tmp_path / "ds"
    cams = ", ".join(f"{k}: {{type: armctl_fake, height: {h}, width: {w}, fps: {FPS}}}" for k, (h, w) in CAMS.items())
    argv = [
        "armctl-lerobot-record",
        "--robot.type=armctl_follower",
        "--robot.port=mock:follow",
        f"--robot.cameras={{{cams}}}",
        "--robot.id=test_follower",
        f"--robot.calibration_dir={tmp_path / 'cal'}",
        "--teleop.type=armctl_leader",
        "--teleop.port=mock:lead",
        f"--teleop.calibration_dir={tmp_path / 'cal'}",
        "--dataset.repo_id=local/armctl_e2e",
        f"--dataset.root={root}",
        "--dataset.single_task=wave the arm",
        f"--dataset.fps={FPS}",
        "--dataset.episode_time_s=1.2",
        "--dataset.reset_time_s=0.3",
        "--dataset.num_episodes=2",
        "--dataset.video=false",
        "--dataset.push_to_hub=false",
        "--dataset.no_stamp=true",
        "--dataset.num_image_writer_threads_per_camera=1",
        "--play_sounds=false",
    ]
    monkeypatch.setattr(sys, "argv", argv)
    th = threading.Thread(target=operator, daemon=True)
    th.start()
    try:
        scripts.record_main()
    finally:
        stop.set()
        th.join()
    follow = AL.mock_board("follow")
    assert not follow.faults & P.LATCHED_FAULTS and follow.mode == P.Mode.IDLE  # torque off at exit

    ds = LeRobotDataset("local/armctl_e2e", root=root)
    assert ds.fps == FPS and ds.meta.robot_type == "armctl_follower"
    assert ds.num_episodes == 2
    f = ds.features
    keys = [f"{n}.pos" for n in P.JOINT_NAMES]
    assert f["action"]["shape"] == (6,) and f["action"]["names"] == keys
    assert f["observation.state"]["shape"] == (6,) and f["observation.state"]["names"] == keys
    for k, (h, w) in CAMS.items():
        assert f[f"observation.images.{k}"]["dtype"] == "image"
        assert tuple(f[f"observation.images.{k}"]["shape"]) == (h, w, 3)
    lengths = [int(e["length"]) for e in ds.meta.episodes]
    # 1.2 s at 30 fps = 36 frames; allow a few dropped ticks on a loaded CI box
    assert all(30 <= n <= 36 for n in lengths) and ds.num_frames == sum(lengths)

    item = ds[lengths[0] + 5]
    assert int(item["episode_index"]) == 1 and item["task"] == "wave the arm"
    assert tuple(item["observation.images.wrist"].shape) == (3, 48, 64)
    assert tuple(item["observation.state"].shape) == (6,) and tuple(item["action"].shape) == (6,)
    ts = np.array([float(ds[i]["timestamp"]) for i in range(lengths[0])])
    np.testing.assert_allclose(np.diff(ts), 1 / FPS, atol=1e-4)

    # the recording shows the follower tracking the leader
    acts = np.stack([ds[i]["action"].numpy() for i in range(ds.num_frames)])
    states = np.stack([ds[i]["observation.state"].numpy() for i in range(ds.num_frames)])
    assert np.ptp(acts[:, 0]) > 0.3  # the operator moved the base
    assert np.ptp(states[:, 0]) > 0.25  # and the follower followed
    lag = 3  # ~0.1 s pipeline + move_ms blend
    assert np.median(np.abs(states[lag:, 0] - acts[:-lag, 0])) < 0.06
