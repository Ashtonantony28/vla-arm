"""draccus config classes for the LeRobot integration.

Registered choice names (usable as ``--robot.type=`` / ``--teleop.type=`` / camera ``type:``):

* ``armctl_follower``  -> :class:`ArmctlFollowerConfig`  -> :class:`~armctl.lerobot.ArmctlFollower`
* ``armctl_leader``    -> :class:`ArmctlLeaderConfig`    -> :class:`~armctl.lerobot.ArmctlLeader`
* ``armctl_fake``      -> :class:`ArmctlFakeCameraConfig` -> :class:`~armctl.lerobot.ArmctlFakeCamera`

LeRobot's factories (``make_robot_from_config`` etc.) fall back to
``make_device_from_device_class`` for unknown types: it strips ``Config`` from the class name
and looks for that class in this module, then in its parent package ``armctl.lerobot``, which
re-exports the device classes. So the class names here must stay ``<Device>Config``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from lerobot.cameras.configs import CameraConfig
from lerobot.robots.config import RobotConfig
from lerobot.teleoperators.config import TeleoperatorConfig

# Ports: a serial device path, "mock" (a private in-process MockBoard) or "mock:<name>" (a
# named MockBoard shared through armctl.lerobot.boards.mock_board(<name>), so a test or demo
# can backdrive a mock leader that a LeRobot script opened by config).


@RobotConfig.register_subclass("armctl_follower")
@dataclass(kw_only=True)
class ArmctlFollowerConfig(RobotConfig):
    port: str = "mock"
    baudrate: int = 1_000_000
    # armctl calibration YAML (offsets/directions) pushed to the board on connect
    calibration_file: str | None = None
    # SET_TARGETS move duration. None -> two periods of control_fps, so consecutive commands
    # blend (the board replans from its current (p, v, a) on every command).
    move_ms: int | None = None
    control_fps: int = 30
    # move duration used when torque is enabled at the measured pose on connect
    enable_move_ms: int = 500
    # run every action through armctl.safety.SafetyShim (limits, velocity, workspace,
    # self/base/table collision along the board's planned trajectory)
    safety: bool = True
    vel_scale: float = 1.0
    collision_margin_m: float = 0.005
    # optional extra clip (rad) of |target - measured| per joint, like SO-100's option
    max_relative_target: float | None = None
    # a rejected action (no safe partial step) raises instead of holding the last target
    raise_on_reject: bool = False
    disable_torque_on_disconnect: bool = True
    # initial joint angles (rad) of a mock board; None -> armctl.lerobot.boards.MOCK_HOME
    mock_initial_q: list[float] | None = None
    cameras: dict[str, CameraConfig] = field(default_factory=dict)


@TeleoperatorConfig.register_subclass("armctl_leader")
@dataclass(kw_only=True)
class ArmctlLeaderConfig(TeleoperatorConfig):
    port: str = "mock"
    baudrate: int = 1_000_000
    # armctl calibration YAML of the *leader*; its leader_to_follower section (if any) maps
    # leader joint angles to follower targets
    calibration_file: str | None = None
    # a leader STATE older than this (s) is stale: get_action waits up to this long for a fresh
    # one, then raises
    max_state_age_s: float = 0.1
    mock_initial_q: list[float] | None = None


@CameraConfig.register_subclass("armctl_fake")
@dataclass(kw_only=True)
class ArmctlFakeCameraConfig(CameraConfig):
    """Synthetic RGB frames (moving gradient + frame counter); for dry runs and tests."""

    fps: int | None = 30
    width: int | None = 96
    height: int | None = 96
    seed: int = 0
