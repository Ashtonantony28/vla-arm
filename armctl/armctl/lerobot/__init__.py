"""LeRobot integration (matches the LeRobot 0.7.0 ``Robot`` / ``Teleoperator`` / ``Camera`` APIs).

Optional: needs ``pip install 'armctl[lerobot]'`` (or any environment with ``lerobot``
installed). Importing this package registers the draccus choices ``armctl_follower``,
``armctl_leader`` and ``armctl_fake`` so LeRobot's config parsing and factories find them;
``armctl-lerobot-record`` / ``armctl-lerobot-teleoperate`` are ``lerobot-record`` /
``lerobot-teleoperate`` with that import done first.

    from armctl.lerobot import ArmctlFollower, ArmctlFollowerConfig
    robot = ArmctlFollower(ArmctlFollowerConfig(port="/dev/ttyACM0"))
"""

try:
    import lerobot as _lerobot  # noqa: F401
except ImportError as e:  # pragma: no cover - exercised only without lerobot
    raise ImportError(
        "armctl.lerobot needs LeRobot >= 0.7: pip install 'armctl[lerobot]'"
    ) from e

from .boards import MOCK_HOME, forget_mock_board, mock_board, open_client
from .camera import ArmctlFakeCamera
from .config import ArmctlFakeCameraConfig, ArmctlFollowerConfig, ArmctlLeaderConfig
from .follower import JOINT_KEYS, ArmctlFollower, FollowerFault
from .leader import ArmctlLeader

__all__ = [
    "JOINT_KEYS",
    "MOCK_HOME",
    "ArmctlFakeCamera",
    "ArmctlFakeCameraConfig",
    "ArmctlFollower",
    "ArmctlFollowerConfig",
    "ArmctlLeader",
    "ArmctlLeaderConfig",
    "FollowerFault",
    "forget_mock_board",
    "mock_board",
    "open_client",
]
