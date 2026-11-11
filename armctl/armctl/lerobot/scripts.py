"""``armctl-lerobot-record`` / ``armctl-lerobot-teleoperate``: LeRobot's own CLIs with the armctl
devices registered first. LeRobot only auto-imports plugins whose *distribution* is named
``lerobot_robot_*`` / ``lerobot_teleoperator_*`` / ``lerobot_camera_*``, which armctl is not.

    armctl-lerobot-record \\
        --robot.type=armctl_follower --robot.port=/dev/ttyACM0 \\
        --robot.cameras='{wrist: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}' \\
        --teleop.type=armctl_leader --teleop.port=/dev/ttyACM1 \\
        --dataset.repo_id=me/vla_arm_pick --dataset.single_task="pick up the red block" \\
        --dataset.num_episodes=10 --dataset.push_to_hub=false
"""

from __future__ import annotations


def record_main() -> None:
    import armctl.lerobot  # noqa: F401  (registers the config choices)
    from lerobot.scripts import lerobot_record

    lerobot_record.main()


def teleoperate_main() -> None:
    import armctl.lerobot  # noqa: F401
    from lerobot.scripts import lerobot_teleoperate

    lerobot_teleoperate.main()
