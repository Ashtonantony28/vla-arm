"""LeRobot ``Teleoperator`` for the leader arm: a second armctl board with torque off."""

from __future__ import annotations

import logging
from functools import cached_property
from typing import Any

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..calibration import Calibration, LeaderMap
from .boards import open_client
from .config import ArmctlLeaderConfig
from .follower import JOINT_KEYS

log = logging.getLogger(__name__)


class ArmctlLeader(Teleoperator):
    """``get_action`` returns the leader's latest STATE joint angles mapped through the leader
    calibration's ``leader_to_follower`` section (identity by default), as follower
    ``<joint>.pos`` targets in rad. The leader stays limp (torque is switched off on connect);
    there is no force feedback, so ``feedback_features`` is empty."""

    config_class = ArmctlLeaderConfig
    name = "armctl_leader"

    def __init__(self, config: ArmctlLeaderConfig):
        super().__init__(config)
        self.config = config
        self.client = None
        self.mapping = LeaderMap()

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {k: float for k in JOINT_KEYS}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.client is not None

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.client = open_client(self.config.port, self.config.baudrate, initial_q=self.config.mock_initial_q)
        try:
            self.client.wait_for_state(timeout=1.0)
            if self.config.calibration_file:
                cal = Calibration.load(self.config.calibration_file)
                cal.push(self.client)
                self.mapping = cal.leader_to_follower or LeaderMap()
            self.configure()
        except Exception:
            self.client.close()
            self.client = None
            raise
        log.info("%s connected", self)

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        log.info("armctl boards are calibrated with armctl calibration YAML (calibration_file)")

    def configure(self) -> None:
        self.client.torque_off()  # the operator moves it by hand

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        cli = self.client
        st = cli.state
        if st is None or cli.state_age() > self.config.max_state_age_s:
            st = cli.wait_for_state(timeout=self.config.max_state_age_s)  # raises ArmTimeout
        q = self.mapping(st.positions_rad)
        return {k: float(v) for k, v in zip(JOINT_KEYS, q)}

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass  # no force feedback on the leader

    @check_if_not_connected
    def disconnect(self) -> None:
        try:
            self.client.close()
        finally:
            self.client = None
        log.info("%s disconnected", self)
