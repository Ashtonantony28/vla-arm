"""LeRobot ``Robot`` for the VLA arm (follower): armctl protocol client + safety shim + cameras."""

from __future__ import annotations

import logging
import time
from functools import cached_property

import numpy as np
from lerobot.cameras import make_cameras_from_configs
from lerobot.robots.robot import Robot
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from .. import protocol as P
from ..calibration import Calibration
from ..safety import SafetyConfig, SafetyShim
from .boards import open_client
from .config import ArmctlFollowerConfig

log = logging.getLogger(__name__)

JOINT_KEYS = tuple(f"{n}.pos" for n in P.JOINT_NAMES)  # radians; gripper in servo rad


class FollowerFault(RuntimeError):
    """The board is in FAULT (or has latched faults): commands are refused until cleared."""


class ArmctlFollower(Robot):
    """Observation: ``<joint>.pos`` (rad, protocol order: base_yaw, shoulder_pitch, elbow_pitch,
    wrist_pitch, wrist_roll, gripper) plus one ``(h, w, 3)`` image per camera. Action: the same
    six ``<joint>.pos`` absolute targets. Same names/units/order as the sim datasets
    (``sim/record.py``), with LeRobot's ``.pos`` suffix.

    ``send_action`` passes each target through :class:`~armctl.safety.SafetyShim` (unless
    ``config.safety`` is False) and sends it as a fire-and-forget SET_TARGETS with
    ``move_ms`` duration; it returns what was actually sent. A rejected target (no safe partial
    step exists) keeps the previous target and, by default, is only logged.
    """

    config_class = ArmctlFollowerConfig
    name = "armctl_follower"
    supports_position_hold = True

    def __init__(self, config: ArmctlFollowerConfig):
        super().__init__(config)
        self.config = config
        self.client = None
        self.cameras = make_cameras_from_configs(config.cameras)
        self.move_ms = int(config.move_ms or max(1, round(2000.0 / config.control_fps)))
        self.shim = (
            SafetyShim(
                config=SafetyConfig(vel_scale=config.vel_scale, collision_margin=config.collision_margin_m)
            )
            if config.safety
            else None
        )
        self._last_sent: np.ndarray | None = None
        self.safety_modified = 0
        self.safety_rejected = 0
        self.last_reasons: list[str] = []

    # ------------------------------------------------------------------ features
    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {k: (c.height, c.width, 3) for k, c in self.cameras.items()}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**{k: float for k in JOINT_KEYS}, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {k: float for k in JOINT_KEYS}

    # ------------------------------------------------------------------ lifecycle
    @property
    def is_connected(self) -> bool:
        return self.client is not None and all(c.is_connected for c in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        self.client = open_client(
            self.config.port, self.config.baudrate, initial_q=self.config.mock_initial_q
        )
        try:
            self.client.wait_for_state(timeout=1.0)
            if self.config.calibration_file:
                Calibration.load(self.config.calibration_file).push(self.client)
            if not self.is_calibrated and calibrate:
                self.calibrate()
            for cam in self.cameras.values():
                cam.connect()
            self.configure()
        except Exception:
            self._close_client()
            raise
        log.info("%s connected", self)

    @property
    def is_calibrated(self) -> bool:
        # Board-side calibration is the armctl YAML pushed on connect (or the board's stored
        # CONFIG values); there is no LeRobot-side motor calibration to perform.
        return True

    def calibrate(self) -> None:
        log.info("armctl boards are calibrated with armctl calibration YAML (calibration_file)")

    def configure(self) -> None:
        """Torque on, holding the measured pose; seeds the safety shim there."""
        st = self.client.wait_for_state(timeout=1.0)
        if st.mode == P.Mode.FAULT or st.faults & P.LATCHED_FAULTS:
            raise FollowerFault(f"board has latched faults {P.Fault(st.fault_flags)!r}; clear them first")
        q = np.array(st.positions_rad)
        if self.shim is not None:
            self.shim.reset(q)
        self.client.set_targets(q, self.config.enable_move_ms, apply_safety=False)
        self._last_sent = q

    def _close_client(self) -> None:
        if self.client is not None:
            try:
                self.client.close()
            finally:
                self.client = None

    @check_if_not_connected
    def disconnect(self) -> None:
        try:
            if self.config.disable_torque_on_disconnect:
                self.client.torque_off()
        except Exception as e:
            log.warning("torque off on disconnect failed: %s", e)
        finally:
            self._close_client()
            for cam in self.cameras.values():
                if cam.is_connected:
                    cam.disconnect()
        log.info("%s disconnected", self)

    # ------------------------------------------------------------------ I/O
    @check_if_not_connected
    def get_observation(self) -> dict:
        st = self.client.state
        if st is None or self.client.state_age() > 0.25:
            st = self.client.wait_for_state(timeout=0.25)
        obs: dict = {k: float(v) for k, v in zip(JOINT_KEYS, st.positions_rad)}
        for key, cam in self.cameras.items():
            obs[key] = cam.read_latest()
        return obs

    def _to_dict(self, q) -> dict[str, float]:
        return {k: float(v) for k, v in zip(JOINT_KEYS, q)}

    @check_if_not_connected
    def send_action(self, action: dict) -> dict[str, float]:
        st = self.client.state
        if st is not None and st.mode == P.Mode.FAULT:
            raise FollowerFault(f"follower in FAULT: {P.Fault(st.fault_flags)!r}")
        q = self._last_sent.copy()
        for i, k in enumerate(JOINT_KEYS):
            if k in action:
                q[i] = float(action[k])
        if self.config.max_relative_target is not None and st is not None:
            meas = np.array(st.positions_rad)
            m = float(self.config.max_relative_target)
            q = np.clip(q, meas - m, meas + m)
        if self.shim is not None:
            rep = self.shim.filter(q, self.move_ms / 1000.0, now=time.monotonic())
            if rep.rejected:
                self.safety_rejected += 1
                self.last_reasons = rep.reasons
                if self.config.raise_on_reject:
                    raise RuntimeError(f"safety rejected action: {rep.reasons}")
                log.warning("safety rejected action (holding last target): %s", rep.reasons)
                return self._to_dict(self._last_sent)
            if rep.modified:
                self.safety_modified += 1
                self.last_reasons = rep.reasons
                log.debug("safety: %s", rep)
            q = rep.q_out
        self.client.set_targets(q, self.move_ms, wait=False, apply_safety=False)
        self._last_sent = np.asarray(q, dtype=float).copy()
        return self._to_dict(q)
