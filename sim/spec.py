"""Load spec/kinematics.yaml (the single source of truth for geometry, limits and masses).

Nothing in sim/ hard-codes a link length, limit or mass: everything goes through `load_spec()`.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
SPEC_PATH = REPO_ROOT / "spec" / "kinematics.yaml"

ARM_JOINTS = ["base_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll"]
# Order of the 6-D state/action vector everywhere in sim/ and policies/.
ACTION_NAMES = ARM_JOINTS + ["gripper"]


@dataclass(frozen=True)
class Joint:
    name: str
    axis: np.ndarray
    origin: np.ndarray
    limits: tuple[float, float]
    vmax: float


@dataclass(frozen=True)
class Spec:
    joints: list[Joint]
    tcp: np.ndarray
    gripper_limits: tuple[float, float]
    gripper_opening_at_max: float
    gripper_vmax: float
    stall_torque: float
    max_cont_current: float
    stall_current: float
    masses: dict[str, float]
    payload: float
    raw: dict

    def joint(self, name: str) -> Joint:
        return next(j for j in self.joints if j.name == name)

    @property
    def lower(self) -> np.ndarray:
        return np.array([j.limits[0] for j in self.joints] + [self.gripper_limits[0]])

    @property
    def upper(self) -> np.ndarray:
        return np.array([j.limits[1] for j in self.joints] + [self.gripper_limits[1]])

    @property
    def vmax(self) -> np.ndarray:
        return np.array([j.vmax for j in self.joints] + [self.gripper_vmax])

    @property
    def finger_travel_per_rad(self) -> float:
        """Metres of travel *per finger* per radian of gripper servo (linear approx from spec)."""
        return 0.5 * self.gripper_opening_at_max / self.gripper_limits[1]


@lru_cache(maxsize=4)
def load_spec(path: str | Path = SPEC_PATH) -> Spec:
    raw = yaml.safe_load(Path(path).read_text())
    joints = [
        Joint(
            name=j["name"],
            axis=np.asarray(j["axis"], dtype=float),
            origin=np.asarray(j["origin"], dtype=float),
            limits=tuple(float(x) for x in j["limits"]),
            vmax=float(j["vmax"]),
        )
        for j in raw["joints"]
    ]
    names = [j.name for j in joints]
    if names != ARM_JOINTS:
        raise ValueError(f"spec joint order {names} != expected {ARM_JOINTS}")
    g = raw["gripper"]
    s = raw["servo"]
    return Spec(
        joints=joints,
        tcp=np.asarray(raw["tool"]["tcp"], dtype=float),
        gripper_limits=tuple(float(x) for x in g["limits"]),
        gripper_opening_at_max=float(g["opening_m_at_max"]),
        gripper_vmax=float(g["vmax"]),
        stall_torque=float(s["stall_torque_nm"]),
        max_cont_current=float(s["max_continuous_current_a"]),
        stall_current=float(s["stall_current_a"]),
        masses={k: float(v) for k, v in raw["masses_kg"].items()},
        payload=float(raw["payload_kg"]),
        raw=raw,
    )
