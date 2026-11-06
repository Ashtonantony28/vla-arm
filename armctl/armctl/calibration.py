"""Joint calibration and leader -> follower mapping.

Board-side joint mapping (INTERP, see README; the spec names the CONFIG keys but not the
formula)::

    joint_rad = direction * (ticks - center_tick - offset_ticks) * 2*pi / ticks_per_rev

File format (YAML)::

    version: 1
    arm: follower                # free text
    joints:                      # one entry per servo, protocol joint order
      base_yaw:      {offset_ticks: 12, direction: 1}
      ...
      gripper:       {offset_ticks: -40, direction: -1}
    leader_to_follower:          # optional; used by teleop on the *leader's* calibration
      joints:                    # q_follower = scale * q_leader + offset_rad
        base_yaw:    {scale: 1.0, offset_rad: 0.0}
        ...
      gripper:                   # leader handle range mapped linearly onto follower range
        leader_closed_rad: 0.0
        leader_open_rad: 0.9
        follower_closed_rad: 0.0
        follower_open_rad: 1.6
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from . import protocol as P
from .kinematics import ArmModel, default_model

NAMES = P.JOINT_NAMES
N = P.N_JOINTS


@dataclass
class JointCal:
    offset_ticks: int = 0
    direction: int = 1


@dataclass
class LinearMap:
    scale: float = 1.0
    offset_rad: float = 0.0


@dataclass
class GripperMap:
    leader_closed_rad: float = 0.0
    leader_open_rad: float = 1.6
    follower_closed_rad: float = 0.0
    follower_open_rad: float = 1.6

    def __call__(self, q_leader: float) -> float:
        span = self.leader_open_rad - self.leader_closed_rad
        frac = 0.0 if abs(span) < 1e-9 else (q_leader - self.leader_closed_rad) / span
        frac = min(max(frac, 0.0), 1.0)
        return self.follower_closed_rad + frac * (self.follower_open_rad - self.follower_closed_rad)


@dataclass
class LeaderMap:
    joints: list[LinearMap] = field(default_factory=lambda: [LinearMap() for _ in range(N - 1)])
    gripper: GripperMap = field(default_factory=GripperMap)

    def __call__(self, q_leader) -> np.ndarray:
        q = np.asarray(q_leader, dtype=float)
        out = np.empty(N)
        for i, m in enumerate(self.joints):
            out[i] = m.scale * q[i] + m.offset_rad
        out[N - 1] = self.gripper(q[N - 1])
        return out


@dataclass
class Calibration:
    joints: list[JointCal] = field(default_factory=lambda: [JointCal() for _ in range(N)])
    leader_to_follower: LeaderMap | None = None
    arm: str = ""

    # ---------------------------------------------------------------- YAML
    def to_dict(self) -> dict:
        d: dict = {"version": 1, "arm": self.arm, "joints": {}}
        for name, j in zip(NAMES, self.joints):
            d["joints"][name] = {"offset_ticks": int(j.offset_ticks), "direction": int(j.direction)}
        if self.leader_to_follower is not None:
            lm = self.leader_to_follower
            d["leader_to_follower"] = {
                "joints": {
                    name: {"scale": float(m.scale), "offset_rad": float(m.offset_rad)}
                    for name, m in zip(NAMES[:-1], lm.joints)
                },
                "gripper": {k: float(v) for k, v in vars(lm.gripper).items()},
            }
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Calibration":
        if d.get("version", 1) != 1:
            raise ValueError(f"unsupported calibration version {d.get('version')}")
        jd = d.get("joints", {})
        missing = [n for n in NAMES if n not in jd]
        if missing:
            raise ValueError(f"calibration missing joints: {missing}")
        joints = []
        for n in NAMES:
            j = JointCal(int(jd[n]["offset_ticks"]), int(jd[n].get("direction", 1)))
            if j.direction not in (1, -1):
                raise ValueError(f"{n}: direction must be +1 or -1")
            joints.append(j)
        lm = None
        if "leader_to_follower" in d and d["leader_to_follower"]:
            l2f = d["leader_to_follower"]
            lj = l2f.get("joints", {})
            lm = LeaderMap(
                joints=[LinearMap(**lj.get(n, {})) for n in NAMES[:-1]],
                gripper=GripperMap(**l2f.get("gripper", {})),
            )
        return cls(joints=joints, leader_to_follower=lm, arm=d.get("arm", ""))

    def save(self, path: str | Path) -> None:
        Path(path).write_text(yaml.safe_dump(self.to_dict(), sort_keys=False))

    @classmethod
    def load(cls, path: str | Path) -> "Calibration":
        return cls.from_dict(yaml.safe_load(Path(path).read_text()))

    # ---------------------------------------------------------------- board
    def push(self, client) -> None:
        """Write offset/direction for every joint to the board with CONFIG (ACKed).

        Do this with torque off: changing the mapping under torque would jump the arm."""
        st = client.state
        if st is not None and st.mode in (P.Mode.ACTIVE, P.Mode.HOLD):
            raise RuntimeError("refusing to change calibration while torque is on")
        for i, j in enumerate(self.joints):
            client.configure(i, P.ConfigKey.DIRECTION, j.direction)
            client.configure(i, P.ConfigKey.OFFSET_TICKS, j.offset_ticks)


# -------------------------------------------------------------------- procedure helpers
def rad_per_tick(model: ArmModel | None = None) -> float:
    return 2 * math.pi / (model or default_model()).ticks_per_rev


def raw_ticks_from_identity(pos_rad, model: ArmModel | None = None) -> np.ndarray:
    """Servo ticks relative to centre, from positions read with offset 0 / direction +1."""
    return np.asarray(pos_rad, dtype=float) / rad_per_tick(model)


def compute_offsets(raw_pos_rad, known_pose_rad, directions, model: ArmModel | None = None) -> list[int]:
    """Offsets that make the board report ``known_pose_rad`` at the current physical pose.

    ``raw_pos_rad``: positions read while the board had offset 0 and direction +1.
    Solves ``known = dir * (rel_ticks - offset) * k`` for ``offset``.
    """
    rel = raw_ticks_from_identity(raw_pos_rad, model)
    k = rad_per_tick(model)
    d = np.asarray(directions, dtype=float)
    return [int(round(x)) for x in rel - np.asarray(known_pose_rad, dtype=float) / (d * k)]


def compute_directions(raw_a, raw_b, pose_a, pose_b, min_delta: float = 0.1) -> list[int]:
    """Directions from two known poses: sign of (raw change) / (true change) per joint."""
    out = []
    for i, (ra, rb, pa, pb) in enumerate(zip(raw_a, raw_b, pose_a, pose_b)):
        if abs(pb - pa) < min_delta:
            raise ValueError(f"joint {i}: poses differ by < {min_delta} rad, cannot tell direction")
        if abs(rb - ra) < 0.5 * min_delta:
            raise ValueError(f"joint {i}: servo barely moved between poses")
        out.append(1 if (rb - ra) * (pb - pa) > 0 else -1)
    return out


def read_identity_positions(client, settle_states: int = 3, timeout: float = 1.0) -> np.ndarray:
    """Reset the board mapping to identity (offset 0, dir +1) and read averaged positions."""
    Calibration().push(client)
    samples = []
    for _ in range(settle_states):
        samples.append(client.wait_for_state(timeout=timeout).positions_rad)
    return np.mean(samples, axis=0)


def calibrate_at_known_pose(
    client,
    known_pose_rad,
    directions=None,
    *,
    verify_tol: float = 0.01,
    model: ArmModel | None = None,
) -> Calibration:
    """Full procedure, arm limp and physically placed at ``known_pose_rad``:

    1. push identity mapping, read raw positions;
    2. compute offsets for the given ``directions`` (default all +1);
    3. push the result and verify the board now reports the known pose.
    """
    directions = list(directions) if directions is not None else [1] * N
    raw = read_identity_positions(client)
    offs = compute_offsets(raw, known_pose_rad, directions, model)
    cal = Calibration([JointCal(o, d) for o, d in zip(offs, directions)])
    cal.push(client)
    got = np.array(client.wait_for_state(timeout=1.0).positions_rad)
    err = np.abs(got - np.asarray(known_pose_rad))
    if np.any(err > verify_tol):
        raise RuntimeError(f"calibration verify failed: errors {np.round(err, 4)} rad")
    return cal
