"""Optional proprioceptive feature: append the end-effector (TCP) position to the joint state.

The TCP position is a deterministic function of the joints (forward kinematics from
spec/kinematics.yaml), so it adds no information -- but it gives the policy the Cartesian
quantity the task is defined in (block and bowl positions are Cartesian), which a small network
would otherwise have to learn from 5 joint angles. Works identically on the real robot (same FK).

state_mode:
  "joints"     6-d  (joint positions; the dataset's observation.state)
  "joints_ee"  9-d  joints + TCP xyz in metres (base frame)
"""
from __future__ import annotations

import numpy as np

from sim.kinematics import fk
from sim.spec import load_spec

STATE_MODES = ("joints", "joints_ee")
_SPEC = None


def state_dim(mode: str) -> int:
    return {"joints": 6, "joints_ee": 9}[mode]


def transform_state(q: np.ndarray, mode: str) -> np.ndarray:
    """q: (..., 6) joint states -> (..., state_dim(mode)) float32."""
    global _SPEC
    if mode not in STATE_MODES:
        raise ValueError(f"state_mode must be one of {STATE_MODES}")
    q = np.asarray(q, dtype=np.float64)
    if mode == "joints":
        return q.astype(np.float32)
    _SPEC = _SPEC or load_spec()
    flat = q.reshape(-1, q.shape[-1])
    ee = np.stack([fk(row, _SPEC)[0] for row in flat])
    return np.concatenate([flat, ee], axis=1).reshape(*q.shape[:-1], 9).astype(np.float32)
