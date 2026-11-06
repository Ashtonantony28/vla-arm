"""Forward / inverse kinematics of the arm, derived from spec/kinematics.yaml (sim-local copy).

armctl has its own IK; this one exists so sim/ has no dependency on armctl.

Conventions (from the spec): pitch joints rotate about +y, positive pitch = DOWN; yaw about +z,
positive = +x toward +y; roll about +x. All lengths along +x at zero pose.
"""
from __future__ import annotations

import math

import numpy as np

from sim.spec import Spec, load_spec


def _rot(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    C = 1 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def fk(q: np.ndarray, spec: Spec | None = None) -> tuple[np.ndarray, np.ndarray]:
    """TCP position (3,) and orientation (3x3, columns = gripper x/y/z axes) for 5 arm joints."""
    spec = spec or load_spec()
    R = np.eye(3)
    p = np.zeros(3)
    for jt, qi in zip(spec.joints, q[:5]):
        p = p + R @ jt.origin
        R = R @ _rot(jt.axis, float(qi))
    return p + R @ spec.tcp, R


class IKError(ValueError):
    pass


class TopDownIK:
    """Closed-form IK for this 5-DOF layout given TCP position, approach pitch and jaw angle.

    approach_pitch: total pitch (shoulder+elbow+wrist); pi/2 = gripper pointing straight down.
    jaw_angle:      world yaw (rad) of the line along which the jaws close (only meaningful for
                    near-vertical approach). Fingers are symmetric so jaw_angle is mod pi.
    """

    def __init__(self, spec: Spec | None = None):
        self.spec = spec = spec or load_spec()
        J = {j.name: j for j in spec.joints}
        # structural assumptions the closed form relies on
        for name in ("elbow_pitch", "wrist_pitch", "wrist_roll"):
            if abs(J[name].origin[1]) > 1e-9 or abs(J[name].origin[2]) > 1e-9:
                raise IKError(f"closed-form IK assumes {name} offset purely along x")
        if abs(spec.tcp[1]) > 1e-9 or abs(spec.tcp[2]) > 1e-9:
            raise IKError("closed-form IK assumes TCP along gripper x")
        self.h0 = J["base_yaw"].origin[2] + J["shoulder_pitch"].origin[2]
        self.r0 = J["base_yaw"].origin[0] + J["shoulder_pitch"].origin[0]
        self.L1 = J["elbow_pitch"].origin[0]
        self.L2 = J["wrist_pitch"].origin[0]
        self.Lw = J["wrist_roll"].origin[0] + spec.tcp[0]
        self.lo = np.array([j.limits[0] for j in spec.joints])
        self.hi = np.array([j.limits[1] for j in spec.joints])

    def solve(self, pos, approach_pitch: float = math.pi / 2, jaw_angle: float | None = None,
              check_limits: bool = True) -> np.ndarray:
        x, y, z = (float(v) for v in pos)
        yaw = math.atan2(y, x)
        r = math.hypot(x, y)
        a = approach_pitch
        # wrist-pitch axis in the (radial u, height v) plane; link dir at pitch b is (cos b, -sin b)
        wu = r - self.Lw * math.cos(a) - self.r0
        wv = z + self.Lw * math.sin(a) - self.h0
        # standard 2-link in (u, -v) coordinates
        pu, pw = wu, -wv
        D = (pu * pu + pw * pw - self.L1 ** 2 - self.L2 ** 2) / (2 * self.L1 * self.L2)
        if abs(D) > 1.0:
            raise IKError(f"target {pos} out of reach (D={D:.3f})")
        q3 = math.acos(D)  # elbow-up: positive elbow pitch, shoulder raised
        q2 = math.atan2(pw, pu) - math.atan2(self.L2 * math.sin(q3), self.L1 + self.L2 * math.cos(q3))
        q4 = a - q2 - q3
        if jaw_angle is None:
            roll = 0.0
        else:
            # pointing down, roll rotates the jaw line by -roll about world z;
            # jaw line at roll=0 is yaw + pi/2.
            roll = yaw + math.pi / 2 - jaw_angle
            roll = (roll + math.pi / 2) % math.pi - math.pi / 2  # jaws symmetric: mod pi
        q = np.array([yaw, q2, q3, q4, roll])
        if check_limits and (np.any(q < self.lo - 1e-6) or np.any(q > self.hi + 1e-6)):
            raise IKError(f"IK solution {np.round(q, 3)} violates joint limits")
        return q

    def roll_for_jaw(self, yaw: float, jaw_angle: float) -> float:
        """Unwrapped roll (no mod) for continuous interpolation."""
        return yaw + math.pi / 2 - jaw_angle


def gripper_rad_from_opening(opening_m: float, spec: Spec | None = None) -> float:
    spec = spec or load_spec()
    return opening_m / spec.gripper_opening_at_max * spec.gripper_limits[1]


def gripper_opening_from_rad(rad: float, spec: Spec | None = None) -> float:
    spec = spec or load_spec()
    return rad / spec.gripper_limits[1] * spec.gripper_opening_at_max
