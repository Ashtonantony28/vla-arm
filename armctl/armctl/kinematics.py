"""Forward/inverse kinematics for the 5-DOF arm + gripper.

Source of truth: ``spec/kinematics.yaml``. Conventions (from its header):

* base frame at the centre of the base plate, +z up, +x forward;
* each joint: translate by ``origin`` (in the parent link frame) then rotate about ``axis``;
* positive pitch (rotation about +y) moves +x toward -z, i.e. pitch DOWN is positive;
* zero pose: arm stretched along +x at shoulder height, jaws along +x.

All angles here are radians, lengths metres. Arm joint vectors have 5 entries
(base_yaw, shoulder_pitch, elbow_pitch, wrist_pitch, wrist_roll); the gripper is separate.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

N_ARM = 5


# --------------------------------------------------------------------------- loading
def find_kinematics_yaml() -> Path:
    """Locate spec/kinematics.yaml: $ARMCTL_KINEMATICS, else walk up from this file."""
    env = os.environ.get("ARMCTL_KINEMATICS")
    if env:
        return Path(env)
    here = Path(__file__).resolve()
    for parent in here.parents:
        cand = parent / "spec" / "kinematics.yaml"
        if cand.is_file():
            return cand
    raise FileNotFoundError("spec/kinematics.yaml not found; set ARMCTL_KINEMATICS")


@dataclass
class JointSpec:
    name: str
    axis: np.ndarray
    origin: np.ndarray
    limits: tuple[float, float]
    vmax: float


@dataclass
class GripperSpec:
    name: str
    limits: tuple[float, float]
    opening_m_at_max: float
    vmax: float


@dataclass
class ArmModel:
    joints: list[JointSpec]
    tcp: np.ndarray
    gripper: GripperSpec
    ticks_per_rev: int = 4096
    center_tick: int = 2048
    raw: dict = field(default_factory=dict, repr=False)

    # ---- derived geometry used by the analytic IK -----------------------------------
    @property
    def shoulder_height(self) -> float:
        return float(self.joints[0].origin[2] + self.joints[1].origin[2])

    @property
    def upper_arm(self) -> float:
        return float(self.joints[2].origin[0])

    @property
    def forearm(self) -> float:
        return float(self.joints[3].origin[0])

    @property
    def wrist_to_tcp(self) -> float:
        return float(self.joints[4].origin[0] + self.tcp[0])

    @property
    def lower(self) -> np.ndarray:
        return np.array([j.limits[0] for j in self.joints])

    @property
    def upper(self) -> np.ndarray:
        return np.array([j.limits[1] for j in self.joints])

    @property
    def vmax(self) -> np.ndarray:
        """vmax for all 6 servos (5 arm joints + gripper), rad/s."""
        return np.array([j.vmax for j in self.joints] + [self.gripper.vmax])

    @property
    def lower6(self) -> np.ndarray:
        return np.append(self.lower, self.gripper.limits[0])

    @property
    def upper6(self) -> np.ndarray:
        return np.append(self.upper, self.gripper.limits[1])

    def within_limits(self, q: np.ndarray, tol: float = 1e-9) -> bool:
        q = np.asarray(q)[:N_ARM]
        return bool(np.all(q >= self.lower - tol) and np.all(q <= self.upper + tol))


def load_model(path: str | Path | None = None) -> ArmModel:
    path = Path(path) if path else find_kinematics_yaml()
    data = yaml.safe_load(path.read_text())
    joints = [
        JointSpec(
            name=j["name"],
            axis=np.asarray(j["axis"], dtype=float),
            origin=np.asarray(j["origin"], dtype=float),
            limits=(float(j["limits"][0]), float(j["limits"][1])),
            vmax=float(j["vmax"]),
        )
        for j in data["joints"]
    ]
    if len(joints) != N_ARM:
        raise ValueError(f"expected {N_ARM} arm joints, got {len(joints)}")
    # The analytic IK assumes this exact structure; fail loudly if the YAML changes shape.
    expect_axes = [(0, 0, 1), (0, 1, 0), (0, 1, 0), (0, 1, 0), (1, 0, 0)]
    for j, ax in zip(joints, expect_axes):
        if not np.allclose(j.axis, ax):
            raise ValueError(f"joint {j.name}: axis {j.axis} != expected {ax}")
    for j in joints[2:]:
        if not np.allclose(j.origin[1:], 0):
            raise ValueError(f"joint {j.name}: analytic IK assumes origin along x")
    g = data["gripper"]
    gripper = GripperSpec(
        name=g.get("name", "gripper"),
        limits=(float(g["limits"][0]), float(g["limits"][1])),
        opening_m_at_max=float(g["opening_m_at_max"]),
        vmax=float(g["vmax"]),
    )
    servo = data.get("servo", {})
    return ArmModel(
        joints=joints,
        tcp=np.asarray(data["tool"]["tcp"], dtype=float),
        gripper=gripper,
        ticks_per_rev=int(servo.get("ticks_per_rev", 4096)),
        center_tick=int(servo.get("center_tick", 2048)),
        raw=data,
    )


_DEFAULT: ArmModel | None = None


def default_model() -> ArmModel:
    global _DEFAULT
    if _DEFAULT is None:
        _DEFAULT = load_model()
    return _DEFAULT


# --------------------------------------------------------------------------- rotations
def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def axis_angle(axis: np.ndarray, a: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    c, s = math.cos(a), math.sin(a)
    C = 1 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def rot_log(R: np.ndarray) -> np.ndarray:
    """SO(3) log map -> rotation vector."""
    cos = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    ang = math.acos(cos)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if ang < 1e-9:
        return 0.5 * w
    if math.pi - ang < 1e-6:
        # near pi: axis from the diagonal
        A = (R + np.eye(3)) / 2
        k = int(np.argmax(np.diag(A)))
        axis = A[:, k] / math.sqrt(max(A[k, k], 1e-12))
        if np.dot(axis, w) < 0:
            axis = -axis
        return ang * axis
    return ang / (2 * math.sin(ang)) * w


def wrap(a: float) -> float:
    """Wrap to (-pi, pi]."""
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a <= 0:
        a += 2 * math.pi
    return a - math.pi


def make_pose(p, R) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def target_pose(p, pitch: float, roll: float = 0.0) -> np.ndarray:
    """Pose with TCP at ``p``, tool x-axis pitched ``pitch`` below horizontal (positive =
    pointing down) and heading radially away from the base axis, and wrist roll ``roll``.

    This is ``Rz(atan2(py, px)) @ Ry(pitch) @ Rx(roll)``.
    """
    p = np.asarray(p, dtype=float)
    yaw = math.atan2(p[1], p[0]) if math.hypot(p[0], p[1]) > 1e-12 else 0.0
    return make_pose(p, rot_z(yaw) @ rot_y(pitch) @ rot_x(roll))


# --------------------------------------------------------------------------- FK
def _chain(model: ArmModel, q) -> list[np.ndarray]:
    """Return the world transform of each joint frame (after its rotation) and the TCP."""
    q = np.asarray(q, dtype=float)
    T = np.eye(4)
    frames = []
    for j, qi in zip(model.joints, q[:N_ARM]):
        T = T @ make_pose(j.origin, np.eye(3))
        T = T @ make_pose(np.zeros(3), axis_angle(j.axis, qi))
        frames.append(T)
    frames.append(T @ make_pose(model.tcp, np.eye(3)))
    return frames


def fk(q, model: ArmModel | None = None) -> np.ndarray:
    """TCP pose (4x4) for arm joints ``q`` (5 values; extra gripper entry ignored)."""
    return _chain(model or default_model(), q)[-1]


def joint_positions(q, model: ArmModel | None = None) -> dict[str, np.ndarray]:
    """World positions of key points: shoulder, elbow, wrist (pitch axis), roll, tcp."""
    fr = _chain(model or default_model(), q)
    return {
        "base": fr[0][:3, 3],
        "shoulder": fr[1][:3, 3],
        "elbow": fr[2][:3, 3],
        "wrist": fr[3][:3, 3],
        "roll": fr[4][:3, 3],
        "tcp": fr[5][:3, 3],
    }


def jacobian(q, model: ArmModel | None = None) -> np.ndarray:
    """Geometric 6x5 Jacobian of the TCP: rows [v; w] in the base frame."""
    model = model or default_model()
    fr = _chain(model, q)
    p_tcp = fr[-1][:3, 3]
    J = np.zeros((6, N_ARM))
    for i, j in enumerate(model.joints):
        z = fr[i][:3, :3] @ (j.axis / np.linalg.norm(j.axis))
        o = fr[i][:3, 3]
        J[:3, i] = np.cross(z, p_tcp - o)
        J[3:, i] = z
    return J


def pose_error(T_cur: np.ndarray, T_des: np.ndarray) -> np.ndarray:
    """6-vector [dp; dtheta] (world frame) taking T_cur to T_des."""
    dp = T_des[:3, 3] - T_cur[:3, 3]
    dw = rot_log(T_des[:3, :3] @ T_cur[:3, :3].T)
    return np.concatenate([dp, dw])


# --------------------------------------------------------------------------- gripper
def gripper_rad_to_opening(q: float, model: ArmModel | None = None) -> float:
    g = (model or default_model()).gripper
    q = min(max(q, g.limits[0]), g.limits[1])
    return (q - g.limits[0]) / (g.limits[1] - g.limits[0]) * g.opening_m_at_max


def gripper_opening_to_rad(opening_m: float, model: ArmModel | None = None) -> float:
    g = (model or default_model()).gripper
    frac = min(max(opening_m / g.opening_m_at_max, 0.0), 1.0)
    return g.limits[0] + frac * (g.limits[1] - g.limits[0])


# --------------------------------------------------------------------------- IK
@dataclass
class IKSolution:
    q: np.ndarray  # 5 arm joints, rad
    branch: str  # e.g. "front/elbow_up"
    pos_err: float  # metres
    rot_err: float  # radians


@dataclass
class IKResult:
    solutions: list[IKSolution]
    reason: str = ""  # why there are no (or fewer) solutions
    rejected: list[tuple[str, str]] = field(default_factory=list)  # (branch, why)

    @property
    def ok(self) -> bool:
        return bool(self.solutions)

    def best(self, q_ref=None) -> IKSolution | None:
        """Solution closest to ``q_ref`` (joint-space L2), else the first."""
        if not self.solutions:
            return None
        if q_ref is None:
            return self.solutions[0]
        q_ref = np.asarray(q_ref, dtype=float)[:N_ARM]
        return min(self.solutions, key=lambda s: float(np.linalg.norm(s.q - q_ref)))


class Unreachable(ValueError):
    pass


def _limit_violations(model: ArmModel, q: np.ndarray, tol: float) -> list[str]:
    out = []
    for j, qi in zip(model.joints, q):
        lo, hi = j.limits
        if qi < lo - tol or qi > hi + tol:
            out.append(f"{j.name}={qi:+.3f} outside [{lo:+.2f}, {hi:+.2f}]")
    return out


def solve_ik(
    T_des: np.ndarray,
    model: ArmModel | None = None,
    *,
    refine: bool = True,
    pos_tol: float = 1e-5,
    rot_tol: float = 1e-4,
    limit_tol: float = 1e-6,
) -> IKResult:
    """Analytic IK for a TCP pose, returning every valid branch.

    A 5-DOF arm cannot reach an arbitrary 6-DOF pose: the tool x-axis must lie in the
    vertical plane through the base axis that contains the TCP. The analytic solver
    projects the orientation onto that constraint (base yaw from the target position,
    tool pitch and roll from the requested rotation); the reported ``rot_err`` says how
    far the achieved orientation is from the request.

    Branches: base ``front`` (yaw = atan2(y, x)) and ``back`` (yaw + pi, reaching over the
    top), each with ``elbow_up`` / ``elbow_down``. Branches outside the joint limits are
    listed in ``rejected`` with the reason.
    """
    model = model or default_model()
    T_des = np.asarray(T_des, dtype=float)
    p = T_des[:3, 3]
    R = T_des[:3, :3]
    a = R[:, 0]  # tool approach (x) axis
    L1, L2, Lw, h = model.upper_arm, model.forearm, model.wrist_to_tcp, model.shoulder_height

    r_xy = math.hypot(p[0], p[1])
    if r_xy > 1e-9:
        yaw0 = math.atan2(p[1], p[0])
    elif math.hypot(a[0], a[1]) > 1e-9:
        yaw0 = math.atan2(a[1], a[0])  # TCP on the base axis: use the tool heading
    else:
        yaw0 = 0.0  # fully degenerate: any yaw works, pick 0

    sols: list[IKSolution] = []
    rejected: list[tuple[str, str]] = []
    geom_reasons: list[str] = []
    for base_name, yaw in (("front", wrap(yaw0)), ("back", wrap(yaw0 + math.pi))):
        Rb = rot_z(yaw)
        # orientation in the arm plane: R_plane = Ry(phi) Rx(roll)
        M = Rb.T @ R
        phi = math.atan2(-M[2, 0], M[0, 0])
        roll = math.atan2(-M[1, 2], M[1, 1])
        # wrist pitch axis position = TCP minus Lw along the (projected) tool axis
        a_plane = Rb @ np.array([math.cos(phi), 0.0, -math.sin(phi)])
        w = p - Lw * a_plane
        e = Rb[:, 0]
        u = float(np.dot(w, e))  # forward distance in arm plane
        v = -(float(w[2]) - h)  # downward distance from the shoulder axis
        d2 = u * u + v * v
        D = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        if D > 1 + 1e-12:
            geom_reasons.append(
                f"{base_name}: wrist point {math.sqrt(d2):.4f} m from shoulder > reach {L1 + L2:.3f} m"
            )
            continue
        if D < -1 - 1e-12:
            geom_reasons.append(
                f"{base_name}: wrist point {math.sqrt(d2):.4f} m from shoulder < min reach {abs(L1 - L2):.3f} m"
            )
            continue
        D = min(1.0, max(-1.0, D))
        q2_mag = math.acos(D)
        for elbow_name, q2 in (("elbow_up", q2_mag), ("elbow_down", -q2_mag)):
            if q2_mag < 1e-9 and elbow_name == "elbow_down":
                continue  # straight arm: both branches coincide
            q1 = math.atan2(v, u) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
            q1 = wrap(q1)
            q3 = wrap(phi - q1 - q2)
            q = np.array([yaw, q1, q2, q3, wrap(roll)])
            branch = f"{base_name}/{elbow_name}"
            viol = _limit_violations(model, q, limit_tol)
            if viol:
                rejected.append((branch, "joint limits: " + "; ".join(viol)))
                continue
            if refine:
                # polish toward the *achievable* (projected) pose, so an infeasible
                # orientation request cannot drag the position off target
                T_proj = make_pose(p, Rb @ rot_y(phi) @ rot_x(roll))
                q = refine_ik(T_proj, q, model, max_iters=20, tol=1e-12)
            q = np.clip(q, model.lower, model.upper)
            err = pose_error(fk(q, model), T_des)
            sols.append(IKSolution(q, branch, float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))))

    # keep solutions that actually hit the position; orientation error is reported
    good = [s for s in sols if s.pos_err <= pos_tol]
    for s in sols:
        if s.pos_err > pos_tol:
            rejected.append((s.branch, f"position error {s.pos_err:.2e} m after refinement"))
    # de-duplicate (e.g. degenerate on-axis targets)
    uniq: list[IKSolution] = []
    for s in good:
        if not any(np.allclose(s.q, u.q, atol=1e-7) for u in uniq):
            uniq.append(s)
    uniq.sort(key=lambda s: (s.rot_err > rot_tol, not s.branch.startswith("front"), "elbow_down" in s.branch))

    reason = ""
    if not uniq:
        parts = geom_reasons + [f"{b}: {why}" for b, why in rejected]
        reason = "unreachable: " + (" | ".join(parts) if parts else "no branch")
    return IKResult(uniq, reason, rejected)


def solve_ik_xyz(
    p, pitch: float, roll: float = 0.0, model: ArmModel | None = None, **kw
) -> IKResult:
    """IK for TCP position ``p`` with tool pitch (positive = pointing down) and wrist roll."""
    return solve_ik(target_pose(p, pitch, roll), model, **kw)


def ik(
    p, pitch: float, roll: float = 0.0, q_ref=None, model: ArmModel | None = None
) -> np.ndarray:
    """Convenience: best IK solution (closest to ``q_ref``) or raise :class:`Unreachable`."""
    res = solve_ik_xyz(p, pitch, roll, model)
    if not res.ok:
        raise Unreachable(res.reason)
    return res.best(q_ref).q


def refine_ik(
    T_des: np.ndarray,
    q0,
    model: ArmModel | None = None,
    *,
    max_iters: int = 100,
    damping: float = 1e-3,
    tol: float = 1e-10,
    rot_weight: float = 0.1,
) -> np.ndarray:
    """Damped-least-squares refinement toward ``T_des``, clamped to joint limits.

    ``rot_weight`` scales orientation error (rad) against position error (m); 0.1 means
    1 rad of orientation error weighs like 0.1 m. For poses the 5-DOF arm cannot reach
    exactly this converges to the weighted least-squares nearest pose.
    """
    model = model or default_model()
    q = np.array(q0, dtype=float)[:N_ARM]
    W = np.diag([1.0, 1.0, 1.0, rot_weight, rot_weight, rot_weight])
    lam2 = damping**2
    prev = math.inf
    for _ in range(max_iters):
        e = W @ pose_error(fk(q, model), T_des)
        cost = float(e @ e)
        if cost < tol * tol:
            break
        J = W @ jacobian(q, model)
        dq = J.T @ np.linalg.solve(J @ J.T + lam2 * np.eye(6), e)
        q_new = np.clip(q + dq, model.lower, model.upper)
        e_new = W @ pose_error(fk(q_new, model), T_des)
        if float(e_new @ e_new) > cost:  # step got worse: increase damping
            lam2 *= 10
            if lam2 > 1e4:
                break
            continue
        lam2 = max(lam2 / 10, damping**2)
        q = q_new
        if abs(prev - cost) < 1e-30:
            break
        prev = cost
    return q


def solve_ik_numeric(T_des, q_seed, model: ArmModel | None = None, **kw) -> IKSolution:
    """Pure numerical IK from a seed (DLS). Always returns the best effort; check errors."""
    model = model or default_model()
    q = refine_ik(T_des, q_seed, model, **kw)
    err = pose_error(fk(q, model), T_des)
    return IKSolution(q, "numeric", float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:])))
