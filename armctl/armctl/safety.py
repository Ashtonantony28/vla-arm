"""Host-side safety shim, applied to every joint command before it is sent.

Checks, in order:

1. **Joint limits** – clamp each of the 6 servos to its limits (optionally shrunk by a margin).
2. **Velocity** – limit the per-joint step from the previous *sent* command to
   ``vmax * dt`` where ``dt`` is the move duration (or the elapsed time, whichever is longer).
3. **Workspace and collisions** – TCP and wrist (wrist-pitch axis) must stay at
   ``z >= z_min`` (table plane); the TCP must stay outside a vertical cylinder of radius
   ``base_radius`` around the base; and the capsule model of every link
   (:mod:`armctl.collision`, fitted to the CAD) must not touch a non-adjacent link, the base or
   the table. The check runs along the **path the board will execute**: a Python twin of the
   firmware's trajectory generator (:mod:`armctl.trajectory`) replans from the board's current
   commanded state (position, velocity, acceleration; advanced to ``now`` when the caller
   passes a timestamp) with the same quintic, vmax stretching and blend, and the planned
   setpoints are sampled. If the path leaves the safe set the command is pulled back toward
   the previous command (bisection on the step); if even the zero step is unsafe the command
   is rejected, so the board keeps executing the previously verified plan.

With no timestamp the previous move is assumed finished (the rest-to-rest quintic, i.e. the
straight joint-space line). If the previous pose was itself unsafe (e.g. the arm slumped on the
table at power-up), steps that do not increase the total violation depth are allowed so the arm
can be brought back.

The board also clamps to limits/vmax; this shim exists so the host never *asks* for
something unsafe and so every modification is explained.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .collision import LINKS, CollisionModel
from .kinematics import ArmModel, default_model, joint_positions
from .trajectory import FirmwareTrajectory

N = 6


@dataclass
class SafetyConfig:
    z_min: float = 0.01  # m, table plane margin for TCP and wrist points
    base_radius: float = 0.07  # m, keep-out cylinder around the base axis (TCP)
    limit_margin: float = 0.0  # rad, shrink joint limits by this
    vel_scale: float = 1.0  # fraction of kinematics.yaml vmax allowed per step
    path_samples: int = 5  # minimum samples along the planned path
    vmax: np.ndarray | None = None  # override per-joint vmax (6,), rad/s
    collision: bool = True  # capsule self/base/table collision check
    collision_margin: float = 0.005  # m, clearance required between links and to the base
    table_margin: float = 0.0  # m, clearance required between capsules and the table
    trajectory: bool = True  # check the firmware's quintic (twin) instead of a straight line
    traj_dt: float = 0.01  # s, path sampling period
    traj_max_samples: int = 200
    bisect_iters: int = 12


@dataclass
class SafetyReport:
    q_in: np.ndarray
    q_out: np.ndarray | None  # None = rejected (do not send)
    reasons: list[str] = field(default_factory=list)

    @property
    def modified(self) -> bool:
        return bool(self.reasons)

    @property
    def rejected(self) -> bool:
        return self.q_out is None

    def __str__(self) -> str:
        if not self.reasons:
            return "ok"
        return ("REJECTED: " if self.rejected else "modified: ") + "; ".join(self.reasons)


@dataclass
class ChunkReport:
    """Result of :meth:`SafetyShim.check_chunk`."""

    ok: np.ndarray  # (T,) bool per step
    depth: np.ndarray  # (T,) total violation depth per step (m)
    clearance: np.ndarray  # (T,) capsule clearance per step (m), inf without collision check
    first_bad: int | None  # index of the first unsafe step
    reason: str = ""


class SafetyShim:
    def __init__(self, model: ArmModel | None = None, config: SafetyConfig | None = None):
        self.model = model or default_model()
        self.cfg = config or SafetyConfig()
        m = self.cfg.limit_margin
        self.lower = self.model.lower6 + m
        self.upper = self.model.upper6 - m
        vmax = self.cfg.vmax if self.cfg.vmax is not None else self.model.vmax
        self.vmax = np.asarray(vmax, dtype=float) * self.cfg.vel_scale
        self.collision = (
            CollisionModel(self.model, margin=self.cfg.collision_margin, table_margin=self.cfg.table_margin,
                           exact_below=0.0)
            if self.cfg.collision
            else None
        )
        self.last: np.ndarray | None = None  # last command that was let through
        self.traj = FirmwareTrajectory(model=self.model)  # board's commanded state (mrad)
        self._t_last: float | None = None  # timestamp of the last committed command
        self._last_depth: float | None = None  # cached violation depth of self.last

    def reset(self, q_current=None) -> None:
        """Forget (or seed) the previous command, e.g. with the measured joint state. The
        trajectory twin is reset to rest there (the board starts from the measured position
        when torque is enabled)."""
        self.last = None if q_current is None else np.asarray(q_current, dtype=float).copy()
        if self.last is not None:
            self.traj.reset(self.last * 1000.0)
        self._t_last = None
        self._last_depth = None

    # ---- workspace ------------------------------------------------------------------
    def workspace_violations(self, q) -> list[str]:
        """Point checks (TCP/wrist above the table, TCP outside the base keep-out)."""
        pts = joint_positions(q, self.model)
        out = []
        for name in ("tcp", "wrist"):
            z = float(pts[name][2])
            if z < self.cfg.z_min:
                out.append(f"{name} z={z:.4f} m below table limit {self.cfg.z_min:.3f} m")
        r = math.hypot(*pts["tcp"][:2])
        if r < self.cfg.base_radius:
            out.append(f"tcp r={r:.4f} m inside base keep-out radius {self.cfg.base_radius:.3f} m")
        return out

    def collision_violations(self, q) -> list[str]:
        if self.collision is None:
            return []
        c = self.collision.check(np.asarray(q, float))
        return [f"collision {name} ({d * 1000:+.1f} mm)" for name, d in c.hits[0]]

    def violations(self, q) -> list[str]:
        return self.workspace_violations(q) + self.collision_violations(q)

    def _depths(self, Q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Total violation depth (m) and capsule clearance for each pose of Q (n,6)."""
        Q = np.atleast_2d(Q)
        R, t = (self.collision or self._frames_model()).link_frames(Q)
        wrist = t[:, LINKS.index("wrist")]
        g = LINKS.index("gripper")
        tcp = t[:, g] + R[:, g] @ self.model.tcp
        d = np.maximum(0.0, self.cfg.z_min - tcp[:, 2]) + np.maximum(0.0, self.cfg.z_min - wrist[:, 2])
        d = d + np.maximum(0.0, self.cfg.base_radius - np.hypot(tcp[:, 0], tcp[:, 1]))
        clear = np.full(len(Q), np.inf)
        if self.collision is not None:
            c = self.collision.clearance(Q, frames=(R, t))
            d = d + c.depth
            clear = c.min_clearance
        return d, clear

    def _frames_model(self) -> CollisionModel:
        if getattr(self, "_frames", None) is None:
            self._frames = CollisionModel(self.model, check_self=False, check_base=False, check_table=False)
        return self._frames

    def violation_depth(self, q) -> float:
        """Total distance (m) by which the workspace/collision constraints are violated."""
        return float(self._depths(np.asarray(q, float))[0][0])

    def _plan(self, q_target: np.ndarray, dt: float, now: float | None) -> tuple[FirmwareTrajectory, np.ndarray]:
        """Twin of the board after receiving q_target, and the setpoints it will run (rad)."""
        tw = self.traj.copy()
        if now is not None and self._t_last is not None:
            tw.advance(max(0.0, now - self._t_last))
        else:
            tw.advance(1e9)  # previous move assumed finished
        if not self.cfg.trajectory:  # straight joint-space line from the current setpoint
            start = tw.cmd_p / 1000.0
            s = np.linspace(0.0, 1.0, self.cfg.path_samples + 1)
            tw.replan(q_target * 1000.0, max(dt, 1e-3))
            return tw, start + (q_target - start) * s[:, None]
        tw.replan(q_target * 1000.0, max(dt, 1e-3))
        P = tw.sample(self.cfg.traj_dt, self.cfg.traj_max_samples, self.cfg.path_samples) / 1000.0
        return tw, P

    def _path_ok(self, P: np.ndarray) -> bool:
        return bool(np.all(self._depths(P[1:] if len(P) > 1 else P)[0] <= 0.0))

    # ---- action chunks ----------------------------------------------------------------
    def check_chunk(self, Q) -> ChunkReport:
        """Check a whole action chunk (T,6) at once (limits + workspace + collisions per step).
        Does not modify shim state; use it to screen a policy's chunk before streaming it."""
        Q = np.atleast_2d(np.asarray(Q, float))
        in_lim = np.all((Q >= self.lower - 1e-9) & (Q <= self.upper + 1e-9), axis=1)
        depth, clear = self._depths(Q)
        ok = in_lim & (depth <= 0.0)
        bad = np.flatnonzero(~ok)
        first = int(bad[0]) if len(bad) else None
        reason = ""
        if first is not None:
            v = self.violations(Q[first])
            reason = "; ".join(v) if v else "joint limits"
        return ChunkReport(ok, depth, clear, first, reason)

    # ---- main entry -----------------------------------------------------------------
    def filter(self, q, dt: float, commit: bool = True, now: float | None = None) -> SafetyReport:
        """Make ``q`` (6 joint targets, rad) safe for a move lasting ``dt`` seconds.

        ``now`` (seconds, any monotonic clock) is the send time; with it the trajectory twin
        is advanced from the previous command so mid-move replans are checked faithfully."""
        q_in = np.asarray(q, dtype=float).copy()
        if q_in.shape != (N,):
            raise ValueError(f"expected 6 joint values, got shape {q_in.shape}")
        reasons: list[str] = []
        if not np.all(np.isfinite(q_in)):
            return SafetyReport(q_in, None, ["non-finite joint value"])

        # 1. joint limits
        q = np.clip(q_in, self.lower, self.upper)
        for i in np.nonzero(q != q_in)[0]:
            reasons.append(
                f"joint {i} clamped {q_in[i]:+.3f} -> {q[i]:+.3f} (limits [{self.lower[i]:+.2f}, {self.upper[i]:+.2f}])"
            )

        # 2. per-step velocity
        if self.last is not None:
            max_step = self.vmax * max(dt, 1e-3)
            step = q - self.last
            lim = np.clip(step, -max_step, max_step)
            for i in np.nonzero(np.abs(lim - step) > 1e-12)[0]:
                reasons.append(
                    f"joint {i} step {step[i]:+.3f} rad limited to {lim[i]:+.3f} "
                    f"(vmax {self.vmax[i]:.2f} rad/s over {dt * 1000:.0f} ms)"
                )
            q = self.last + lim

        # 3. workspace + collisions along the planned path
        if self.last is None:
            viol = self.violations(q)
            if viol:
                return SafetyReport(q_in, None, reasons + ["workspace: " + "; ".join(viol)])
            plan = None
        else:
            if self._last_depth is None:
                self._last_depth = self.violation_depth(self.last)
            depth_last = self._last_depth
            plan, P = self._plan(q, dt, now)
            new_depth = 0.0
            if depth_last > 0.0:
                new_depth = self.violation_depth(q)
                if new_depth <= depth_last + 1e-9:
                    # previous pose was outside the safe set (e.g. arm resting on the table at
                    # start): allow steps that do not make the violation worse
                    reasons.append(
                        "workspace: previous pose unsafe (" + "; ".join(self.violations(self.last))
                        + "), allowing recovery step"
                    )
                else:
                    return SafetyReport(q_in, None, reasons + ["workspace: " + "; ".join(self.violations(q) or ["deeper"])])
            elif not self._path_ok(P):
                viol = self.violations(q)
                lo, hi = 0.0, 1.0
                best = None
                for _ in range(self.cfg.bisect_iters):
                    mid = (lo + hi) / 2
                    cand = self.last + (q - self.last) * mid
                    pl, Pm = self._plan(cand, dt, now)
                    if self._path_ok(Pm):
                        lo, best = mid, (cand, pl)
                    else:
                        hi = mid
                if best is None:
                    cand = self.last.copy()
                    pl, Pm = self._plan(cand, dt, now)
                    if not self._path_ok(Pm):
                        return SafetyReport(
                            q_in, None, reasons + ["workspace: " + "; ".join(viol or ["planned path leaves the safe set"])
                                                   + " (even holding the previous target)"]
                        )
                    best = (cand, pl)
                q, plan = best
                reasons.append(
                    "workspace: " + "; ".join(viol or ["path leaves workspace"]) + f" -> scaled step to {lo:.3f}"
                )

        if commit:
            self._last_depth = new_depth if self.last is not None else 0.0
            self.last = q.copy()
            if plan is None:  # first command: assume the board ends up at q
                self.traj.reset(q * 1000.0)
            else:
                self.traj = plan
            self._t_last = now
        return SafetyReport(q_in, q, reasons)
