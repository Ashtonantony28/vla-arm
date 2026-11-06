"""Host-side safety shim, applied to every joint command before it is sent.

Checks, in order:

1. **Joint limits** – clamp each of the 6 servos to its limits (optionally shrunk by a margin).
2. **Velocity** – limit the per-joint step from the previous *sent* command to
   ``vmax * dt`` where ``dt`` is the move duration (or the elapsed time, whichever is longer).
3. **Workspace** – TCP and wrist (wrist-pitch axis) must stay at ``z >= z_min`` (table plane);
   the TCP must stay outside a vertical cylinder of radius ``base_radius`` around the base.
   The straight joint-space segment from the previous command is sampled; if any sample
   violates the workspace the command is pulled back along that segment to the furthest
   safe fraction (bisection). With no previous safe command it is rejected.

The board also clamps to limits/vmax; this shim exists so the host never *asks* for
something unsafe and so every modification is explained.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .kinematics import ArmModel, default_model, joint_positions

N = 6


@dataclass
class SafetyConfig:
    z_min: float = 0.01  # m, table plane margin for TCP and wrist
    base_radius: float = 0.07  # m, keep-out cylinder around the base axis (TCP)
    limit_margin: float = 0.0  # rad, shrink joint limits by this
    vel_scale: float = 1.0  # fraction of kinematics.yaml vmax allowed per step
    path_samples: int = 5  # samples along the segment from the previous command
    vmax: np.ndarray | None = None  # override per-joint vmax (6,), rad/s


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


class SafetyShim:
    def __init__(self, model: ArmModel | None = None, config: SafetyConfig | None = None):
        self.model = model or default_model()
        self.cfg = config or SafetyConfig()
        m = self.cfg.limit_margin
        self.lower = self.model.lower6 + m
        self.upper = self.model.upper6 - m
        vmax = self.cfg.vmax if self.cfg.vmax is not None else self.model.vmax
        self.vmax = np.asarray(vmax, dtype=float) * self.cfg.vel_scale
        self.last: np.ndarray | None = None  # last command that was let through

    def reset(self, q_current=None) -> None:
        """Forget (or seed) the previous command, e.g. with the measured joint state."""
        self.last = None if q_current is None else np.asarray(q_current, dtype=float).copy()

    # ---- workspace ------------------------------------------------------------------
    def workspace_violations(self, q) -> list[str]:
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

    def violation_depth(self, q) -> float:
        """Total distance (m) by which the workspace constraints are violated (0 = safe)."""
        pts = joint_positions(q, self.model)
        d = sum(max(0.0, self.cfg.z_min - float(pts[n][2])) for n in ("tcp", "wrist"))
        return d + max(0.0, self.cfg.base_radius - math.hypot(*pts["tcp"][:2]))

    def _segment_ok(self, q0: np.ndarray, q1: np.ndarray) -> bool:
        n = max(1, self.cfg.path_samples)
        for k in range(1, n + 1):
            if self.workspace_violations(q0 + (q1 - q0) * (k / n)):
                return False
        return True

    # ---- main entry -----------------------------------------------------------------
    def filter(self, q, dt: float, commit: bool = True) -> SafetyReport:
        """Make ``q`` (6 joint targets, rad) safe for a move lasting ``dt`` seconds."""
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

        # 3. workspace
        viol = self.workspace_violations(q)
        last_viol = self.workspace_violations(self.last) if self.last is not None else []
        if last_viol and self.violation_depth(q) <= self.violation_depth(self.last) + 1e-9:
            # previous pose was outside the workspace (e.g. arm resting on the table at start):
            # allow steps that do not make the violation worse, so the arm can get back in.
            reasons.append("workspace: previous pose unsafe (" + "; ".join(last_viol) + "), allowing recovery step")
        elif viol or (self.last is not None and not self._segment_ok(self.last, q)):
            if self.last is None or last_viol:
                return SafetyReport(q_in, None, reasons + ["workspace: " + "; ".join(viol or ["path"])])
            lo, hi = 0.0, 1.0
            for _ in range(20):
                mid = (lo + hi) / 2
                if self._segment_ok(self.last, self.last + (q - self.last) * mid):
                    lo = mid
                else:
                    hi = mid
            q = self.last + (q - self.last) * lo
            reasons.append(
                "workspace: " + "; ".join(viol or ["path leaves workspace"]) + f" -> scaled step to {lo:.3f}"
            )

        if commit:
            self.last = q.copy()
        return SafetyReport(q_in, q, reasons)
