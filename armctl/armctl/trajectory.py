"""Python twin of the firmware's trajectory generator (firmware/core/src/traj.c).

libarmfw's C API does not expose the trajectory (only the whole in-process simulator), so this is
a line-by-line port of ``traj.c``: per-joint quintics from the current commanded state
``(p, v, a)`` to ``(target, 0, 0)``, one shared duration stretched until no joint exceeds vmax
(``search_T``), the constant-jerk acceleration-removal blend used when no single quintic can
respect vmax (``choose_blends``), and the per-step limit saturation with rest-to-rest
continuation (``traj_step``). Units inside are the firmware's (mrad, mrad/s, mrad/s², seconds),
so every constant (vmax floor 1 mrad/s, |a| > 1 mrad/s² blend trigger, ...) is the same; the
public helpers take radians.

Differences from the C code (checked by ``tests/test_trajectory.py`` against traj.c compiled on
the fly and by ``tests/integration/test_trajectory_vs_firmware.py`` against the firmware
simulator's ``cmd_pos_crc``):

* float64 instead of float32;
* the peak velocity / position range of a quintic come from exact polynomial roots (numpy
  eigenvalues) instead of 8 Illinois iterations; the C result differs by ~1e-8 relative.

Both can flip a threshold decision only when a duration-search ratio lands within ~1e-6 of
1.0, which is why the integration test tolerates a rare ±1 mrad rounding difference.

Used by :class:`armctl.safety.SafetyShim` to check the path the board will actually execute,
including mid-move replans that start with non-zero velocity/acceleration (the joint-space
"straight line" assumption only holds for rest-to-rest moves).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

N = 6
TRAJ_MAX_DURATION_S = 60.0
TRAJ_BLEND_MAX_S = 0.020
TRAJ_BLEND_VFRAC = 0.95
TRAJ_SEARCH_ITERS = 8
CONTROL_PERIOD_US = 1000


# --------------------------------------------------------------------------- quintic helpers
def quintic_coeffs(p0, v0, a0, pf, T):
    """Normalised coefficients b (n,6) of P(s) = sum b_k s^k, s = t/T (traj.c quintic_plan)."""
    p0, v0, a0, pf, T = np.broadcast_arrays(*(np.asarray(x, float) for x in (p0, v0, a0, pf, T)))
    T = np.where(T > 1e-6, T, 1e-6)
    V0 = v0 * T
    A0 = a0 * T * T
    h = pf - p0 - V0 - 0.5 * A0
    g = -V0 - A0
    k = -A0
    b = np.stack([p0, V0, 0.5 * A0, 10 * h - 4 * g + 0.5 * k, -15 * h + 7 * g - k, 6 * h - 3 * g + 0.5 * k], axis=-1)
    return b, T


def _polyval(c, s):
    """Evaluate polynomials c (...,deg+1) (ascending) at s (broadcast over the leading dims)."""
    r = c[..., -1]
    for k in range(c.shape[-1] - 2, -1, -1):
        r = r * s + c[..., k]
    return r


def _deriv(c):
    k = np.arange(1, c.shape[-1])
    return c[..., 1:] * k


def _roots01_row(row):
    r = row.copy()
    scale = np.max(np.abs(r))
    if scale == 0:
        return np.empty(0)
    while len(r) > 1 and abs(r[-1]) <= 1e-7 * scale:
        r = r[:-1]
    if len(r) <= 1:
        return np.empty(0)
    if len(r) == 2:
        rt = np.array([-r[0] / r[1]])
    elif len(r) == 3:
        a, b, k = r[2], r[1], r[0]
        disc = b * b - 4 * a * k
        if disc < 0:
            return np.empty(0)
        sq = np.sqrt(disc)
        q = -0.5 * (b + (sq if b >= 0 else -sq))
        rt = np.array([q / a, k / q]) if q != 0 else np.array([0.0])
    else:
        rt = np.roots(r[::-1])
        rt = rt[np.abs(rt.imag) < 1e-9].real
    return rt[(rt > 0.0) & (rt < 1.0)]


def _roots01(c):
    """Real roots in (0,1) of each row of ascending coefficient array c (n, deg+1); list of arrays.

    Rows of full degree go through one batched companion-matrix eigvals call (what np.roots does
    per row); degenerate rows (tiny leading coefficient) take the per-row path."""
    c = np.asarray(c, float)
    n, m = c.shape
    deg = m - 1
    out = [None] * n
    scale = np.max(np.abs(c), axis=1)
    full = (scale > 0) & (np.abs(c[:, -1]) > 1e-7 * scale)
    if deg >= 3 and np.any(full):
        idx = np.nonzero(full)[0]
        cf = c[idx]
        comp = np.zeros((len(idx), deg, deg))
        comp[:, 0, :] = -cf[:, -2::-1] / cf[:, -1:]
        if deg > 1:
            comp[:, np.arange(1, deg), np.arange(deg - 1)] = 1.0
        ev = np.linalg.eigvals(comp)
        for k, j in enumerate(idx):
            rt = ev[k]
            rt = rt[np.abs(rt.imag) < 1e-9].real
            out[j] = rt[(rt > 0.0) & (rt < 1.0)]
    for j in range(n):
        if out[j] is None:
            out[j] = _roots01_row(c[j])
    return out


def quintic_peak_vel(b, T):
    """Peak |velocity| over [0, T] of each quintic (b (n,6), T (n,))."""
    d1 = _deriv(b)
    d2 = _deriv(d1)
    peaks = np.maximum(np.abs(d1[:, 0]), np.abs(_polyval(d1, 1.0)))
    for j, rts in enumerate(_roots01(d2)):
        if len(rts):
            peaks[j] = max(peaks[j], float(np.max(np.abs(_polyval(d1[j], rts)))))
    return peaks / T


def quintic_pos_range(b, pf):
    """(min, max) position over [0, T] of each quintic."""
    d1 = _deriv(b)
    lo = np.minimum(b[:, 0], pf)
    hi = np.maximum(b[:, 0], pf)
    for j, rts in enumerate(_roots01(d1)):
        if len(rts):
            v = _polyval(b[j], rts)
            lo[j] = min(lo[j], float(v.min()))
            hi[j] = max(hi[j], float(v.max()))
    return lo, hi


@dataclass
class PlanResult:
    pos_clamped: bool = False
    stretched: bool = False
    blended: bool = False
    vmax_infeasible: bool = False
    overshoot: bool = False
    T: float = 0.0
    peak_ratio: float = 0.0
    evals: int = 0


class FirmwareTrajectory:
    """State of the board's trajectory group (6 joints), in mrad.

    ``lim_min``/``lim_max``/``vmax`` default to spec/kinematics.yaml as the firmware's defaults
    (``config.c``), rounded to whole mrad like the firmware's int16 config."""

    def __init__(self, lim_min=None, lim_max=None, vmax=None, model=None):
        if lim_min is None or lim_max is None or vmax is None:
            from .kinematics import default_model

            m = model or default_model()
            lim_min = np.round(m.lower6 * 1000) if lim_min is None else lim_min
            lim_max = np.round(m.upper6 * 1000) if lim_max is None else lim_max
            vmax = np.round(m.vmax * 1000) if vmax is None else vmax
        self.lim_min = np.asarray(lim_min, float).copy()
        self.lim_max = np.asarray(lim_max, float).copy()
        self.vmax = np.asarray(vmax, float).copy()
        self.reset(np.zeros(N))

    # ---- state -------------------------------------------------------------------------
    def reset(self, pos_mrad) -> None:
        """traj_reset: hold at pos with zero velocity/acceleration."""
        pos = np.asarray(pos_mrad, float).copy()
        self.cmd_p, self.cmd_v, self.cmd_a = pos.copy(), np.zeros(N), np.zeros(N)
        self.b, self.T = quintic_coeffs(pos, 0.0, 0.0, pos, 1e-3)
        self.pf = pos.copy()
        self.t0 = np.zeros(N)
        self.tau = np.zeros(N)
        self.bl_p0 = np.zeros(N)
        self.bl_v0 = np.zeros(N)
        self.bl_a0 = np.zeros(N)
        self.jerk = np.zeros(N)
        self.elapsed_us = 0
        self.moving = False
        self.saturated = False
        self._may_saturate = False

    def freeze(self) -> None:
        self.reset(self.cmd_p)

    def copy(self) -> "FirmwareTrajectory":
        c = object.__new__(type(self))
        c.__dict__.update({k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in self.__dict__.items()})
        return c

    # ---- blends ------------------------------------------------------------------------
    def _blend_end(self):
        tau = self.tau
        p = self.bl_p0 + tau * (self.bl_v0 + tau * (0.5 * self.bl_a0 + tau * (self.jerk / 6.0)))
        v = self.bl_v0 + tau * (self.bl_a0 + 0.5 * self.jerk * tau)
        return p, v

    def _choose_blends(self) -> float:
        self.bl_p0, self.bl_v0, self.bl_a0 = self.cmd_p.copy(), self.cmd_v.copy(), self.cmd_a.copy()
        tau = np.zeros(N)
        jerk = np.zeros(N)
        for j in range(N):
            a, v = self.cmd_a[j], self.cmd_v[j]
            if abs(a) > 1.0:
                t = TRAJ_BLEND_MAX_S
                vcap = TRAJ_BLEND_VFRAC * max(self.vmax[j], 1.0)
                if (a > 0.0) == (v > 0.0):
                    room = vcap - abs(v)
                    tau_v = 2.0 * room / abs(a) if room > 0.0 else 0.0
                    t = min(t, tau_v)
                t = max(t, 1e-3)
                tau[j], jerk[j] = t, -a / t
        self.tau, self.jerk = tau, jerk
        return float(tau.max())

    # ---- planning ----------------------------------------------------------------------
    def _plan_all(self, tgt, T_total, use_blend) -> float:
        vmax = np.maximum(self.vmax, 1.0)
        if use_blend:
            bl = self.tau > 0.0
            pe, ve = self._blend_end()
            p0 = np.where(bl, pe, self.cmd_p)
            v0 = np.where(bl, ve, self.cmd_v)
            a0 = np.where(bl, 0.0, self.cmd_a)
            T = np.where(bl, T_total - self.tau, T_total)
            self.t0 = np.where(bl, self.tau, 0.0)
        else:
            p0, v0, a0 = self.cmd_p, self.cmd_v, self.cmd_a
            T = np.full(N, T_total)
            self.t0 = np.zeros(N)
        self.b, self.T = quintic_coeffs(p0, v0, a0, tgt, T)
        self.pf = np.asarray(tgt, float).copy()
        peak = quintic_peak_vel(self.b, self.T)
        if use_blend:
            pe, ve = self._blend_end()
            bl = self.tau > 0.0
            peak = np.where(bl, np.maximum(np.maximum(np.abs(self.bl_v0), np.abs(ve)), peak), peak)
        return float(np.max(peak / vmax))

    def _search_T(self, tgt, T_min, fixed, use_blend, res: PlanResult):
        goal = 0.999
        T = T_min
        ratio = self._plan_all(tgt, T, use_blend)
        res.evals += 1
        best_T, best_ratio = T, ratio
        T_prev = r_prev = 0.0
        it = 0
        while it < TRAJ_SEARCH_ITERS and ratio > 1.0 and T < TRAJ_MAX_DURATION_S:
            span = T - fixed
            if it == 0 or not (r_prev > ratio):
                Tn = fixed + span * min(ratio / goal, 8.0)
            else:
                x1, x2 = 1.0 / (T_prev - fixed), 1.0 / span
                xn = x2 + (goal - ratio) * (x2 - x1) / (ratio - r_prev)
                Tn = fixed + 1.0 / xn if xn > 0.0 else fixed + span * 8.0
                Tn = min(max(Tn, fixed + span * 1.001), fixed + span * 8.0)
            T_prev, r_prev = T, ratio
            T = min(Tn, TRAJ_MAX_DURATION_S)
            ratio = self._plan_all(tgt, T, use_blend)
            res.evals += 1
            if ratio < best_ratio:
                best_ratio, best_T = ratio, T
            it += 1
            if ratio > 1.0 and ratio >= r_prev:
                break
        if best_T != T:
            T = best_T
            ratio = self._plan_all(tgt, T, use_blend)
            res.evals += 1
        return T, ratio

    def replan(self, target_mrad, move_s: float) -> PlanResult:
        """traj_replan: from the current commanded state to ``target`` (mrad) over ``move_s``."""
        res = PlanResult()
        target = np.asarray(target_mrad, float)
        tgt = np.clip(target, self.lim_min, self.lim_max)
        res.pos_clamped = bool(np.any(tgt != target))
        self.tau = np.zeros(N)
        T0 = min(max(float(move_s), 1e-3), TRAJ_MAX_DURATION_S)
        T, ratio = self._search_T(tgt, T0, 0.0, False, res)
        if ratio > 1.0:
            max_tau = self._choose_blends()
            if max_tau > 0.0:
                Tb, ratio_b = self._search_T(tgt, max(T0, max_tau + 1e-3), max_tau, True, res)
                if ratio_b < ratio:
                    res.blended = True
                    T, ratio = Tb, ratio_b
                else:
                    self.tau = np.zeros(N)
                    ratio = self._plan_all(tgt, T, False)
                    res.evals += 1
            res.vmax_infeasible = ratio > 1.0
        res.T, res.peak_ratio = T, ratio
        res.stretched = T > T0 * 1.000001
        lo, hi = quintic_pos_range(self.b, self.pf)
        res.overshoot = bool(np.any(lo < self.lim_min - 0.01) or np.any(hi > self.lim_max + 0.01))
        # step() == eval() exactly unless a limit can be touched (quintic range, or a blend cubic,
        # whose range is not computed): advance() uses that to skip the per-ms stepping
        self._may_saturate = bool(np.any(lo < self.lim_min) or np.any(hi > self.lim_max) or np.any(self.tau > 0))
        self.elapsed_us = 0
        self.moving = True
        return res

    # ---- evaluation --------------------------------------------------------------------
    def eval(self, t):
        """(p, v, a) of every joint at move time(s) t (s): arrays (6,) or (len(t), 6). No
        saturation (see :meth:`step` / :meth:`sample`)."""
        t = np.asarray(t, float)
        tt = t[..., None] if t.ndim else t
        in_blend = tt < self.tau
        tb = np.maximum(tt, 0.0)
        pb = self.bl_p0 + tb * (self.bl_v0 + tb * (0.5 * self.bl_a0 + tb * (self.jerk / 6.0)))
        vb = self.bl_v0 + tb * (self.bl_a0 + 0.5 * self.jerk * tb)
        ab = self.bl_a0 + self.jerk * tb
        tq = tt - self.t0
        done = tq >= self.T
        s = np.clip(tq, 0.0, None) / self.T
        d1 = _deriv(self.b)
        d2 = _deriv(d1)
        pq = np.where(done, self.pf, _polyval(self.b, s))
        vq = np.where(done, 0.0, _polyval(d1, s) / self.T)
        aq = np.where(done, 0.0, _polyval(d2, s) / self.T**2)
        return (np.where(in_blend, pb, pq), np.where(in_blend, vb, vq), np.where(in_blend, ab, aq))

    @property
    def duration(self) -> float:
        """Time until every joint has reached its target (s), from the start of the move."""
        return float(np.max(self.t0 + self.T))

    @property
    def remaining(self) -> float:
        return max(0.0, self.duration - self.elapsed_us * 1e-6) if self.moving else 0.0

    def advance(self, dt_s: float) -> None:
        """Advance by ``dt_s`` seconds of 1 kHz control steps. Exact (step by step) for short
        gaps; a gap reaching past the end of the move jumps to the end."""
        n = int(round(dt_s * 1e6 / CONTROL_PERIOD_US))
        if n <= 0:
            return
        if not self.moving or (self.elapsed_us + n * CONTROL_PERIOD_US) * 1e-6 >= self.duration:
            if self.moving:
                self.cmd_p = np.clip(self.pf, self.lim_min, self.lim_max)
                self.cmd_v, self.cmd_a = np.zeros(N), np.zeros(N)
                self.moving = False
            return
        if n > 50 or not self._may_saturate:
            # no limit contact possible (or a long gap: saturation is then not emulated): the
            # stepped state equals the direct evaluation at the final step
            self.elapsed_us += (n - 1) * CONTROL_PERIOD_US
            n = 1
        for _ in range(n):
            self.step()

    def step(self, dt_us: int = CONTROL_PERIOD_US) -> None:
        """traj_step: advance time, update the commanded state, saturate at the limits."""
        self.saturated = False
        t = 0.0
        if self.moving:
            self.elapsed_us += dt_us
            t = self.elapsed_us * 1e-6
            self.cmd_p, self.cmd_v, self.cmd_a = (x.copy() for x in self.eval(t))
            if np.all(t >= self.t0 + self.T):
                self.moving = False
        out = (self.cmd_p < self.lim_min) | (self.cmd_p > self.lim_max)
        for j in np.nonzero(out)[0]:
            self.cmd_p[j] = min(max(self.cmd_p[j], self.lim_min[j]), self.lim_max[j])
            self.cmd_v[j] = 0.0
            self.cmd_a[j] = 0.0
            self.saturated = True
            self._may_saturate = True
            if self.moving:
                pf = self.pf[j]
                rem = self.t0[j] + self.T[j] - t
                vmax = max(self.vmax[j], 1.0)
                tmin = 1.875 * abs(pf - self.cmd_p[j]) / vmax * 1.002
                bj, Tj = quintic_coeffs(self.cmd_p[j], 0.0, 0.0, pf, max(max(rem, tmin), 1e-3))
                self.b[j], self.T[j] = bj, Tj
                self.t0[j] = t
                self.tau[j] = 0.0

    def sample(self, dt: float = 0.01, max_samples: int = 400, min_samples: int = 1) -> np.ndarray:
        """Commanded positions (mrad, (n+1,6)) from now to the end of the move: the current
        setpoint, then every ``dt`` seconds (at least ``min_samples``, at most ``max_samples``
        points), always including the end. Positions are clipped to the limits (the firmware
        saturates there)."""
        start = self.elapsed_us * 1e-6 if self.moving else 0.0
        end = self.duration if self.moving else 0.0
        if end <= start:
            return self.cmd_p[None].copy()
        n = int(np.ceil((end - start) / dt))
        n = max(1, min_samples, min(n, max_samples))
        ts = np.linspace(start, end, n + 1)[1:]
        p, _, _ = self.eval(ts)
        p = np.vstack([self.cmd_p[None], p])
        return np.clip(p, self.lim_min, self.lim_max)


def f2i16(x) -> np.ndarray:
    """Firmware's float -> int16 rounding (round half away from zero, saturating)."""
    x = np.asarray(x, float)
    r = np.where(x >= 0, np.floor(x + 0.5), -np.floor(-x + 0.5))
    return np.clip(r, -32768, 32767).astype(np.int64)
