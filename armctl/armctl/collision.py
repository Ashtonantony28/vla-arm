"""Self- and environment-collision checking with a capsule model of the arm.

Geometry lives in ``armctl/data/collision_model.yaml`` (capsule radii fitted to the CAD point
clouds, see the comments there and ``armctl/tools/fit_capsules.py``). Checked, for every pose:

* **link-link**: every capsule pair of two *non-adjacent* moving links (shoulder bracket, upper
  arm, forearm, wrist bracket, gripper body, jaw A, jaw B). Adjacent pairs are guaranteed clear by
  the joint limits (CAD report, "adjacent-pair free range"); the two jaws are not checked against
  each other (their pads touch when closed);
* **link-base**: every capsule of a link not adjacent to the base vs the static base primitives
  (base plate, clamp tabs, ribs, servo housing: boxes and vertical cylinders) and the turntable;
* **link-table**: the lowest point of every capsule vs the table plane ``z = table_z``.

Everything is vectorised over a batch of poses (an action chunk, or samples of a trajectory):
:meth:`CollisionModel.clearance` returns the signed clearance of every check for each pose, with
bounding-sphere culling so far-apart pairs cost almost nothing. Distances are exact for
capsule-capsule and capsule-plane; capsule-box/cylinder brackets the minimum over the segment
with three 12-point grids (the signed distance to a convex solid is convex along a segment) and
returns a guaranteed lower bound within 0.15 mm of the exact value.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .kinematics import ArmModel, default_model

DATA = Path(__file__).with_name("data") / "collision_model.yaml"
LINKS = ("shoulder", "upper_arm", "forearm", "wrist", "gripper", "jaw_a", "jaw_b")
_GRID_K, _GRID_PASSES = 12, 3  # segment-vs-static bracketing grid (see _seg_static)
_GRID = np.linspace(0.0, 1.0, _GRID_K)


def _rot_batch(axis: np.ndarray, q: np.ndarray) -> np.ndarray:
    """(B,3,3) rotations about a fixed unit axis (Rodrigues; direct for coordinate axes)."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.flatnonzero(np.abs(a) > 1 - 1e-12)
    if len(k) == 1:
        i = int(k[0])
        j, l = (i + 1) % 3, (i + 2) % 3
        s, c = np.sin(q) * np.sign(a[i]), np.cos(q)
        R = np.zeros((len(q), 3, 3))
        R[:, i, i] = 1.0
        R[:, j, j] = c
        R[:, l, l] = c
        R[:, l, j] = s
        R[:, j, l] = -s
        return R
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    s, c = np.sin(q)[:, None, None], np.cos(q)[:, None, None]
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


def _seg_seg_dist(p1, q1, p2, q2):
    """Distance between segments [p1,q1] and [p2,q2] (arrays (...,3)); Ericson 5.1.9,
    vectorised. Degenerate segments (points) and parallel segments are handled by clamping."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a = np.einsum("...i,...i", d1, d1)
    e = np.einsum("...i,...i", d2, d2)
    f = np.einsum("...i,...i", d2, r)
    c = np.einsum("...i,...i", d1, r)
    b = np.einsum("...i,...i", d1, d2)
    tiny = 1e-30
    # closest point of the infinite lines, clamped to segment 1; for (near-)parallel lines any
    # s works because t is clamped and s recomputed below
    s = np.minimum(np.maximum((b * f - c * e) / np.maximum(a * e - b * b, tiny), 0.0), 1.0)
    t = (b * s + f) / np.maximum(e, tiny)
    tc = np.minimum(np.maximum(t, 0.0), 1.0)
    s = np.where(t != tc, np.minimum(np.maximum((b * tc - c) / np.maximum(a, tiny), 0.0), 1.0), s)
    w = r + d1 * s[..., None] - d2 * tc[..., None]
    return np.sqrt(np.einsum("...i,...i", w, w))


def _sd_box_c(x, y, z, c, h):
    """Signed distance to axis-aligned boxes (centre c (...,3), half-size h (...,3))."""
    qx = np.abs(x - c[..., 0]) - h[..., 0]
    qy = np.abs(y - c[..., 1]) - h[..., 1]
    qz = np.abs(z - c[..., 2]) - h[..., 2]
    ox, oy, oz = np.maximum(qx, 0.0), np.maximum(qy, 0.0), np.maximum(qz, 0.0)
    return np.sqrt(ox * ox + oy * oy + oz * oz) + np.minimum(np.maximum(np.maximum(qx, qy), qz), 0.0)


def _sd_zcyl_c(x, y, z, xy, r, zc, hz):
    """Signed distance to vertical cylinders (axis at xy (...,2), radius r, centre height zc,
    half-height hz)."""
    dr = np.hypot(x - xy[..., 0], y - xy[..., 1]) - r
    dz = np.abs(z - zc) - hz
    odr, odz = np.maximum(dr, 0.0), np.maximum(dz, 0.0)
    return np.sqrt(odr * odr + odz * odz) + np.minimum(np.maximum(dr, dz), 0.0)


@dataclass
class Clearance:
    """Result of :meth:`CollisionModel.clearance` for a batch of B poses."""

    min_clearance: np.ndarray  # (B,) metres; negative = penetration (beyond the margin)
    worst_code: np.ndarray  # (B,) index of the worst check per pose
    depth: np.ndarray  # (B,) sum of all penetrations (>= 0), a "how bad" measure for recovery
    hits: list[list[tuple[str, float]]] = field(default_factory=list)  # per pose: (check, clearance<0)
    model: "CollisionModel | None" = field(default=None, repr=False)

    @property
    def worst(self) -> list[str]:
        """(B,) human-readable name of the worst check per pose."""
        return [
            self.model._code_name(int(c)) if np.isfinite(v) else "-"
            for c, v in zip(self.worst_code, self.min_clearance)
        ]

    @property
    def ok(self) -> np.ndarray:
        return self.min_clearance >= 0.0


class CollisionModel:
    """Capsule model of the arm. ``margin`` is added to every link-link / link-base check,
    ``table_margin`` to the link-table check (metres)."""

    def __init__(
        self,
        model: ArmModel | None = None,
        path: str | Path | None = None,
        *,
        margin: float = 0.0,
        table_margin: float = 0.0,
        table_z: float = 0.0,
        check_table: bool = True,
        check_base: bool = True,
        check_self: bool = True,
        exact_below: float = 0.01,
    ):
        self.model = model or default_model()
        data = yaml.safe_load(Path(path or DATA).read_text())
        self.margin, self.table_margin, self.table_z = float(margin), float(table_margin), float(table_z)
        self.check_table, self.check_base, self.check_self = check_table, check_base, check_self
        # pairs whose bounding spheres are further apart than this are not evaluated exactly;
        # min_clearance is exact below this value and a lower bound above it
        self.exact_below = float(exact_below)
        self.jaw_shift = float(data["jaw_shift_per_rad"])
        parent = data["parent"]
        names, link_idx, p0, p1, r = [], [], [], [], []
        for li, link in enumerate(LINKS):
            for c in data["links"][link]:
                names.append(f"{link}.{c['name']}")
                link_idx.append(li)
                p0.append(c["p0"])
                p1.append(c["p1"])
                r.append(c["r"])
        self.cap_names = names
        self.cap_link = np.array(link_idx)
        self.cap_p0 = np.array(p0, float)
        self.cap_p1 = np.array(p1, float)
        self.cap_r = np.array(r, float)
        # bounding sphere of every capsule (link frame): centre + radius
        self.cap_mid = (self.cap_p0 + self.cap_p1) / 2
        self.cap_brad = np.linalg.norm(self.cap_p1 - self.cap_p0, axis=1) / 2 + self.cap_r
        # stacked local points [p0; p1; mid] for the batched transform in _capsules
        self._cap_pts_T = np.vstack([self.cap_p0, self.cap_p1, self.cap_mid]).T.copy()
        self._cap_pick_l = np.tile(self.cap_link, 3)
        self._cap_pick_c = np.arange(3 * len(self.cap_link))

        def adjacent(a: str, b: str) -> bool:
            return parent.get(a) == b or parent.get(b) == a or {a, b} == {"jaw_a", "jaw_b"}

        # capsule-capsule pairs of non-adjacent links
        # ordered in contiguous blocks per link pair, row-major over (capsule of la, capsule of
        # lb), so the per-block dense distance matrices of clearance() line up with pair indices
        ii, jj = [], []
        self._blocks = []  # (start, end, la_slice, lb_slice)
        span = {li: (int(np.flatnonzero(self.cap_link == li)[0]), int(np.flatnonzero(self.cap_link == li)[-1]) + 1)
                for li in range(len(LINKS))}
        for la_i in range(len(LINKS)):
            for lb_i in range(la_i + 1, len(LINKS)):
                if adjacent(LINKS[la_i], LINKS[lb_i]):
                    continue
                start = len(ii)
                (sa, ea), (sb, eb) = span[la_i], span[lb_i]
                for i in range(sa, ea):
                    for j in range(sb, eb):
                        ii.append(i)
                        jj.append(j)
                self._blocks.append((start, len(ii), slice(sa, ea), slice(sb, eb)))
        self.pair_i, self.pair_j = np.array(ii, int), np.array(jj, int)
        self._pair_brad = self.cap_brad[self.pair_i] + self.cap_brad[self.pair_j]
        self._pair_r = self.cap_r[self.pair_i] + self.cap_r[self.pair_j]
        self._block_brad = [self.cap_brad[sa][:, None] + self.cap_brad[sb][None, :] for _, _, sa, sb in self._blocks]
        # static primitives
        self.static_names, kinds, boxes_lo, boxes_hi, cyl_xy, cyl_r, cyl_z = [], [], [], [], [], [], []
        static_link = []
        for s in data["static"]:
            self.static_names.append(s["name"])
            static_link.append(s["link"])
            if "box" in s:
                kinds.append(0)
                boxes_lo.append(s["box"]["lo"])
                boxes_hi.append(s["box"]["hi"])
                cyl_xy.append([0, 0])
                cyl_r.append(0.0)
                cyl_z.append([0, 0])
            else:
                kinds.append(1)
                g = s["zcyl"]
                boxes_lo.append([0, 0, 0])
                boxes_hi.append([0, 0, 0])
                cyl_xy.append(g["xy"])
                cyl_r.append(g["r"])
                cyl_z.append(g["z"])
        self.st_kind = np.array(kinds)
        self.st_lo, self.st_hi = np.array(boxes_lo, float), np.array(boxes_hi, float)
        self.st_xy, self.st_r, self.st_z = np.array(cyl_xy, float), np.array(cyl_r, float), np.array(cyl_z, float)
        # bounding sphere of each primitive (world)
        ctr, brad = [], []
        for k in range(len(kinds)):
            if kinds[k] == 0:
                ctr.append((self.st_lo[k] + self.st_hi[k]) / 2)
                brad.append(np.linalg.norm(self.st_hi[k] - self.st_lo[k]) / 2)
            else:
                z0, z1 = self.st_z[k]
                ctr.append([*self.st_xy[k], (z0 + z1) / 2])
                brad.append(math.hypot(self.st_r[k], (z1 - z0) / 2))
        self.st_ctr, self.st_brad = np.array(ctr, float), np.array(brad, float)
        self.st_c, self.st_h = (self.st_lo + self.st_hi) / 2, (self.st_hi - self.st_lo) / 2
        self.st_zc, self.st_hz = self.st_z.mean(axis=1), (self.st_z[:, 1] - self.st_z[:, 0]) / 2
        si, sj = [], []
        for i in range(len(names)):
            for k, sl in enumerate(static_link):
                la = LINKS[self.cap_link[i]]
                if la != sl and not adjacent(la, sl):
                    si.append(i)
                    sj.append(k)
        self.spair_i, self.spair_k = np.array(si, int), np.array(sj, int)
        # union AABB of all static primitives (cylinders by their bounding box)
        lo_all = np.where(self.st_kind[:, None] == 0, self.st_lo,
                          np.column_stack([self.st_xy - self.st_r[:, None], self.st_z[:, 0]]))
        hi_all = np.where(self.st_kind[:, None] == 0, self.st_hi,
                          np.column_stack([self.st_xy + self.st_r[:, None], self.st_z[:, 1]]))
        self._base_c = (lo_all.min(0) + hi_all.max(0)) / 2
        self._base_h = (hi_all.max(0) - lo_all.min(0)) / 2
        self._sp_box = np.nonzero(self.st_kind[self.spair_k] == 0)[0]
        self._sp_cyl = np.nonzero(self.st_kind[self.spair_k] == 1)[0]
        # table: every capsule except the yaw bracket's (it cannot reach the table)
        self.table_caps = np.nonzero(self.cap_link != LINKS.index("shoulder"))[0]

    # ------------------------------------------------------------------ kinematics
    def link_frames(self, Q) -> tuple[np.ndarray, np.ndarray]:
        """World rotation (B,7,3,3) and origin (B,7,3) of every link in :data:`LINKS`.

        ``Q`` is (B,5) or (B,6) (gripper last, rad). Without a gripper value the jaws are placed
        fully open."""
        Q = np.atleast_2d(np.asarray(Q, float))
        B = Q.shape[0]
        R = np.empty((B, len(LINKS), 3, 3))
        t = np.empty((B, len(LINKS), 3))
        Rc = None
        tc = np.zeros((B, 3))
        for k, j in enumerate(self.model.joints):
            Rj = _rot_batch(j.axis, Q[:, k])
            if Rc is None:  # base frame is the identity
                tc = tc + j.origin
                Rc = Rj
            else:
                tc = tc + Rc @ j.origin
                Rc = np.matmul(Rc, Rj)
            R[:, k], t[:, k] = Rc, tc
        g = LINKS.index("gripper")
        qg = Q[:, 5] if Q.shape[1] > 5 else np.full(B, self.model.gripper.limits[1])
        s = self.jaw_shift * np.clip(qg, *self.model.gripper.limits)
        ygrip = R[:, g, :, 1]
        R[:, g + 1] = R[:, g]
        R[:, g + 2] = R[:, g]
        t[:, g + 1] = t[:, g] + ygrip * s[:, None]
        t[:, g + 2] = t[:, g] - ygrip * s[:, None]
        return R, t

    def capsules_world(self, Q) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """World endpoints (B,C,3), (B,C,3) and bounding-sphere centres (B,C,3)."""
        return self._capsules(*self.link_frames(Q))

    def _capsules(self, R, t):
        # one matmul of every link rotation with every capsule point, then pick each capsule's
        # own link: X[b, l, :, c] = R[b, l] @ P[c]
        B, nl = R.shape[:2]
        C = len(self.cap_link)
        X = np.matmul(R.reshape(B, nl * 3, 3), self._cap_pts_T).reshape(B, nl, 3, 3 * C)
        W = X[:, self._cap_pick_l, :, self._cap_pick_c].transpose(1, 0, 2) + t[:, self._cap_pick_l]
        return W[:, :C], W[:, C : 2 * C], W[:, 2 * C :]

    # ------------------------------------------------------------------ distances
    def clearance(self, Q, *, details: bool = False) -> Clearance:
        """Signed clearance (m) of the worst check for each pose in ``Q`` (B,5|6).

        Values below ``exact_below`` are exact; larger ones are lower bounds (bounding spheres).
        With ``details`` every failing check is listed per pose."""
        Q = np.atleast_2d(np.asarray(Q, float))
        B = Q.shape[0]
        R, t = self.link_frames(Q)
        A, Bp, M = self._capsules(R, t)
        C = A.shape[1]
        Af, Bf, Mf = A.reshape(-1, 3), Bp.reshape(-1, 3), M.reshape(-1, 3)
        cols = []

        # 1. link-link. Capsule bounding spheres as a lower bound (dense per link-pair block, no
        #    gathers), exact segment distance where it is small.
        V = np.full((B, len(self.pair_i)), np.inf)
        if self.check_self and len(self.pair_i):
            for (s0, s1, sa, sb), brad in zip(self._blocks, self._block_brad):
                d = M[:, sa, None, :] - M[:, None, sb, :]
                V[:, s0:s1] = (np.sqrt(np.einsum("bxyi,bxyi->bxy", d, d)) - brad).reshape(B, -1)
            V -= self.margin
            flat = np.flatnonzero(V < self.exact_below)  # flat (pose, pair) indices
            if len(flat):
                bb, kk = np.divmod(flat, len(self.pair_i))
                fi, fj = bb * C + self.pair_i[kk], bb * C + self.pair_j[kk]  # flat (pose, capsule)
                V.reshape(-1)[flat] = (
                    _seg_seg_dist(Af[fi], Bf[fi], Af[fj], Bf[fj]) - self._pair_r[kk] - self.margin
                )
        cols.append(V)

        # 2. link-base. Coarse: capsule bounding sphere vs the AABB around all base primitives;
        #    fine: sphere vs each primitive; then the bracketing search over the segment (boxes
        #    and cylinders separately so each evaluates only its own distance function).
        V = np.full((B, len(self.spair_i)), np.inf)
        if self.check_base and len(self.spair_i):
            g = _sd_box_c(M[..., 0], M[..., 1], M[..., 2], self._base_c, self._base_h) - self.cap_brad - self.margin
            V[:] = g[:, self.spair_i]
            for sel, box in ((self._sp_box, True), (self._sp_cyl, False)):
                if not len(sel):
                    continue
                bb, kk = np.nonzero(V[:, sel] < self.exact_below)
                if not len(bb):
                    continue
                kk = sel[kk]
                i, k = self.spair_i[kk], self.spair_k[kk]
                fi = bb * C + i
                Mc = Mf[fi]
                if box:
                    lb = _sd_box_c(Mc[:, 0], Mc[:, 1], Mc[:, 2], self.st_c[k], self.st_h[k])
                else:
                    lb = _sd_zcyl_c(Mc[:, 0], Mc[:, 1], Mc[:, 2], self.st_xy[k], self.st_r[k], self.st_zc[k], self.st_hz[k])
                lb = lb - self.cap_brad[i] - self.margin
                V[bb, kk] = lb
                near = lb < self.exact_below
                bb, kk, i, k, fi = bb[near], kk[near], i[near], k[near], fi[near]
                if len(bb):
                    V[bb, kk] = self._seg_static(Af[fi], Bf[fi], k, box) - self.cap_r[i] - self.margin
        cols.append(V)

        # 3. table
        tc = self.table_caps
        if self.check_table:
            cols.append(np.minimum(A[:, tc, 2], Bp[:, tc, 2]) - self.cap_r[tc] - self.table_z - self.table_margin)
        else:
            cols.append(np.full((B, len(tc)), np.inf))

        V = np.concatenate(cols, axis=1)
        arg = V.argmin(axis=1)
        best = V[np.arange(B), arg]
        depth = -np.minimum(V, 0.0).sum(axis=1)
        hits: list[list[tuple[str, float]]] = [[] for _ in range(B)]
        if details:
            for b_, c_ in zip(*np.nonzero(V < 0.0)):
                hits[b_].append((self._code_name(int(c_)), float(V[b_, c_])))
        return Clearance(best, arg, depth, hits, self)

    def _seg_static(self, a, b, k, box: bool):
        """Lower bound on the signed distance from segments [a,b] (N,3) to static boxes (or
        vertical cylinders) k (N,).

        The signed distance to a convex solid is convex and 1-Lipschitz along a segment, so the
        minimum is bracketed by successive grids: each pass samples K points in the current
        bracket and keeps the two cells around the best sample. After the last pass the true
        minimum is at most half a grid spacing below the best sample (Lipschitz), and that
        bound is subtracted, so the result never overestimates the clearance. Three passes of
        12 points: spacing 4/11^3 = 3e-3 of the segment length (bound < 0.15 mm here)."""
        d = b - a
        L = np.linalg.norm(d, axis=1)
        if box:
            c, h = self.st_c[k][:, None], self.st_h[k][:, None]

            def f(t):
                p = a[:, None, :] + d[:, None, :] * t[..., None]
                return _sd_box_c(p[..., 0], p[..., 1], p[..., 2], c, h)
        else:
            xy, r = self.st_xy[k][:, None], self.st_r[k][:, None]
            zc, hz = self.st_zc[k][:, None], self.st_hz[k][:, None]

            def f(t):
                p = a[:, None, :] + d[:, None, :] * t[..., None]
                return _sd_zcyl_c(p[..., 0], p[..., 1], p[..., 2], xy, r, zc, hz)

        n = len(k)
        rows = np.arange(n)
        lo, width = np.zeros(n), np.ones(n)
        for _ in range(_GRID_PASSES):
            t = lo[:, None] + width[:, None] * _GRID
            v = f(t)
            kbest = v.argmin(axis=1)
            best = v[rows, kbest]
            tb = t[rows, kbest]
            step = width * (1.0 / (_GRID_K - 1))
            lo = np.maximum(tb - step, 0.0)
            width = np.minimum(tb + step, 1.0) - lo
        return best - 0.5 * step * L

    def _code_name(self, c: int) -> str:
        n_pair, n_st = len(self.pair_i), len(self.spair_i)
        if c < n_pair:
            return f"{self.cap_names[self.pair_i[c]]} <-> {self.cap_names[self.pair_j[c]]}"
        c -= n_pair
        if c < n_st:
            return f"{self.cap_names[self.spair_i[c]]} <-> base.{self.static_names[self.spair_k[c]]}"
        c -= n_st
        return f"{self.cap_names[self.table_caps[c]]} <-> table"

    # ------------------------------------------------------------------ convenience
    def check(self, q) -> Clearance:
        """Single pose, with the list of failing checks."""
        return self.clearance(np.asarray(q, float)[None], details=True)

    def in_collision(self, Q) -> np.ndarray:
        return ~self.clearance(Q).ok

    def first_contact(self, q0, q1, steps: int = 2000, **kw) -> tuple[float | None, str]:
        """Sweep the straight joint-space line q0->q1; return (fraction, check) of the first
        sample in collision (None if none). Used to reproduce CAD sweep findings."""
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        s = np.linspace(0.0, 1.0, steps + 1)
        Q = q0 + (q1 - q0) * s[:, None]
        c = self.clearance(Q)
        bad = np.nonzero(~c.ok)[0]
        if not len(bad):
            return None, ""
        return float(s[bad[0]]), c.worst[bad[0]]
