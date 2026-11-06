"""Geometric checks on the generated B-rep: validity, measured joint axes, wall thickness, bed fit,
mass estimate, and collision-free joint ranges. Used by tests and by cad.build to write the report."""
from __future__ import annotations

import math

import cadquery as cq
import numpy as np
from OCP.BRep import BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface

from spec_loader import load_spec
from . import assembly as A
from . import params as P
from . import parts as PT


# ------------------------------------------------------------------ validity
def solid_report(s: cq.Shape) -> dict:
    sols = s.Solids()
    shells = [sh for so in sols for sh in so.Shells()]
    return dict(
        n_solids=len(sols),
        valid=bool(s.isValid()),
        closed=all(BRep_Tool.IsClosed_s(sh.wrapped) for sh in shells) and len(shells) > 0,
        volume=float(s.Volume()),
    )


# ------------------------------------------------------------------ measured features
def cylinders(s: cq.Shape, radius: float, axis, tol=1e-3):
    """Axis points of cylindrical faces with the given radius and axis direction."""
    ax = np.asarray(axis, float)
    out = []
    for f in s.Faces():
        if f.geomType() != "CYLINDER":
            continue
        cy = BRepAdaptor_Surface(f.wrapped).Cylinder()
        if abs(cy.Radius() - radius) > tol:
            continue
        d = cy.Axis().Direction(); d = np.array([d.X(), d.Y(), d.Z()])
        if abs(abs(d @ ax) - 1) > 1e-6:
            continue
        loc = cy.Axis().Location()
        out.append(np.array([loc.X(), loc.Y(), loc.Z()]))
    return out


def planes(s: cq.Shape, normal):
    """(offset along normal, centre) of planar faces whose outward normal equals `normal`."""
    n = np.asarray(normal, float)
    out = []
    for f in s.Faces():
        if f.geomType() != "PLANE":
            continue
        c = f.Center(); fn = f.normalAt(c)
        if abs(np.array([fn.x, fn.y, fn.z]) @ n - 1) < 1e-6:
            cc = np.array([c.x, c.y, c.z])
            out.append((float(cc @ n), cc))
    return out


def _axis_xz(points, near_x, near_z, tol=2.0):
    """Pick the axis (projected to the x-z plane) closest to the expected location."""
    best = min(points, key=lambda p: (p[0] - near_x) ** 2 + (p[2] - near_z) ** 2)
    assert abs(best[0] - near_x) < tol + 20 and abs(best[2] - near_z) < tol + 20
    return np.array([best[0], best[2]])


def measured_dimensions() -> dict[str, dict]:
    """Joint-axis distances measured from the printed parts' own geometry (not from parameters).

    Each entry: measured value (mm) and the spec value it must equal."""
    spec = load_spec()
    sp = {j["name"]: [v * 1000 for v in j["origin"]] for j in spec["joints"]}
    out = {}

    # base: servo pocket floor (case bottom) + servo stack = yaw horn face height
    b = PT.base()
    floors = [z for z, c in planes(b, (0, 0, 1)) if abs(c[0]) < 20 and abs(c[1]) < 10 and z > 0.5]
    out["base: plate -> yaw horn face"] = (min(floors) + P.SERVO.stack, sp["base_yaw"][2])

    # shoulder bracket: turntable underside (z=0) -> idler stub axis height
    sb = PT.shoulder_bracket()
    stub = cylinders(sb, P.STUB_R, (0, 1, 0))
    # horn mating face = highest downward-facing face of the turntable region (z < 5)
    under = max(-off for off, c in planes(sb, (0, 0, -1)) if np.hypot(c[0], c[1]) < 5 and -off < 5)
    out["shoulder bracket: yaw horn face -> shoulder axis"] = (stub[0][2] - under, sp["shoulder_pitch"][2])

    for name, builder, nxt in (("upper_arm", PT.upper_arm, "elbow_pitch"), ("forearm", PT.forearm, "wrist_pitch")):
        s = builder()
        bore = _axis_xz(cylinders(s, P.IDLER_BORE_R, (0, 1, 0)), 0, 0)
        stub = _axis_xz(cylinders(s, P.STUB_R, (0, 1, 0)), sp[nxt][0], 0)
        out[f"{name}: proximal bore axis -> distal stub axis"] = (float(np.linalg.norm(stub - bore)), sp[nxt][0])

    wb = PT.wrist_bracket()
    bore = _axis_xz(cylinders(wb, P.IDLER_BORE_R, (0, 1, 0)), 0, 0)
    front = max(z for z, c in planes(wb, (1, 0, 0)))
    out["wrist bracket: pitch axis -> roll horn face"] = (front - bore[0], sp["wrist_roll"][0])

    # gripper: back-wall rear face minus servo stack = roll horn face; pads' centre = TCP
    gb = PT.gripper_body()
    # end face of the roll-servo pocket (the case bottom seats here), centred on the roll axis
    seat = [c for _, c in planes(gb, (-1, 0, 0)) if abs(c[1]) < 1 and abs(c[2]) < 6 and 20 < c[0] < 40]
    horn_face = seat[0][0] - P.SERVO.stack
    pa = PT.pad(+1)
    inner = [c for y, c in planes(pa, (0, -1, 0))][0]
    out["gripper: roll horn face -> TCP (pad centre)"] = (inner[0] - horn_face, spec["tool"]["tcp"][0] * 1000)
    return out


def assembled_axes(q=None) -> dict[str, np.ndarray]:
    """World positions of joint axes computed by placing each part's MEASURED features with FK."""
    tf = A.link_transforms(q)
    pts = {}
    sb = PT.shoulder_bracket()
    pts["shoulder_pitch"] = (tf["shoulder"] @ np.append(cylinders(sb, P.STUB_R, (0, 1, 0))[0], 1))[:3]
    ua = PT.upper_arm()
    c = cylinders(ua, P.STUB_R, (0, 1, 0))[0]
    pts["elbow_pitch"] = (tf["upper_arm"] @ np.append(c, 1))[:3]
    fa = PT.forearm()
    c = cylinders(fa, P.STUB_R, (0, 1, 0))[0]
    pts["wrist_pitch"] = (tf["forearm"] @ np.append(c, 1))[:3]
    wb = PT.wrist_bracket()
    front = max(z for z, _ in planes(wb, (1, 0, 0)))
    pts["wrist_roll"] = (tf["wrist"] @ np.array([front, 0, 0, 1]))[:3]
    pa = PT.pad(+1)
    inner = [c for y, c in planes(pa, (0, -1, 0))][0]
    pts["tcp"] = (tf["jaw_a"] @ np.append(inner, 1))[:3]
    # ignore the y coordinate of the pitch stubs (they sit on the idler side of the joint)
    for k in ("shoulder_pitch", "elbow_pitch", "wrist_pitch"):
        pts[k] = pts[k].copy()
    return pts


# ------------------------------------------------------------------ wall thickness (ray casting)
def _mesh(s: cq.Shape, tol=0.05):
    v, t = s.tessellate(tol, 0.2)
    V = np.array([[p.x, p.y, p.z] for p in v]); F = np.array(t, dtype=int)
    return V, F


def wall_thickness(s: cq.Shape, n_samples=3000, seed=0) -> dict:
    """Local wall thickness by casting a ray from surface samples along the inward normal and
    measuring the distance to the next boundary crossing. Returns min / 1st percentile (mm)."""
    V, F = _mesh(s)
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    n = np.cross(b - a, c - a)
    area = np.linalg.norm(n, axis=1) / 2
    ok = area > 1e-6
    a, b, c, n, area = a[ok], b[ok], c[ok], n[ok], area[ok]
    n = n / (2 * area[:, None])
    # orientation sanity: signed volume must be positive for outward normals
    signed = np.sum(np.einsum("ij,ij->i", a, np.cross(b, c))) / 6
    if signed < 0:
        n = -n
    rng = np.random.default_rng(seed)
    idx = rng.choice(len(a), size=min(n_samples, len(a)), replace=False, p=area / area.sum())
    # sample a random interior point of each chosen triangle (away from edges)
    r1, r2 = rng.uniform(0.2, 0.6, len(idx)), rng.uniform(0.2, 0.6, len(idx))
    m = r1 + r2 > 0.9
    r1[m], r2[m] = r1[m] * 0.5, r2[m] * 0.5
    o = a[idx] + r1[:, None] * (b[idx] - a[idx]) + r2[:, None] * (c[idx] - a[idx])
    d = -n[idx]
    o = o + d * 1e-4
    e1, e2 = b - a, c - a
    th = np.full(len(idx), np.inf)
    for k0 in range(0, len(idx), 64):
        O, D = o[k0:k0 + 64, None, :], d[k0:k0 + 64, None, :]
        p = np.cross(D, e2[None])
        det = np.einsum("rtk,tk->rt", p, e1)
        good = np.abs(det) > 1e-12
        inv = np.where(good, 1.0 / np.where(good, det, 1), 0)
        tv = O - a[None]
        u = np.einsum("rtk,rtk->rt", tv, p) * inv
        q = np.cross(tv, e1[None])
        v = np.einsum("rtk,rtk->rt", np.broadcast_to(D, q.shape), q) * inv
        t = np.einsum("rtk,tk->rt", q, e2) * inv
        hit = good & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-3)
        t = np.where(hit, t, np.inf)
        th[k0:k0 + 64] = t.min(axis=1)
    fin = np.isfinite(th)
    k = int(np.argmin(np.where(fin, th, np.inf)))
    th_f = th[fin]
    return dict(min=float(th_f.min()), p1=float(np.percentile(th_f, 1)), n=int(len(th_f)),
                where=tuple(round(float(x), 2) for x in o[k]))


# ------------------------------------------------------------------ print bed fit
def fits_bed(s: cq.Shape, bed=P.PRINT_BED) -> tuple[bool, str]:
    bb = s.BoundingBox()
    dims = [bb.xlen, bb.ylen, bb.zlen]
    bx, by, bz = bed
    for h_i in range(3):
        h = dims[h_i]
        if h > bz:
            continue
        l, w = [dims[i] for i in range(3) if i != h_i]
        for deg in np.linspace(0, 90, 181):
            t = math.radians(deg)
            fx = l * math.cos(t) + w * math.sin(t)
            fy = l * math.sin(t) + w * math.cos(t)
            if fx <= bx and fy <= by:
                return True, f"height {h:.1f} mm, footprint {l:.1f} x {w:.1f} rotated {deg:.1f} deg"
    return False, f"bbox {dims}"


# ------------------------------------------------------------------ mass estimate
PURCHASED_G = {
    "servo case (each)": P.SERVO.mass_g,
    "yaw bearing": P.BEARING["mass_g"],
    "wrist camera board (assumed)": 15.0,
}


def printed_mass_g(name: str, s: cq.Shape | None = None) -> float:
    s = s or PT.build_printed(name)
    mat = PT.PRINTED[name][2]
    frac = P.INFILL_EFF if mat == "PETG" else 1.0  # TPU pads printed solid
    return s.Volume() / 1000.0 * P.DENSITY[mat] * frac


SPEC_LINK = {"base": "base_link", "shoulder": "shoulder_link", "upper_arm": "upper_arm",
             "forearm": "forearm", "wrist": "wrist", "gripper": "gripper"}


def link_masses() -> dict[str, dict]:
    """Estimated link masses (g): printed parts + servo cases + purchased parts, mapped onto the
    spec's link names (grip_drive / jaws are lumped into 'gripper')."""
    agg = {k: {"printed": 0.0, "purchased": 0.0} for k in SPEC_LINK}
    lump = lambda l: "gripper" if l in ("grip_drive", "jaw_a", "jaw_b") else l
    for name, (_, link, _) in PT.PRINTED.items():
        agg[lump(link)]["printed"] += printed_mass_g(name)
    for it in A.items():
        if it.printed:
            continue
        if it.name.endswith("_case"):
            agg[lump(it.link)]["purchased"] += P.SERVO.mass_g
        elif it.name == "yaw_bearing":
            agg["base"]["purchased"] += P.BEARING["mass_g"]
    agg["gripper"]["purchased"] += PURCHASED_G["wrist camera board (assumed)"]
    spec = load_spec()["masses_kg"]
    for k, v in agg.items():
        v["total"] = v["printed"] + v["purchased"]
        v["spec"] = spec[SPEC_LINK[k]] * 1000
    return agg


# ------------------------------------------------------------------ joint ranges
def joint_limits() -> dict[str, tuple[float, float]]:
    spec = load_spec()
    lim = {j["name"]: tuple(j["limits"]) for j in spec["joints"]}
    lim["gripper"] = tuple(spec["gripper"]["limits"])
    return lim


CHILD_LINK = {"base_yaw": "shoulder", "shoulder_pitch": "upper_arm", "elbow_pitch": "forearm",
              "wrist_pitch": "wrist", "wrist_roll": "gripper"}


def moving_set(jn: str) -> set[str]:
    if jn == "gripper":
        return {"grip_drive", "jaw_a", "jaw_b"}
    return A.descendants(CHILD_LINK[jn])


def sweep(jn: str, n=13, adjacent_only=True, base_q=None) -> list[tuple[float, list]]:
    lo, hi = joint_limits()[jn]
    out = []
    for q in np.linspace(lo, hi, n):
        qq = dict(base_q or {}); qq[jn] = float(q)
        out.append((float(q), A.collisions(qq, moving_set(jn), adjacent_only=adjacent_only)))
    return out


def free_range(jn: str, adjacent_only=True, step=0.02, base_q=None) -> tuple[float, float]:
    """Largest collision-free interval around 0 (rad) within the spec limits, resolved to ~step."""
    lo, hi = joint_limits()[jn]
    res = []
    for end in (lo, hi):
        def bad(q):
            qq = dict(base_q or {}); qq[jn] = q
            return bool(A.collisions(qq, moving_set(jn), adjacent_only=adjacent_only))
        if not bad(end):
            res.append(end); continue
        good_q, bad_q = 0.0, end
        while abs(bad_q - good_q) > step:
            mid = 0.5 * (good_q + bad_q)
            if bad(mid):
                bad_q = mid
            else:
                good_q = mid
        res.append(good_q)
    return tuple(res)
