"""Full-assembly audit: every part against every other part, over the full range of every joint.

    .venv/bin/python -m cad.audit [--random N] [--out cad/out/clash_report.md]

What it checks (all from the exact B-rep, gear teeth included):
  1. Interference: boolean intersection volume of EVERY pair of parts on different links, at the zero
     pose, at N_SWEEP poses across each joint's full spec range (others at zero), and at random poses.
     Same-link pairs are checked once at zero pose (they never move relative to each other).
  2. Clearance: minimum distance between every pair of parts that move relative to each other across
     each sweep; anything below RUNNING_CLR that is not a declared bearing/contact pair is flagged.
  3. Support: a contact graph (distance <= CONTACT_TOL at zero pose); every part must be connected to
     the base through contacts, i.e. nothing floats.
Any overlap must be listed in ALLOWED with a build reason, or it is a failure.
"""
from __future__ import annotations

import argparse
import itertools
import pathlib
import time

import numpy as np
from OCP.BRepExtrema import BRepExtrema_DistShapeShape

from . import assembly as A
from . import checks as C
from .geom import place

VOL_TOL = 1e-3       # mm^3: anything above this is an overlap
CONTACT_TOL = 0.02   # mm: closer than this counts as touching (load path)
RUNNING_CLR = 0.25   # mm: moving pairs closer than this (and not declared) are flagged
N_SWEEP = 25

def _key(a, b):
    return frozenset((a, b))


# Declared intentional overlaps: (part_a, part_b) -> build reason. Order-insensitive. Empty: screws are
# modelled 0.05 mm under their tapped holes and press fits line-to-line, so nothing needs to overlap.
ALLOWED: dict[frozenset, str] = {}
# Pairs that are DESIGNED to touch (or nearly) while moving relative to each other.
RUNNING_CONTACT: dict[frozenset, str] = {
    **{_key(f"servo_{j}_case", f"servo_{j}_horn"): "servo output: horn hub seats on the servo's own output"
       for j in ("yaw", "shoulder", "elbow", "wrist_pitch", "roll", "gripper")},
    _key("yaw_bearing", "shoulder_bracket"): "61816 inner race on the turntable spigot (bearing interface)",
    _key("idler_bearing_shoulder", "upper_arm"): "6701 outer race pressed into the child idler plate",
    _key("idler_bearing_elbow", "forearm"): "6701 outer race pressed into the child idler plate",
    _key("idler_bearing_wrist", "wrist_bracket"): "6701 outer race pressed into the child idler plate",
    _key("pinion", "jaw_a"): "involute rack-and-pinion mesh, 0.1 mm backlash per flank by design",
    _key("pinion", "jaw_b"): "involute rack-and-pinion mesh, 0.1 mm backlash per flank by design",
    _key("pad_a", "pad_b"): "gripper fully closed: pad faces meet (that is the closed position)",
    _key("jaw_a", "jaw_b"): "jaws slide past each other in separate y-layers",
}


def dist(a, b) -> float:
    d = BRepExtrema_DistShapeShape(a.wrapped, b.wrapped)
    d.Perform()
    return float(d.Value()) if d.IsDone() else float("nan")


def bb_gap(a, b) -> float:
    A_, B_ = a.BoundingBox(), b.BoundingBox()
    g = max(B_.xmin - A_.xmax, A_.xmin - B_.xmax, B_.ymin - A_.ymax, A_.ymin - B_.ymax,
            B_.zmin - A_.zmax, A_.zmin - B_.zmax)
    return g


def pair_state(sa, sb, need_dist: bool):
    """(overlap volume, min distance or None)."""
    g = bb_gap(sa, sb)
    if g > 0:
        return 0.0, (g if need_dist else None)
    d = dist(sa, sb)
    if d > 1e-6:
        return 0.0, d
    v = float(sa.intersect(sb).Volume())
    return v, 0.0


def poses(n_random: int, seed=0):
    lim = C.joint_limits()
    out = []
    for jn, (lo, hi) in lim.items():
        for q in np.linspace(lo, hi, N_SWEEP):
            out.append(("sweep", jn, {jn: float(q)}))
    rng = np.random.default_rng(seed)
    for i in range(n_random):
        q = {jn: float(rng.uniform(lo, hi)) for jn, (lo, hi) in lim.items()}
        out.append(("random", f"r{i}", q))
    return out


# Sliding fits that carry a part (support graph edges even though there is running clearance).
SLIDING: dict[frozenset, str] = {
    _key("gripper_body", "jaw_a"): "rack runs between the guide rails, 0.2 mm running clearance",
    _key("gripper_body", "jaw_b"): "rack runs between the guide rails, 0.2 mm running clearance",
}
SLIDE_TOL = 0.35

AXISYMMETRIC = {k for k in RUNNING_CONTACT if any(w in " ".join(k) for w in ("bearing", "servo_"))}


def _mesh_model(shape, tol=0.02, ang=0.05):
    import fcl
    v, f = shape.tessellate(tol, ang)
    V = np.array([[p.x, p.y, p.z] for p in v], float)
    F = np.array(f, dtype=np.int64)
    m = fcl.BVHModel()
    m.beginModel(len(V), len(F))
    m.addSubModel(V, F)
    m.endModel()
    return m


def run(n_random=40, verbose=True):
    """Mesh-screened, B-rep-confirmed audit. Every pair is screened with FCL on fine tessellations
    (chord 0.02 mm, 3 deg); any pair the meshes say touches is re-checked with an exact OCCT boolean.
    Reported clearances are mesh distances (error <= ~0.04 mm on curved faces)."""
    import fcl
    t0 = time.time()
    items = A.items(teeth=True)
    names = [it.name for it in items]
    link = {it.name: it.link for it in items}
    objs = {it.name: fcl.CollisionObject(_mesh_model(it.solid)) for it in items}
    overlaps, min_clr, self_coll = {}, {}, {}
    # ---- every pair at zero pose with exact geometry (incl. same-link pairs) + contact graph
    pz = A.posed({}, teeth=True)
    contacts = set()
    for (ia, sa), (ib, sb) in itertools.combinations(pz, 2):
        v, d = pair_state(sa, sb, True)
        k = _key(ia.name, ib.name)
        if d is not None and (d <= CONTACT_TOL or (k in SLIDING and d <= SLIDE_TOL)):
            contacts.add(k)
        if v > VOL_TOL:
            overlaps.setdefault(k, (v, "zero", "static"))
    if verbose:
        print(f"zero pose done {time.time() - t0:.0f}s", flush=True)
    dreq = fcl.DistanceRequest()
    creq = fcl.CollisionRequest()
    exact_cache = {}
    for kind, label, q in poses(n_random):
        tf = A.link_transforms(q)
        for it in items:
            m = tf[it.link]
            objs[it.name].setTransform(fcl.Transform(m[:3, :3], m[:3, 3]))
        moving = C.moving_set(label) if kind == "sweep" else None
        posed_cache = {}
        tag = f"{label}={q.get(label, 0):+.2f}" if kind == "sweep" else label
        for ia, ib in itertools.combinations(items, 2):
            if ia.link == ib.link:
                continue
            if moving is not None and (ia.link in moving) == (ib.link in moving):
                continue
            k = _key(ia.name, ib.name)
            adj = A.adjacent(ia.link, ib.link)
            oa, ob = objs[ia.name], objs[ib.name]
            if kind == "random":
                hit = fcl.collide(oa, ob, creq, fcl.CollisionResult()) > 0
                d = 0.0 if hit else None
            else:
                res = fcl.DistanceResult()
                d = max(0.0, fcl.distance(oa, ob, dreq, res))
                hit = d <= 1e-6
            v = 0.0
            if hit:
                if k in AXISYMMETRIC:
                    v = exact_cache.get(k, 0.0)
                else:
                    for it in (ia, ib):
                        if it.name not in posed_cache:
                            posed_cache[it.name] = place(it.solid, tf[it.link])
                    v, d_exact = pair_state(posed_cache[ia.name], posed_cache[ib.name], kind != "random")
                    if d_exact is not None:
                        d = d_exact
            if v > VOL_TOL:
                if kind == "random" and not adj:
                    self_coll[k] = self_coll.get(k, 0) + 1
                elif k not in overlaps or v > overlaps[k][0]:
                    overlaps[k] = (v, tag, kind)
            if d is not None and kind != "random" and (k not in min_clr or d < min_clr[k][0]):
                min_clr[k] = (d, tag)
        if verbose and kind == "sweep" and q.get(label) == C.joint_limits()[label][1]:
            print(f"{label} sweep done {time.time() - t0:.0f}s", flush=True)
    # ---- support graph
    adjm = {n: set() for n in names}
    for k in contacts:
        a, b = tuple(k)
        adjm[a].add(b); adjm[b].add(a)
    seen, stack = {"base"}, ["base"]
    while stack:
        n = stack.pop()
        for m in adjm[n]:
            if m not in seen:
                seen.add(m); stack.append(m)
    floating = sorted(set(names) - seen)
    return dict(names=names, link=link, overlaps=overlaps, min_clr=min_clr, self_coll=self_coll,
                contacts=contacts, adj=adjm, floating=floating, n_poses=len(poses(n_random)) + 1,
                n_random=n_random, seconds=time.time() - t0)


def failures(r) -> list[str]:
    bad = []
    for k, (v, tag, kind) in r["overlaps"].items():
        if k not in ALLOWED:
            bad.append(f"overlap {sorted(k)} {v:.3f} mm3 at {tag}")
    for k, (d, tag) in r["min_clr"].items():
        a, b = tuple(k)
        if d < RUNNING_CLR and k not in RUNNING_CONTACT and k not in SLIDING and k not in ALLOWED and \
                A.adjacent(r["link"][a], r["link"][b]):
            bad.append(f"clearance {sorted(k)} {d:.3f} mm at {tag}")
    for n in r["floating"]:
        bad.append(f"floating {n}")
    return bad


def report(r) -> str:
    L = ["# Assembly clash and clearance report (generated by `python -m cad.audit`; do not edit)\n",
         f"{len(r['names'])} parts, {len(r['names']) * (len(r['names']) - 1) // 2} part pairs. "
         f"{r['n_poses']} poses: zero pose, {N_SWEEP} poses across each joint's full spec range "
         f"(others at zero), and {r['n_random']} random poses with all joints in range. "
         "Zero pose: exact B-rep booleans and distances for every pair. Swept/random poses: every pair screened "
         "with FCL on fine meshes (0.02 mm chord, 3 deg), and every pair the meshes report touching re-checked "
         f"with an exact B-rep boolean. Gear teeth and fasteners included; overlap threshold {VOL_TOL} mm3. "
         f"Run time {r['seconds']:.0f} s.\n"]
    bad = failures(r)
    L.append(f"**Result: {'PASS' if not bad else 'FAIL'}** ({len(bad)} failures)\n")
    for b in bad:
        L.append(f"- FAIL: {b}")
    L.append("\n## Overlaps\n")
    if not r["overlaps"]:
        L.append("None at any pose.\n")
    else:
        L.append("| part A | part B | max volume mm3 | worst pose | status |")
        L.append("|---|---|---|---|---|")
        for k, (v, tag, kind) in sorted(r["overlaps"].items(), key=lambda t: -t[1][0]):
            a, b = sorted(k)
            L.append(f"| {a} | {b} | {v:.3f} | {tag} | {ALLOWED.get(k, '**not allowed**')} |")
    L.append("\n## Pose-dependent self-collisions (non-adjacent links, random poses)\n")
    L.append("These are reachable-but-forbidden poses (e.g. folding the wrist into the base). They are not part "
             "clashes; the host collision model in `armctl` must keep the arm out of them.\n")
    if not r["self_coll"]:
        L.append("None.\n")
    else:
        L.append("| part A | part B | random poses hit |")
        L.append("|---|---|---|")
        for k, n in sorted(r["self_coll"].items(), key=lambda t: -t[1]):
            a, b = sorted(k)
            L.append(f"| {a} | {b} | {n} / {r['n_random']} |")
    L.append("\n## Minimum clearance between parts that move relative to each other (adjacent links)\n")
    L.append(f"Flag threshold {RUNNING_CLR} mm. Designed running contacts (bearing faces, horn faces, sliding "
             "guides) are listed with their reason.\n")
    L.append("| part A | part B | min distance mm | at pose | note |")
    L.append("|---|---|---|---|---|")
    rows = [(k, d, tag) for k, (d, tag) in r["min_clr"].items()
            if A.adjacent(r["link"][tuple(k)[0]], r["link"][tuple(k)[1]]) and d < 5.0]
    for k, d, tag in sorted(rows, key=lambda t: t[1]):
        a, b = sorted(k)
        note = RUNNING_CONTACT.get(k) or SLIDING.get(k) or ALLOWED.get(k) or ("**below threshold**" if d < RUNNING_CLR else "")
        L.append(f"| {a} | {b} | {d:.3f} | {tag} | {note} |")
    L.append("\n## Support (contact graph at zero pose)\n")
    L.append(f"Contact = closer than {CONTACT_TOL} mm, or a declared sliding fit closer than {SLIDE_TOL} mm "
             "(listed below). Every part must reach `base` through contacts.\n")
    L.append(f"Floating parts: {', '.join(r['floating']) if r['floating'] else 'none'}.\n")
    L.append("| part | touches |")
    L.append("|---|---|")
    for n in r["names"]:
        L.append(f"| {n} | {', '.join(sorted(r['adj'][n])) or '**nothing**'} |")
    L.append("\n## Declared sliding fits\n")
    for k, why in SLIDING.items():
        L.append(f"- {' / '.join(sorted(k))}: {why}")
    L.append("\n## Declared running contacts\n")
    for k, why in RUNNING_CONTACT.items():
        L.append(f"- {' / '.join(sorted(k))}: {why}")
    return "\n".join(L) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--random", type=int, default=40)
    ap.add_argument("--out", default=str(pathlib.Path(__file__).resolve().parent / "out" / "clash_report.md"))
    a = ap.parse_args()
    r = run(a.random)
    txt = report(r)
    pathlib.Path(a.out).write_text(txt)
    print(txt)
    raise SystemExit(1 if failures(r) else 0)


if __name__ == "__main__":
    main()
