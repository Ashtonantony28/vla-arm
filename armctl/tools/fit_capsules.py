"""Fit the capsule radii of armctl's collision model to the CAD point clouds.

    python armctl/tools/fit_capsules.py links.npz        # links.npz from dump_cad_links.py

The capsule *segments* (and a radius hint) are laid out by hand below, in link frames, mm,
chosen from the part dimensions in hardware/cad/params.py and the per-link point-cloud slices.
Every CAD point of a link (tessellation vertices + dense edge samples of every printed part,
servo case, horn and the bearing on that link) is assigned to the capsule minimising
``distance - hint``; each radius is then the maximum distance of its assigned points from the
segment (rounded up to 0.1 mm). The union of a link's capsules therefore encloses every point.
Base points are checked against the static primitives (boxes / vertical cylinders) instead.
Prints YAML for armctl/armctl/data/collision_model.yaml plus residuals (must be 0).
"""
import sys

import numpy as np

# link -> list of (name, p0_mm, p1_mm, hint_mm). Gripper and jaw frames = wrist_roll joint frame
# (jaw frames are additionally shifted by +-PINION_R*q_gripper along y).
YP = (-24.0, 21.0)  # the two clevis side plates (outer layer, y -26..-22 and 19..23); hollow between
LAYOUT = {
    # shoulder bracket above the turntable (frame at the yaw horn face, world z = 40 mm):
    # pedestal + shoulder cradle box x+-25.8, y -21.5..16 (-27 at the horn), z 6..75.6;
    # counterbalance post x+-10, y 26..34, z 6..108. Turntable r=36, z 0..6 -> static cylinder.
    "shoulder": [
        ("ped_n", (-12.9, -2.75, 20.0), (-12.9, -2.75, 62.0), 23.0),
        ("ped_p", (12.9, -2.75, 20.0), (12.9, -2.75, 62.0), 23.0),
        ("post", (0.0, 30.0, 12.0), (0.0, 30.0, 102.0), 11.0),
    ],
    # upper arm: R26 clevis discs at the shoulder axis (capsule along y, exact in-plane);
    # two side plates z+-14 at y -26..-22 / 19..23 to x=108 (hollow between them: the base
    # housing reaches up between the plates); full-width bridge x 108..122; counterbalance pin
    # y 23..35 at x=70; neck z+-11, y -21.5..16 to x=174; elbow cradle x 174..226, z+-15.6, y -27..16
    # (3 capsules along y so the cradle's flat end is not over-extended).
    "upper_arm": [("disc", (0.0, -26.0, 0.0), (0.0, 23.0, 0.0), 26.0)]
    + [(f"plate{i}", (14.0, y, 0.0), (108.0, y, 0.0), 14.2) for i, y in enumerate(YP)]
    + [("bridge", (115.0, -26.0, 0.0), (115.0, 23.0, 0.0), 15.7)]
    + [("cb_pin", (70.0, 29.0, 0.0), (70.0, 29.0, 0.0), 8.0)]
    + [(f"neck{i}", (126.0, y, 0.0), (168.0, y, 0.0), 12.7) for i, y in enumerate((-15.25, -2.75, 9.75))]
    + [(f"cradle{i}", (x, -27.0, 0.0), (x, 16.0, 0.0), 18.0) for i, x in enumerate((182.0, 200.0, 218.0))],
    # forearm: disc at the elbow, two side plates z+-13 to x=126, bridge x 126..140, neck z+-10
    # to 154, wrist-pitch
    # cradle x 154..206, z+-15.6, y -27..16.
    "forearm": [("disc", (0.0, -26.0, 0.0), (0.0, 23.0, 0.0), 26.0)]
    + [(f"plate{i}", (14.0, y, 0.0), (126.0, y, 0.0), 13.2) for i, y in enumerate(YP)]
    + [("bridge", (133.0, -26.0, 0.0), (133.0, 23.0, 0.0), 14.8)]
    + [("neck", (140.0, -5.0, 0.0), (150.0, -5.0, 0.0), 14.0)]
    + [(f"cradle{i}", (x, -27.0, 0.0), (x, 16.0, 0.0), 18.0) for i, x in enumerate((162.0, 180.0, 198.0))],
    # wrist bracket: R26 disc around the wrist-pitch axis, two side plates z+-16 to x=40, front
    # plate x 40..45 full width, roll horn (x 45..48, r 10).
    "wrist": [("disc", (0.0, -26.0, 0.0), (0.0, 23.0, 0.0), 26.0)]
    + [(f"side{i}", (20.0, y, 0.0), (40.0, y, 0.0), 16.2) for i, y in enumerate(YP)]
    + [("front", (42.5, -26.0, 0.0), (42.5, 23.0, 0.0), 16.2)],
    # gripper body (rotates with wrist roll): servo sleeve x 3..38, y+-25.3, z -15..42 (3 rows
    # along y); back plate + rack guides + pinion + gripper horn x 38..49, y+-40, z -15..57.4
    # (4 rows along y); wrist camera mount x 6..34, y+-18, z 42..76 (2 rows).
    "gripper": [(f"sleeve{i}", (20.5, -18.0, z), (20.5, 18.0, z), 21.0) for i, z in enumerate((-5.5, 13.5, 32.6))]
    + [(f"guide{i}", (43.5, -34.0, z), (43.5, 34.0, z), 12.0) for i, z in enumerate((-6.0, 12.0, 30.0, 48.0))]
    + [(f"camera{i}", (20.0, -10.0, z), (20.0, 10.0, z), 18.0) for i, z in enumerate((50.7, 67.7))],
    # jaw A (+y): finger x 49.8..78, y 0..8 (pad + finger), z -12..10 (2 capsules along z, so
    # the fingertip is not over-extended along x); lower rack x 41..49 along y.
    "jaw_a": [
        ("finger0", (56.8, 4.0, -6.0), (56.8, 4.0, 4.0), 10.0),
        ("finger1", (71.0, 4.0, -6.0), (71.0, 4.0, 4.0), 10.0),
        ("rack", (45.0, -30.0, 4.0), (45.0, 4.0, 4.0), 7.0),
    ],
    # jaw B (-y): finger x 49.8..78, y -8..0, z -12..53.8 (carries the upper rack x 41..49).
    "jaw_b": [
        ("finger0", (56.8, -4.0, -6.0), (56.8, -4.0, 48.0), 10.0),
        ("finger1", (71.0, -4.0, -6.0), (71.0, -4.0, 48.0), 10.0),
        ("rack", (45.0, -4.0, 50.0), (45.0, 30.0, 50.0), 7.0),
    ],
}

# Static primitives (world frame, mm). The turntable is on the yaw link but is a cylinder about
# the yaw axis, so it is the same solid for every yaw angle.
STATIC = [
    ("plate", "base", "box", dict(lo=(-70.0, -60.0, 0.0), hi=(70.0, 60.0, 6.0))),
    ("tab_p", "base", "box", dict(lo=(-20.0, 59.0, 0.0), hi=(20.0, 95.0, 8.0))),
    ("tab_n", "base", "box", dict(lo=(-20.0, -95.0, 0.0), hi=(20.0, -59.0, 8.0))),
    ("rib_p", "base", "box", dict(lo=(-2.0, 38.25, 7.99), hi=(2.0, 78.0, 18.0))),
    ("rib_n", "base", "box", dict(lo=(-2.0, -78.0, 7.99), hi=(2.0, -38.25, 18.0))),
    ("housing", "base", "zcyl", dict(xy=(0.0, 0.0), r=42.5, z=(0.0, 39.5))),
    ("turntable", "shoulder", "zcyl", dict(xy=(0.0, 0.0), r=36.0, z=(40.0, 46.0))),
]


def seg_dist(p, a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    L2 = float(d @ d)
    t = np.zeros(len(p)) if L2 == 0 else np.clip((p - a) @ d / L2, 0, 1)
    return np.linalg.norm(p - (a + t[:, None] * d), axis=1)


def inside_static(p, tol=0.05):
    ok = np.zeros(len(p), bool)
    for _, _, kind, g in STATIC:
        if kind == "box":
            ok |= np.all((p >= np.array(g["lo"]) - tol) & (p <= np.array(g["hi"]) + tol), axis=1)
        else:
            r = np.hypot(p[:, 0] - g["xy"][0], p[:, 1] - g["xy"][1])
            ok |= (r <= g["r"] + tol) & (p[:, 2] >= g["z"][0] - tol) & (p[:, 2] <= g["z"][1] + tol)
    return ok


def fit(points_mm, caps):
    D = np.stack([seg_dist(points_mm, a, b) for _, a, b, _ in caps], axis=1)
    hint = np.array([h for *_, h in caps])
    assign = (D - hint).argmin(1)
    r = np.array([D[assign == k, k].max() if np.any(assign == k) else 0.0 for k in range(len(caps))])
    r = np.ceil(r * 10) / 10
    outside = int(np.sum(np.all(D > r + 1e-9, axis=1)))
    return r, outside


def main(path):
    d = np.load(path)
    base = d["base"] * 1000.0
    print(f"# base: {len(base)} points, {int(np.sum(~inside_static(base)))} outside the static primitives")
    print("links:")
    for link, caps in LAYOUT.items():
        pts = d[link] * 1000.0
        if link == "gripper":
            pts = np.vstack([pts, d["grip_drive"] * 1000.0])
        if link == "shoulder":  # turntable + spigot -> static cylinder (world z = 40 + local z)
            sh = pts + np.array([0.0, 0.0, 40.0])
            pts = pts[~inside_static(sh) & (pts[:, 2] > 0.0)]
        r, outside = fit(pts, caps)
        print(f"  {link}:   # {len(pts)} CAD points, {outside} outside")
        for (name, a, b, h), rk in zip(caps, r):
            a = [round(x / 1000, 5) for x in a]
            b = [round(x / 1000, 5) for x in b]
            print(f"    - {{name: {name}, p0: {a}, p1: {b}, r: {rk / 1000:.4f}}}   # hint {h}")


if __name__ == "__main__":
    main(sys.argv[1])
