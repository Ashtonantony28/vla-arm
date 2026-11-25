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
YP = (-25.75, 21.75)  # the two clevis side plates (outer layer, y -27.75..-23.75 and 19.75..23.75); hollow between
LAYOUT = {
    # shoulder bracket above the turntable (frame at the turntable underside, world z = 40 mm):
    # pedestal + shoulder cradle box x -38.3..13.3 (servo case offset 12.5 mm behind its output),
    # y -27.8..15.3, z 7..75.6; counterbalance post x+-10 (gussets to +-22), y 27.5..37.5, z 7..110.
    # Turntable r=46, z 0..7 -> static cylinder.
    "shoulder": [
        *[(f"ped{i}", (x, y, 19.0), (x, y, 64.0), 14.0)
          for i, (x, y) in enumerate(((-26.0, -16.0), (-26.0, 4.0), (1.0, -16.0), (1.0, 4.0)))],
        ("post", (0.0, 31.5, 14.0), (0.0, 31.5, 100.0), 11.0),
        ("gus_p", (20.0, 31.5, 10.5), (10.0, 31.5, 29.0), 5.0),
        ("gus_n", (-20.0, 31.5, 10.5), (-10.0, 31.5, 29.0), 5.0),
    ],
    # upper arm: R26 clevis disc at the shoulder axis (capsule along y); two tapered side plates in
    # the outer layers to x=122 (hollow between them); bridge x 122..136; counterbalance pin
    # y 23.75..37 at x=70; neck to the elbow cradle x 161.7..213.3 (3 capsules along y).
    "upper_arm": [("disc", (0.0, -27.8, 0.0), (0.0, 23.8, 0.0), 26.0)]
    + [(f"root{i}", (14.0, y, 0.0), (40.0, y, 0.0), 21.0) for i, y in enumerate(YP)]
    + [(f"plate{i}", (40.0, y, 0.0), (112.0, y, 0.0), 14.0) for i, y in enumerate(YP)]
    + [("cb_pin", (70.0, 26.0, 0.0), (70.0, 33.0, 0.0), 7.0)]
    + [(f"neck{i}", (112.0, y, 0.0), (158.0, y, 0.0), 13.0) for i, y in enumerate((-22.0, -9.0, 4.0, 17.0))]
    + [(f"cradle{i}", (x, -27.8, 0.0), (x, 16.0, 0.0), 18.0) for i, x in enumerate((170.0, 187.5, 205.0))],
    # forearm: disc at the elbow, two side plates to x=130, bridge, wrist-pitch cradle x 141.7..193.3.
    "forearm": [("disc", (0.0, -27.8, 0.0), (0.0, 23.8, 0.0), 26.0)]
    + [(f"root{i}", (14.0, y, 0.0), (40.0, y, 0.0), 21.0) for i, y in enumerate(YP)]
    + [(f"plate{i}", (40.0, y, 0.0), (118.0, y, 0.0), 13.0) for i, y in enumerate(YP)]
    + [(f"neck{i}", (118.0, y, 0.0), (140.0, y, 0.0), 13.0) for i, y in enumerate((-22.0, -9.0, 4.0, 17.0))]
    + [(f"cradle{i}", (x, -27.8, 0.0), (x, 16.0, 0.0), 18.0) for i, x in enumerate((150.0, 167.5, 185.0))],
    # wrist bracket: R26 disc around the wrist-pitch axis, two side plates to x=40, front plate
    # x 40..45 full width, roll horn + screws (x 37..49.5, r 10).
    "wrist": [("disc", (0.0, -27.8, 0.0), (0.0, 23.8, 0.0), 26.0)]
    + [(f"side{i}", (20.0, y, 0.0), (40.0, y, 0.0), 16.0) for i, y in enumerate(YP)]
    + [("front", (43.0, -27.8, 0.0), (43.0, 23.8, 0.0), 16.0)],
    # gripper body + cap (rotate with wrist roll): servo sleeve x 1.5..44, y -41.8..40, z -19..45
    # (3 rows along y); back wall + rack guides + pinion + gripper horn x 40..53 (4 rows along y);
    # wrist camera mount + camera board x 6..40, y+-18, z 45..87 (2 rows).
    "gripper": [(f"sleeve{i}", (x, -30.0, z), (x, 28.0, z), 13.0)
                for i, (x, z) in enumerate((x, z) for x in (12.0, 32.0) for z in (-8.0, 12.0, 32.0))]
    + [(f"guide{i}", (47.0, -34.0, z), (47.0, 34.0, z), 12.0) for i, z in enumerate((-6.0, 12.0, 30.0, 48.0))]
    + [(f"camera{i}", (x, -9.0, z), (x, 9.0, z), 12.0)
       for i, (x, z) in enumerate((x, z) for x in (14.0, 31.0) for z in (56.0, 76.0))],
    # jaw A (+y): finger x 53.8..80, y 0..8 incl. pad, z -12..12 (2 capsules along z); lower rack
    # x 45..53 along -y.
    "jaw_a": [
        ("finger0", (60.0, 4.0, -6.0), (60.0, 4.0, 6.0), 10.0),
        ("finger1", (73.0, 4.0, -6.0), (73.0, 4.0, 6.0), 10.0),
        ("rack", (49.0, -30.0, 4.0), (49.0, 4.0, 4.0), 7.0),
    ],
    # jaw B (-y): finger x 53.8..80, y -8..0, z -12..53.9 (spine up to the upper rack x 45..53).
    "jaw_b": [
        ("finger0", (60.0, -4.0, -6.0), (60.0, -4.0, 48.0), 10.0),
        ("finger1", (73.0, -4.0, -6.0), (73.0, -4.0, 48.0), 10.0),
        ("rack", (49.0, -4.0, 50.0), (49.0, 30.0, 50.0), 7.0),
    ],
}

# Static primitives (world frame, mm). The turntable is on the yaw link but is a cylinder about
# the yaw axis, so it is the same solid for every yaw angle.
STATIC = [
    ("plate", "base", "box", dict(lo=(-70.0, -60.0, 0.0), hi=(70.0, 60.0, 6.0))),
    ("tab_p", "base", "box", dict(lo=(-20.0, 52.0, 0.0), hi=(20.0, 95.0, 8.0))),
    ("tab_n", "base", "box", dict(lo=(-20.0, -95.0, 0.0), hi=(20.0, -52.0, 8.0))),
    ("gus_p", "base", "box", dict(lo=(-2.0, 52.0, 7.99), hi=(2.0, 54.5, 30.0))),
    ("gus_n", "base", "box", dict(lo=(-2.0, -54.5, 7.99), hi=(2.0, -52.0, 30.0))),
    ("housing", "base", "zcyl", dict(xy=(0.0, 0.0), r=53.5, z=(0.0, 39.5))),
    ("turntable", "shoulder", "zcyl", dict(xy=(0.0, 0.0), r=46.0, z=(40.0, 47.0))),
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
