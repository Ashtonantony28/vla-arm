"""Parametric part builders. Every builder returns ONE solid in its own link frame (mm).

Link frames (all follow spec/kinematics.yaml, zero pose):
  base           world frame, origin at the centre of the base plate underside, +z up, +x forward
  shoulder       yaw joint frame: origin on the yaw axis at the yaw horn face (z = 40 mm in world)
  upper_arm      shoulder-pitch joint frame (axis +y), arm along +x
  forearm        elbow-pitch joint frame
  wrist          wrist-pitch joint frame
  gripper        wrist-roll joint frame: origin at the roll horn face, +x towards the TCP,
                 jaws open/close along +-y

Construction idea for the pitch joints (shoulder, elbow, wrist pitch): the servo case sits in an
"inner layer" cradle of the PARENT link (y in Y_INNER); the CHILD link straddles it with two clevis
side plates in the "outer layer" (horn plate bolted to the horn on +y, idler plate on -y running on a
stub that the parent cradle carries coaxially with the output shaft). Parent and child therefore live
in different y-layers near the joint and can rotate through large angles without touching.
"""
from __future__ import annotations

import math

import cadquery as cq
import numpy as np

from . import params as P
from .geom import box, cut, cyl, ring, single_solid, union, T

S = P.SERVO


# ============================================================================ servo placeholder
def servo_body() -> cq.Solid:
    """Servo case in servo-local frame: horn face at origin, horn axis +Z, L along X, W along Y."""
    x0 = -S.L / 2 + S.horn_offset
    return single_solid(box(x0, x0 + S.L, -S.W / 2, S.W / 2, -S.stack, -S.horn_t))


def servo_horn() -> cq.Solid:
    return single_solid(cyl(S.horn_r, (0, 0, -S.horn_t), (0, 0, 0)))


def _frame(origin, X, Y, Z) -> np.ndarray:
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2] = X, Y, Z
    m[:3, 3] = origin
    assert abs(np.linalg.det(m[:3, :3]) - 1) < 1e-9
    return m


# servo mount transforms (servo-local -> link frame)
def mount_pitch(px: float, pz: float = 0.0) -> np.ndarray:
    """Pitch servo: horn axis +y, horn face at y = Y_HORN, case L along x, W along z."""
    return _frame((px, P.Y_HORN, pz), (1, 0, 0), (0, 0, -1), (0, 1, 0))


MOUNT_YAW = _frame((0, 0, P.YAW_Z), (1, 0, 0), (0, 1, 0), (0, 0, 1))          # in base frame
MOUNT_ROLL = _frame((0, 0, 0), (0, 1, 0), (0, 0, -1), (-1, 0, 0))             # in gripper frame
MOUNT_GRIP = _frame((P.PINION_X[0], 0, P.GRIP_SERVO_Z), (0, 1, 0), (0, 0, 1), (1, 0, 0))


# ============================================================================ shared features
def _horn_plate_holes(center, axis) -> list[cq.Shape]:
    """Centre bore + 4 horn screws on a PCD, as cutting tools through a plate normal to `axis`."""
    c = np.asarray(center, float); a = np.asarray(axis, float)
    # two in-plane unit vectors
    u = np.cross(a, [0, 0, 1] if abs(a[2]) < 0.9 else [1, 0, 0]); u /= np.linalg.norm(u)
    v = np.cross(a, u)
    tools = [cyl(P.HORN_BORE_R, c - 20 * a, c + 20 * a)]
    for k in range(4):
        th = math.pi / 4 + k * math.pi / 2
        p = c + P.HORN_SCREW_PCD_R * (math.cos(th) * u + math.sin(th) * v)
        tools.append(cyl(P.HORN_SCREW_R, p - 20 * a, p + 20 * a))
    return tools


def _clevis_plates(x_end: float, h: float) -> cq.Shape:
    """Child-side clevis: horn plate (+y) and idler plate (-y), disc around the axis + band to x_end."""
    plates = []
    for y0, y1 in (P.Y_HPLATE, P.Y_IPLATE):
        plates.append(union(cyl(P.R_CLEVIS, (0, y0, 0), (0, y1, 0)), box(0, x_end, y0, y1, -h, h)))
    shape = union(*plates)
    tools = _horn_plate_holes((0, P.Y_HPLATE[0], 0), (0, 1, 0))
    tools.append(cyl(P.IDLER_BORE_R, (0, P.Y_IPLATE[0] - 1, 0), (0, P.Y_IPLATE[1] + 1, 0)))
    return cut(shape, *tools)


def _cradle(px: float, pz: float = 0.0) -> cq.Shape:
    """Parent-side cradle around a pitch servo at (px, *, pz) + idler stub coaxial with the output."""
    outer = box(px - P.CRADLE_X, px + P.CRADLE_X, P.Y_CAP_BOT, P.Y_BODY_TOP,
                pz - P.CRADLE_Z, pz + P.CRADLE_Z)
    hx, hz = S.L / 2 + P.POCKET_CLR, S.W / 2 + P.POCKET_CLR
    pocket = box(px - hx, px + hx, P.Y_BODY_BOT, P.Y_BODY_TOP + 1, pz - hz, pz + hz)
    wire = box(px - P.CRADLE_X - 1, px - hx + 0.01, P.Y_BODY_BOT + 6, P.Y_BODY_TOP + 1, pz - 4, pz + 4)
    stub = cyl(P.STUB_R, (px, P.Y_CAP_BOT, pz), (px, P.Y_IPLATE[0] - 1.0, pz))
    return union(cut(outer, pocket, wire), stub)


def _lightening(x0, x1, y0, y1, r, pitch) -> list[cq.Shape]:
    n = int((x1 - x0) // pitch)
    if n < 1:
        return []
    start = x0 + ((x1 - x0) - (n - 1) * pitch) / 2
    return [cyl(r, (start + i * pitch, y0 - 1, 0), (start + i * pitch, y1 + 1, 0)) for i in range(n)]


# ============================================================================ printed parts
def pitch_link(g: P.LinkGeom, counterbalance_pin: bool = False) -> cq.Solid:
    """Upper arm / forearm: proximal clevis (outer layer) -> bridge -> inner neck -> distal cradle."""
    L = g.length
    plates = _clevis_plates(g.plate_end, g.plate_h)
    holes = []
    for y0, y1 in (P.Y_HPLATE, P.Y_IPLATE):
        holes += _lightening(P.R_CLEVIS + 4, g.plate_end - g.bridge_len - 4, y0, y1,
                             r=g.plate_h - 6.0, pitch=2 * g.plate_h + 2)
    plates = cut(plates, *holes) if holes else plates
    bridge = box(g.plate_end - g.bridge_len, g.plate_end, P.Y_IPLATE[0], P.Y_HPLATE[1], -g.plate_h, g.plate_h)
    x_neck0 = g.plate_end - g.bridge_len
    neck = box(x_neck0, L - P.CRADLE_X + 0.01, P.Y_INNER[0], P.Y_INNER[1], -g.neck_h, g.neck_h)
    # hollow the neck (open-ended tube along x is not printable flat; use a through slot along y)
    neck_slot = box(x_neck0 + 6, L - P.CRADLE_X - 4, P.Y_INNER[0] - 1, P.Y_INNER[1] + 1,
                    -g.neck_h + P.WALL, g.neck_h - P.WALL)
    parts = [plates, bridge, cut(neck, neck_slot), _cradle(L)]
    if counterbalance_pin:
        b = P.CB["b"]
        parts.append(cyl(P.CB["pin_r"], (b, P.CB["pin_y"][0], 0), (b, P.CB["pin_y"][1], 0)))
    return single_solid(union(*parts))


def upper_arm() -> cq.Solid:
    return pitch_link(P.UPPER_ARM, counterbalance_pin=True)


def forearm() -> cq.Solid:
    return pitch_link(P.FOREARM)


def base() -> cq.Solid:
    px, py, pt = P.BASE_PLATE
    tab = P.CLAMP_TAB
    plate = box(-px / 2, px / 2, -py / 2, py / 2, 0, pt)
    housing = cyl(P.BASE_R, (0, 0, 0), (0, 0, P.BASE_TOP))
    tabs = [box(-tab["x_half"], tab["x_half"], s * py / 2 - s * 1, s * tab["y_out"], 0, tab["t"]) for s in (1, -1)]
    ribs = [box(-2, 2, s * P.BASE_R * 0.9, s * (tab["y_out"] - 17), tab["t"] - 0.01, tab["t"] + 10) for s in (1, -1)]
    body = union(plate, housing, *tabs, *ribs)
    seat = cyl(P.BEARING["D"] / 2 + 0.05, (0, 0, P.BASE_TOP - P.BEARING["B"]), (0, 0, P.BASE_TOP + 1))
    hx, hy = S.L / 2 + P.POCKET_CLR, S.W / 2 + P.POCKET_CLR
    pocket = box(-hx, hx, -hy, hy, P.YAW_Z - S.stack, P.BASE_TOP + 1)
    cable = box(-P.BASE_R - 1, -hx + 0.01, -5, 5, 8, 22)
    tab_holes = [cyl(tab["hole_r"], (0, s * (tab["y_out"] - 10), -1), (0, s * (tab["y_out"] - 10), tab["t"] + 1))
                 for s in (1, -1)]
    plate_holes = [cyl(2.75, (sx * (px / 2 - 8), sy * (py / 2 - 8), -1), (sx * (px / 2 - 8), sy * (py / 2 - 8), pt + 1))
                   for sx in (1, -1) for sy in (1, -1)]
    return single_solid(cut(body, seat, pocket, cable, *tab_holes, *plate_holes))


def yaw_bearing() -> cq.Solid:
    """Purchased thin-section bearing placeholder (one ring; races not split)."""
    b = P.BEARING
    z0 = P.BASE_TOP - b["B"]
    return single_solid(ring(b["d"] / 2, b["D"] / 2, (0, 0, z0), (0, 0, P.BASE_TOP)))


def shoulder_bracket() -> cq.Solid:
    z_axis = P.SHOULDER_Z
    tt = cyl(P.TURNTABLE_R, (0, 0, 0), (0, 0, P.TURNTABLE_T))
    spigot = ring(P.SPIGOT_R[0], P.SPIGOT_R[1], (0, 0, -(P.BEARING["B"] - 0.6)), (0, 0, 0))
    race_shoulder = ring(P.SPIGOT_R[0], P.BEARING["d"] / 2 + 3.0, (0, 0, -0.5), (0, 0, 0))
    ped_top = z_axis - P.CRADLE_Z
    ped = box(-P.CRADLE_X, P.CRADLE_X, P.Y_INNER[0], P.Y_INNER[1], P.TURNTABLE_T - 0.01, ped_top + 0.01)
    ped_slot = box(-P.CRADLE_X - 1, P.CRADLE_X + 1, P.Y_INNER[0] + P.WALL, P.Y_INNER[1] - P.WALL,
                   P.TURNTABLE_T + P.WALL, ped_top - P.WALL)
    cb = P.CB
    post_top = z_axis + cb["a"] + 8.0
    post = box(*cb["post_x"], *cb["post_y"], P.TURNTABLE_T - 0.01, post_top)
    axle = cyl(1.6, (0, cb["post_y"][0] - 1, z_axis + cb["a"]), (0, cb["post_y"][1] + 1, z_axis + cb["a"]))
    hook = cyl(1.6, (0, cb["post_y"][0] - 1, P.TURNTABLE_T + 8), (0, cb["post_y"][1] + 1, P.TURNTABLE_T + 8))
    body = union(tt, spigot, race_shoulder, cut(ped, ped_slot), _cradle(0.0, z_axis), post)
    tools = [axle, hook] + _horn_plate_holes((0, 0, 0), (0, 0, 1))
    return single_solid(cut(body, *tools))


def wrist_bracket() -> cq.Solid:
    x_front = P.WRIST_L - P.WRIST_FRONT_T
    plates = _clevis_plates(x_front + 0.01, P.WRIST_PLATE_H)
    front = box(x_front, P.WRIST_L, P.Y_IPLATE[0], P.Y_HPLATE[1], -P.WRIST_PLATE_H, P.WRIST_PLATE_H)
    front = cut(front, *_horn_plate_holes((P.WRIST_L, 0, 0), (1, 0, 0)))
    return single_solid(union(plates, front))


def gripper_body() -> cq.Solid:
    hy = S.L / 2 + P.POCKET_CLR          # servo L along y here
    hz = S.W / 2 + P.POCKET_CLR
    w = 2.5
    zg = P.GRIP_SERVO_Z
    x0, x1 = S.horn_t, S.stack           # servo cases occupy x in [3, 38]
    sleeve = box(x0, x1, -hy - w, hy + w, -hz - w, zg + hz + w)
    pockets = [box(x0 - 1, x1, -hy, hy, -hz, hz), box(x0 - 1, x1, -hy, hy, zg - hz, zg + hz)]
    rack_lo_bot = zg - P.PINION_R - P.RACK_T
    rack_hi_top = zg + P.PINION_R + P.RACK_T
    guide_lo = (rack_lo_bot - 0.5 - 3.35, rack_lo_bot - 0.5)
    guide_hi = (rack_hi_top + 0.5, rack_hi_top + 0.5 + 3.0)
    yw = P.JAW_TRAVEL + 10.0
    back = box(x1, P.RACK_X[0] - 0.3, -yw, yw, min(guide_lo[0], -hz - w), guide_hi[1])
    horn_hole = cyl(S.horn_r + 0.5, (x1 - 1, 0, zg), (P.RACK_X[0] + 1, 0, zg))
    guides = [box(P.RACK_X[0] - 0.31, P.RACK_X[1], -yw, yw, *guide_lo),
              box(P.RACK_X[0] - 0.31, P.RACK_X[1], -yw, yw, *guide_hi)]
    cam_holes = [cyl(1.6, (xh, 0, zg + hz + w - 8), (xh, 0, zg + hz + w + 1)) for xh in (12.0, 28.0)]
    return single_solid(cut(union(sleeve, back, *guides), *pockets, horn_hole, *cam_holes))


def camera_mount() -> cq.Solid:
    """Wrist camera mount: sits on the gripper body top, holds a ~32x32 mm USB camera board facing +x.
    The board is tilted down by a printed 15 deg wedge face so the fingertips are centred in view."""
    z0 = P.GRIP_SERVO_Z + S.W / 2 + P.POCKET_CLR + 2.5   # gripper sleeve top
    base_ = box(6, 34, -15, 15, z0, z0 + 3)
    upright = box(6, 14, -18, 18, z0 + 3 - 0.01, z0 + 34)
    # 15 deg wedge on the front face of the upright (camera board seats on the sloped face)
    t15 = math.tan(math.radians(15))
    wedge = (cq.Workplane("XZ").polyline([(14 - 0.01, z0 + 3), (16.5, z0 + 3), (16.5 + 31 * t15, z0 + 34),
                                           (14 - 0.01, z0 + 34)])
             .close().extrude(18, both=True).val())
    body = union(base_, upright, wedge)
    holes = [cyl(1.6, (xh, 0, z0 - 1), (xh, 0, z0 + 4)) for xh in (12.0, 28.0)]
    for dy in (-14, 14):
        for dz in (16, 30):
            holes.append(cyl(1.1, (0, dy, z0 + dz), (30, dy, z0 + dz)))
    return single_solid(cut(body, *holes))


# ---------------------------------------------------------------- gripper drive (rack & pinion)
def _rack_teeth_profile(y0, y1, z_pitch, up: bool):
    """Trapezoidal 20 deg rack teeth along y. Returns list of (y, z) polygon points of the tooth band."""
    m = P.GEAR_MODULE; p = math.pi * m
    ha, hf = m, 1.25 * m
    s = 1 if up else -1
    zr, zt = z_pitch - s * hf, z_pitch + s * ha
    t_tip = p / 2 - 2 * ha * math.tan(math.radians(20))
    t_root = p / 2 + 2 * hf * math.tan(math.radians(20))
    pts = [(y0, zr)]
    y = y0 + (p - t_root) / 2
    while y + t_root <= y1:
        c = y + t_root / 2
        pts += [(c - t_root / 2, zr), (c - t_tip / 2, zt), (c + t_tip / 2, zt), (c + t_root / 2, zr)]
        y += p
    pts.append((y1, zr))
    return pts, zr


def _rack(y0, y1, lower: bool, teeth: bool) -> cq.Shape:
    zg = P.GRIP_SERVO_Z
    if lower:
        z_pitch = zg - P.PINION_R
        z_back = z_pitch - P.RACK_T
    else:
        z_pitch = zg + P.PINION_R
        z_back = z_pitch + P.RACK_T
    if not teeth:
        return box(*P.RACK_X, y0, y1, z_back, z_pitch)
    pts, zr = _rack_teeth_profile(y0, y1, z_pitch, up=lower)
    poly = [(y1, z_back), (y0, z_back)] + pts
    prof = cq.Workplane("YZ", origin=(P.RACK_X[0], 0, 0)).polyline(poly).close().extrude(P.RACK_X[1] - P.RACK_X[0])
    return prof.val()


def jaw(side: int, teeth: bool = False) -> cq.Solid:
    """side=+1: jaw A (+y), driven by the LOWER rack; side=-1: jaw B (-y), UPPER rack.
    Closed position (q=0): the pad faces meet at y=0. Pads are separate TPU parts."""
    zg = P.GRIP_SERVO_Z
    margin = P.JAW_TRAVEL + 6.0
    f_in, f_out = P.PAD_T, P.PAD_T + P.FINGER_T
    if side > 0:
        rk = _rack(-margin, f_out, lower=True, teeth=teeth)
        zr0, zr1 = zg - P.PINION_R - P.RACK_T, zg - P.PINION_R
        fz = (P.FINGER_ZMIN, zr1)
        yf = (f_in, f_out)
    else:
        rk = _rack(-f_out, margin, lower=False, teeth=teeth)
        zr0, zr1 = zg + P.PINION_R, zg + P.PINION_R + P.RACK_T
        fz = (P.FINGER_ZMIN, zr1)
        yf = (-f_out, -f_in)
    root = box(P.RACK_X[1] - 0.01, P.FINGER_X0 + 0.01, *yf, zr0, zr1)
    finger = box(P.FINGER_X0, P.FINGER_X1, *yf, *fz)
    # pad pocket: shallow 0.8 mm recess so the TPU pad is located
    return single_solid(union(rk, root, finger))


def pad(side: int) -> cq.Solid:
    y = (0.0, P.PAD_T) if side > 0 else (-P.PAD_T, 0.0)
    return single_solid(box(*P.PAD_X, *y, *P.PAD_Z))


def pinion(teeth: bool = False) -> cq.Solid:
    zg = P.GRIP_SERVO_Z
    x0, x1 = P.PINION_X
    if not teeth:
        g = cyl(P.PINION_R, (x0, 0, zg), (x1, 0, zg))
    else:
        m, n = P.GEAR_MODULE, P.PINION_TEETH
        ra, rf = P.PINION_R + m, P.PINION_R - 1.25 * m
        pts = []
        for k in range(n):
            th = 2 * math.pi * k / n
            half_p, half_t = math.pi / n, math.pi / n * 0.5
            for ang, r in ((-half_p * 0.62, rf), (-half_t * 0.55, ra), (half_t * 0.55, ra), (half_p * 0.62, rf)):
                pts.append((r * math.cos(th + ang), zg + r * math.sin(th + ang)))
        g = cq.Workplane("YZ", origin=(x0, 0, 0)).polyline(pts).close().extrude(x1 - x0).val()
    return single_solid(cut(g, *_horn_plate_holes((x0, 0, zg), (1, 0, 0))))


# ============================================================================ registry
PRINTED = {
    # name: (builder, link, material)
    "base": (base, "base", "PETG"),
    "shoulder_bracket": (shoulder_bracket, "shoulder", "PETG"),
    "upper_arm": (upper_arm, "upper_arm", "PETG"),
    "forearm": (forearm, "forearm", "PETG"),
    "wrist_bracket": (wrist_bracket, "wrist", "PETG"),
    "gripper_body": (gripper_body, "gripper", "PETG"),
    "camera_mount": (camera_mount, "gripper", "PETG"),
    "pinion": (pinion, "grip_drive", "PETG"),
    "jaw_a": (lambda teeth=False: jaw(+1, teeth), "jaw_a", "PETG"),
    "jaw_b": (lambda teeth=False: jaw(-1, teeth), "jaw_b", "PETG"),
    "pad_a": (lambda: pad(+1), "jaw_a", "TPU"),
    "pad_b": (lambda: pad(-1), "jaw_b", "TPU"),
}
TOOTHED = {"pinion", "jaw_a", "jaw_b"}


def build_printed(name: str, teeth: bool = False) -> cq.Solid:
    fn = PRINTED[name][0]
    return fn(teeth) if name in TOOTHED else fn()
