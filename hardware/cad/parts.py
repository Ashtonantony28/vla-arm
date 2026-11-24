"""Parametric part builders. Every builder returns ONE solid (or, for fastener sets, one compound) in
its own link frame (mm).

Link frames (all follow spec/kinematics.yaml, zero pose):
  base           world frame, origin at the centre of the base plate underside, +z up, +x forward
  shoulder       yaw joint frame: origin on the yaw axis at the turntable underside (z = 40 mm in world)
  upper_arm      shoulder-pitch joint frame (axis +y), arm along +x
  forearm        elbow-pitch joint frame
  wrist          wrist-pitch joint frame
  gripper        wrist-roll joint frame: origin at the roll horn face, +x towards the TCP,
                 jaws open/close along +-y

Construction idea for the pitch joints (shoulder, elbow, wrist pitch): the servo case sits in an
"inner layer" cradle of the PARENT link (y in Y_INNER); the CHILD link straddles it with two clevis
side plates in the "outer layer" (horn plate bolted to the horn on +y with 4 x M3, idler plate on -y
carrying a 6700 bearing that runs on a stub the parent cradle carries coaxially with the output).
Parent and child therefore live in different y-layers near the joint and can rotate through large
angles without touching.

Servo geometry is the STS3215 from its datasheet (params.Servo): the output axis is 12.5 mm from the
case centre, so every cradle/pocket is centred on the CASE (output axis + cx), not on the joint axis.

Styling rules applied everywhere (2026-11-24 polish): printed parts are built from filleted
primitives and true-arc profiles (no polygonal approximations of round features), plate outlines are
tangent-arc profiles, lightening cut-outs are stadium slots, gear teeth are true involutes.
"""
from __future__ import annotations

import math

import cadquery as cq
import numpy as np

from . import params as P
from .geom import box, cut, cyl, ring, single_solid, union, rbox, plate_xz, fillet_all

S = P.SERVO


# ============================================================================ servo (STS3215)
def servo_body() -> cq.Solid:
    """Servo case in servo-local frame: horn face at origin, horn axis +Z, output axis at X = Y = 0,
    case centre at X = S.cx, length along X, width along Y. Includes the rear idler boss and the
    3-pin connector housing at the -X end (where the drawing puts the two 5264 connectors)."""
    x0, x1 = S.cx - S.L / 2, S.cx + S.L / 2
    zt, zb = -S.horn_t, -S.stack
    case = rbox(x0, x1, -S.W / 2, S.W / 2, zb, zt, r=2.0, edges="|Z")
    case = fillet_all(case, 0.6, ">Z or <Z")
    tools = []
    # parting lines of the top cover and bottom cover (0.35 deep, 0.6 wide grooves)
    for zc in (zt - 3.5, zb + 3.5):
        band = box(x0 - 1, x1 + 1, -S.W / 2 - 1, S.W / 2 + 1, zc - 0.3, zc + 0.3)
        keep = rbox(x0 + 0.35, x1 - 0.35, -S.W / 2 + 0.35, S.W / 2 - 0.35, zc - 0.31, zc + 0.31, r=1.65)
        tools.append(band.cut(keep))
    # case-screw heads (4 x B-PA2.0 at the top corners) and 4 x M2 mounting holes in the bottom corners
    for sx, sy in servo_screw_xy():
        tools.append(cyl(1.7, (sx, sy, zt - 0.6), (sx, sy, zt + 1)))
        tools.append(cyl(P.SERVO_SCREW["r"], (sx, sy, zb - 1), (sx, sy, zb + 7.0)))
    # label recess on the +Y side
    tools.append(box(x0 + 8, x1 - 8, S.W / 2 - 0.3, S.W / 2 + 1, zb + 7, zt - 7))
    case = cut(case, *tools)
    boss = cyl(S.boss_r, (0, 0, zb - S.boss_h), (0, 0, zb + 0.01))
    conn = rbox(x0 - 2.0, x0 + 0.5, -3.0, 3.0, zb + 8.0, zb + 14.0, r=0.5, edges="|X")
    return single_solid(union(case, boss, conn))


def servo_horn() -> cq.Solid:
    """Aluminium disc horn (datasheet accessory): dia 19.95 flange 2.5 thick on top, dia 9 hub
    underneath, 4 x M3 tapped on a 14 mm PCD, centre hole for the spline screw."""
    fl = cyl(S.horn_r, (0, 0, -S.horn_flange_t), (0, 0, 0))
    fl = fillet_all(fl, 0.4, ">Z")
    hub = cyl(S.horn_hub_r, (0, 0, -S.horn_t), (0, 0, -S.horn_flange_t + 0.01))
    tools = [cyl(1.6, (0, 0, -S.horn_t - 1), (0, 0, 1))]
    for k in range(4):
        th = math.pi / 4 + k * math.pi / 2
        p = (P.HORN_SCREW_PCD_R * math.cos(th), P.HORN_SCREW_PCD_R * math.sin(th))
        tools.append(cyl(P.M3_R, (*p, -S.horn_t - 1), (*p, 1)))
    return single_solid(cut(union(fl, hub), *tools))


def servo_screw_xy():
    """(x, y) of the 4 mounting holes in the case bottom, servo-local."""
    x0, x1 = S.cx - S.L / 2, S.cx + S.L / 2
    i = P.SERVO_SCREW["inset"]
    return [(sx, sy) for sx in (x0 + i, x1 - i) for sy in (-S.W / 2 + i, S.W / 2 - i)]


def case_screws(floor_t: float) -> cq.Compound:
    """4 x M2 low-head screws holding a servo by its case bottom, servo-local: they pass through a
    floor/cap of thickness floor_t under the case bottom, heads in counterbores flush-recessed."""
    k = P.SERVO_SCREW
    zb = -S.stack
    z_head = zb - floor_t + k["cb"]           # head bottom (counterbore floor)
    out = []
    for sx, sy in servo_screw_xy():
        out += _socket_screw((sx, sy, z_head), (0, 0, -1), 0.0, r=k["r"], head_r=k["head_r"], head_h=k["head_h"]).Solids()
        out += [single_solid(cyl(k["r"] - P.SHANK_CLR, (sx, sy, z_head - 0.01), (sx, sy, z_head + k["len"])))]
    return cq.Compound.makeCompound(_fuse_pairs(out))


def _fuse_pairs(solids):
    """Fuse each screw's head with its shank (consecutive pairs) so every screw is one solid."""
    out = []
    for i in range(0, len(solids), 2):
        out.append(single_solid(union(solids[i], solids[i + 1])))
    return out


def servo_screw_tools(floor_t: float, m: np.ndarray) -> list[cq.Shape]:
    """Clearance holes + head counterbores for case_screws(floor_t), placed with servo mount m."""
    from .geom import place
    k = P.SERVO_SCREW
    zb = -S.stack
    out = []
    for sx, sy in servo_screw_xy():
        out.append(place(cyl(k["r"] + 0.2, (sx, sy, zb - floor_t - 1), (sx, sy, zb + 0.5)), m))
        out.append(place(cyl(k["head_r"] + 0.3, (sx, sy, zb - floor_t - 1), (sx, sy, zb - floor_t + k["cb"])), m))
    return out


def _socket_screw(p_head, axis, length, r=P.M3_R, head_r=2.75, head_h=3.0) -> cq.Shape:
    """ISO 4762 socket-head cap screw: head bottom at p_head, shank along -axis (into the work)."""
    a = np.asarray(axis, float); a /= np.linalg.norm(a)
    p = np.asarray(p_head, float)
    head = cyl(head_r, p, p + head_h * a)
    hexr = (0.6 * head_r) / 2 / math.cos(math.pi / 6) * 1.6  # socket A/F ~ 0.55 x head dia
    # hex socket (2.5 mm A/F), modelled as a 6-sided prism along the axis
    u = np.cross(a, [0, 0, 1] if abs(a[2]) < 0.9 else [1, 0, 0]); u /= np.linalg.norm(u)
    v = np.cross(a, u)
    top = p + head_h * a
    pts = [top + hexr * (math.cos(k * math.pi / 3) * u + math.sin(k * math.pi / 3) * v) for k in range(6)]
    wire = cq.Wire.makePolygon([cq.Vector(*q) for q in pts], close=True)
    sock = cq.Solid.extrudeLinear(cq.Face.makeFromWires(wire), cq.Vector(*(-0.6 * head_h * a)))
    head = head.cut(sock)
    if length <= 0:
        return head
    shank = cyl(r - P.SHANK_CLR, p - length * a, p + 0.01 * a)
    return union(head, shank)


def horn_screws(plate_t: float, n_len: float | None = None) -> cq.Compound:
    """The 4 x M3 horn screws plus the centre M3 spline screw, in servo-local coordinates (horn face at
    z = 0, +Z out of the horn). Heads sit on the outer face of a plate of thickness plate_t bolted to the
    horn face; the spline-screw head sits in the plate's centre bore, on the horn face."""
    length = n_len if n_len is not None else plate_t + 2.0
    solids = []
    for k in range(4):
        th = math.pi / 4 + k * math.pi / 2
        p = (P.HORN_SCREW_PCD_R * math.cos(th), P.HORN_SCREW_PCD_R * math.sin(th), plate_t)
        solids += _socket_screw(p, (0, 0, 1), length).Solids()
    # spline screw: shank drawn to 0.5 mm above the case top (it threads into the output shaft, not modelled)
    solids += _socket_screw((0, 0, 0), (0, 0, 1), S.horn_t - 0.5, head_r=2.75, head_h=min(3.0, plate_t)).Solids()
    return cq.Compound.makeCompound(solids)


def _frame(origin, X, Y, Z) -> np.ndarray:
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2] = X, Y, Z
    m[:3, 3] = origin
    assert abs(np.linalg.det(m[:3, :3]) - 1) < 1e-9
    return m


# servo mount transforms (servo-local -> link frame)
def mount_pitch(px: float, pz: float = 0.0) -> np.ndarray:
    """Pitch servo: horn axis +y, horn face at y = Y_HORN, case along -x from the axis (inboard)."""
    return _frame((px, P.Y_HORN, pz), (1, 0, 0), (0, 0, -1), (0, 1, 0))


MOUNT_YAW = _frame((0, 0, P.YAW_Z + P.YAW_HORN_UP), (1, 0, 0), (0, 1, 0), (0, 0, 1))   # in base frame
MOUNT_ROLL = _frame((0, 0, 0), (0, 1, 0), (0, 0, -1), (-1, 0, 0))                      # in gripper frame
MOUNT_GRIP = _frame((P.PINION_X[0], 0, P.GRIP_SERVO_Z), (0, 1, 0), (0, 0, 1), (1, 0, 0))


# ============================================================================ purchased bearings
def idler_bearing() -> cq.Solid:
    """6700-2RS deep-groove bearing (10 x 15 x 4), axis +y, centred on the joint axis, outer face at
    y = 0 and inner face at y = +B (placed by the assembly). Races and shields are separate faces."""
    b = P.IDLER_BEARING
    ri, ro, w = b["d"] / 2, b["D"] / 2, b["B"]
    body = ring(ri, ro, (0, 0, 0), (0, w, 0))
    tools = []
    for y0, y1 in ((-1, 0.25), (w - 0.25, w + 1)):          # shield recess between the races
        tools.append(ring(ri + 0.9, ro - 0.8, (0, y0, 0), (0, y1, 0)))
    return single_solid(cut(body, *tools))


def yaw_bearing() -> cq.Solid:
    """61816-2RS thin-section bearing (80 x 100 x 10): inner/outer race with shield recesses."""
    b = P.BEARING
    z0 = P.BASE_TOP - b["B"]
    ri, ro = b["d"] / 2, b["D"] / 2
    body = ring(ri, ro, (0, 0, z0), (0, 0, P.BASE_TOP))
    tools = [ring(ri + 2.2, ro - 2.2, (0, 0, z0 - 1), (0, 0, z0 + 0.3)),
             ring(ri + 2.2, ro - 2.2, (0, 0, P.BASE_TOP - 0.3), (0, 0, P.BASE_TOP + 1))]
    return single_solid(cut(body, *tools))


# ============================================================================ shared features
def _horn_plate_holes(center, axis, cb_depth: float = 0.0) -> list[cq.Shape]:
    """Centre bore (spline-screw head) + 4 x M3 clearance on the 14 mm PCD, as cutting tools through a
    plate normal to `axis` whose horn-side face is at `center`. cb_depth > 0 adds head counterbores
    on the far face (for plates whose far face must stay flush)."""
    c = np.asarray(center, float); a = np.asarray(axis, float)
    u = np.cross(a, [0, 0, 1] if abs(a[2]) < 0.9 else [1, 0, 0]); u /= np.linalg.norm(u)
    v = np.cross(a, u)
    tools = [cyl(P.HORN_BORE_R, c - 20 * a, c + 20 * a)]
    for k in range(4):
        th = math.pi / 4 + k * math.pi / 2
        p = c + P.HORN_SCREW_PCD_R * (math.cos(th) * u + math.sin(th) * v)
        tools.append(cyl(P.HORN_SCREW_R, p - 20 * a, p + 20 * a))
    return tools


def _stadium_slots(x0, x1, half_h, y0, y1, n_max=6, rib=5.0) -> list[cq.Shape]:
    """Row of stadium-shaped lightening slots through a plate normal to y, between x0 and x1."""
    span = x1 - x0
    if span < 2 * half_h + 2:
        return []
    n = max(1, min(n_max, int((span + rib) // (3 * half_h + rib))))
    L = (span - (n - 1) * rib) / n
    out = []
    for i in range(n):
        a = x0 + i * (L + rib)
        out.append(rbox(a, a + L, y0 - 1, y1 + 1, -half_h, half_h, r=half_h - 0.01, edges="|Y"))
    return out


def _clevis_plates(x_end: float, h: float, end_r: float = 3.0) -> cq.Shape:
    """Child-side clevis: horn plate (+y) and idler plate (-y), tangent-arc outline from the disc
    around the joint axis to the band end at x_end. Idler plate carries the 6700 bearing seat."""
    plates = [plate_xz(P.R_CLEVIS, x_end, h, y0, y1, end_r) for y0, y1 in (P.Y_HPLATE, P.Y_IPLATE)]
    shape = union(*plates)
    tools = _horn_plate_holes((0, P.Y_HPLATE[0], 0), (0, 1, 0))
    tools.append(cyl(P.IDLER_BORE_R, (0, P.Y_IPLATE[0] - 1, 0), (0, P.Y_IPLATE[1] + 1, 0)))
    return cut(shape, *tools)


def _cradle(px: float, pz: float = 0.0, round_bottom: bool = True) -> cq.Shape:
    """Parent-side cradle around a pitch servo whose output axis is at (px, *, pz): pocket centred on
    the case (axis + cx along x), bottom cap with a clearance hole for the rear boss, and the bearing
    stub coaxial with the output (hollow for the boss)."""
    cxw = px + S.cx
    r = 4.0
    outer = rbox(cxw - P.CRADLE_X, cxw + P.CRADLE_X, P.Y_CAP_BOT, P.Y_BODY_TOP,
                 pz - P.CRADLE_Z, pz + P.CRADLE_Z, r=r, edges="|Y")
    if not round_bottom:   # square lower corners where the cradle sits on a pedestal
        outer = union(outer, box(cxw - P.CRADLE_X, cxw + P.CRADLE_X, P.Y_CAP_BOT, P.Y_BODY_TOP,
                                 pz - P.CRADLE_Z, pz - P.CRADLE_Z + r + 0.01))
    hx, hz = S.L / 2 + P.POCKET_CLR, S.W / 2 + P.POCKET_CLR
    pocket = rbox(cxw - hx, cxw + hx, P.Y_BODY_BOT, P.Y_BODY_TOP + 1, pz - hz, pz + hz, r=2.2, edges="|Y")
    wire = rbox(cxw - P.CRADLE_X - 1, cxw - hx + 0.01, P.Y_BODY_BOT + 6, P.Y_BODY_TOP + 1, pz - 4, pz + 4,
                r=1.0, edges="|X")
    stub = cyl(P.STUB_R, (px, P.Y_CAP_BOT + 0.01, pz), (px, P.Y_IPLATE[0], pz))
    stub = fillet_all(stub, 0.5, "<Y")
    boss = cyl(P.BOSS_CLR_R, (px, P.Y_BODY_BOT + 0.5, pz), (px, P.Y_BODY_BOT - S.boss_h - 0.4, pz))
    return cut(union(cut(outer, pocket, wire), stub), boss, *servo_screw_tools(P.CAP_T, mount_pitch(px, pz)))


# ============================================================================ printed parts
def pitch_link(g: P.LinkGeom, counterbalance_pin: bool = False) -> cq.Solid:
    """Upper arm / forearm: proximal clevis (outer layer) -> bridge -> inner neck -> distal cradle."""
    L = g.length
    plates = _clevis_plates(g.plate_end, g.plate_h)
    holes = []
    for y0, y1 in (P.Y_HPLATE, P.Y_IPLATE):
        holes += _stadium_slots(P.R_CLEVIS + 3, g.plate_end - g.bridge_len - 4, g.plate_h - 5.0, y0, y1)
    plates = cut(plates, *holes) if holes else plates
    bridge = rbox(g.plate_end - g.bridge_len, g.plate_end, P.Y_IPLATE[0], P.Y_HPLATE[1], -g.plate_h, g.plate_h,
                  r=3.0, edges="|Y")
    x_neck0 = g.plate_end - g.bridge_len
    x_cr = L + S.cx - P.CRADLE_X               # cradle's proximal face
    neck = rbox(x_neck0 + 1, x_cr + 1.0, P.Y_INNER[0], P.Y_INNER[1], -g.neck_h, g.neck_h, r=2.0, edges="|X")
    parts = [plates, bridge]
    if x_cr - x_neck0 > 20:
        slot = rbox(x_neck0 + 6, x_cr - 4, P.Y_INNER[0] - 1, P.Y_INNER[1] + 1,
                    -g.neck_h + P.WALL, g.neck_h - P.WALL, r=g.neck_h - P.WALL - 0.01, edges="|Y")
        parts.append(cut(neck, slot))
    else:
        parts.append(neck)
    parts.append(_cradle(L))
    if counterbalance_pin:
        b = P.CB["b"]
        pin = cyl(P.CB["pin_r"], (b, P.CB["pin_y"][0] - 0.01, 0), (b, P.CB["pin_y"][1], 0))
        pin = fillet_all(pin, 0.6, ">Y")
        groove = ring(P.CB["pin_r"] - 0.8, P.CB["pin_r"] + 1, (b, P.CB_PLANE_Y - 0.8, 0), (b, P.CB_PLANE_Y + 0.8, 0))
        boss = cyl(P.CB["pin_r"] + 3, (b, P.Y_HPLATE[1] - 0.01, 0), (b, P.Y_HPLATE[1] + 2.5, 0))
        parts.append(union(cut(pin, groove), boss))
    return single_solid(union(*parts))


def upper_arm() -> cq.Solid:
    return pitch_link(P.UPPER_ARM, counterbalance_pin=True)


def forearm() -> cq.Solid:
    return pitch_link(P.FOREARM)


def base() -> cq.Solid:
    px, py, pt = P.BASE_PLATE
    tab = P.CLAMP_TAB
    plate = rbox(-px / 2, px / 2, -py / 2, py / 2, 0, pt, r=10.0, edges="|Z")
    plate = fillet_all(plate, 1.2, ">Z")
    housing = cyl(P.BASE_R, (0, 0, pt - 0.01), (0, 0, P.BASE_TOP))
    housing = fillet_all(housing, 1.0, ">Z")
    tabs = []
    for s in (1, -1):
        y_in, y_out = s * (py / 2 - 1), s * tab["y_out"]
        yc = y_out - s * tab["x_half"]
        t = union(box(-tab["x_half"], tab["x_half"], y_in, yc, 0, tab["t"]), cyl(tab["x_half"], (0, yc, 0), (0, yc, tab["t"])))
        tabs.append(fillet_all(t, 1.0, ">Z"))
    # triangular gussets from the housing wall down onto the clamp tabs
    gussets = []
    for s in (1, -1):
        y0, y1 = s * (P.BASE_R - 1.0), s * (tab["y_out"] - 2 * tab["x_half"] - 1)
        g = (cq.Workplane("YZ", origin=(-2.0, 0, 0)).polyline([(y0, tab["t"] - 0.01), (y0, 30.0), (y1, tab["t"] - 0.01)])
             .close().extrude(4.0).val())
        gussets.append(g)
    body = union(plate, housing, *tabs, *gussets)
    seat = cyl(P.BEARING["D"] / 2, (0, 0, P.BASE_TOP - P.BEARING["B"]), (0, 0, P.BASE_TOP + 1))
    lip = cyl(P.BEARING["D"] / 2 - 2.0, (0, 0, P.BASE_TOP - P.BEARING["B"] - 3), (0, 0, P.BASE_TOP))
    hx, hy = S.L / 2 + P.POCKET_CLR, S.W / 2 + P.POCKET_CLR
    z_case_bot = P.YAW_Z + P.YAW_HORN_UP - S.stack
    pocket = rbox(S.cx - hx, S.cx + hx, -hy, hy, z_case_bot, P.BASE_TOP + 1, r=2.2, edges="|Z")
    boss = cyl(P.BOSS_CLR_R, (0, 0, -1), (0, 0, z_case_bot + 0.5))
    cable = rbox(-P.BASE_R - 1, S.cx - hx + 0.01, -5, 5, z_case_bot + 4, z_case_bot + 18, r=1.5, edges="|X")
    tab_holes = [cyl(tab["hole_r"], (0, s * (tab["y_out"] - tab["x_half"]), -1), (0, s * (tab["y_out"] - tab["x_half"]), tab["t"] + 1))
                 for s in (1, -1)]
    plate_holes = [cyl(2.75, (sx * (px / 2 - 9), sy * (py / 2 - 9), -1), (sx * (px / 2 - 9), sy * (py / 2 - 9), pt + 1))
                   for sx in (1, -1) for sy in (1, -1)]
    return single_solid(cut(body, seat, lip, pocket, boss, cable, *tab_holes, *plate_holes,
                            *servo_screw_tools(z_case_bot, MOUNT_YAW)))


def shoulder_bracket() -> cq.Solid:
    z_axis = P.SHOULDER_Z
    tt = cyl(P.TURNTABLE_R, (0, 0, 0), (0, 0, P.TURNTABLE_T))
    tt = fillet_all(tt, 1.0, ">Z")
    spigot = ring(P.SPIGOT_R[0], P.SPIGOT_R[1], (0, 0, -(P.BEARING["B"] - 0.6)), (0, 0, 0.01))
    race_shoulder = ring(P.SPIGOT_R[0], P.BEARING["d"] / 2 + 3.0, (0, 0, -0.5), (0, 0, 0.01))
    ped_top = z_axis - P.CRADLE_Z
    cxw = S.cx
    ped = box(cxw - P.CRADLE_X, cxw + P.CRADLE_X, P.Y_INNER[0], P.Y_INNER[1], P.TURNTABLE_T - 0.01, ped_top + 0.01)
    ped_slot = rbox(cxw - P.CRADLE_X - 1, cxw + P.CRADLE_X + 1, P.Y_INNER[0] + P.WALL, P.Y_INNER[1] - P.WALL,
                    P.TURNTABLE_T + P.WALL, ped_top - P.WALL, r=3.0, edges="|X")
    cb = P.CB
    z_a = z_axis + cb["a"]
    post_top = z_a + 8.0
    px0, px1 = cb["post_x"]
    post = rbox(px0, px1, *cb["post_y"], P.TURNTABLE_T - 0.01, post_top, r=0.0)
    # round the post top into a half-disc about the pulley axle
    post = cut(post, box(px0 - 1, px1 + 1, cb["post_y"][0] - 1, cb["post_y"][1] + 1, z_a, post_top + 1))
    post = union(post, cyl((px1 - px0) / 2, (0, cb["post_y"][0], z_a), (0, cb["post_y"][1], z_a)))
    # pulley slot (open at the top) and axle hole
    slot = box(px0 - 1, px1 + 1, *P.CB_PULLEY_SLOT, z_a - P.CB_PULLEY["r"] - 1.5, z_a + 20)
    axle = cyl(P.CB_PULLEY["axle_r"], (0, cb["post_y"][0] - 1, z_a), (0, cb["post_y"][1] + 1, z_a))
    # gusset tying the post to the pedestal
    gus = union(*[(cq.Workplane("XZ", origin=(0, cb["post_y"][1], 0))
                   .polyline([(sx * (px1 - 0.01), P.TURNTABLE_T - 0.01), (sx * (px1 + 12), P.TURNTABLE_T - 0.01),
                              (sx * (px1 - 0.01), P.TURNTABLE_T + 24)]).close()
                   .extrude(cb["post_y"][1] - cb["post_y"][0]).val()) for sx in (1, -1)])
    body = union(tt, spigot, race_shoulder, cut(ped, ped_slot), _cradle(0.0, z_axis, round_bottom=False),
                 cut(post, slot), gus)
    tools = [axle]
    # yaw horn: counterbore in the turntable underside + screw holes + head counterbores in the
    # pedestal floor (screws go in from inside the open pedestal)
    tools.append(cyl(P.HORN_CB_R, (0, 0, -1), (0, 0, P.YAW_HORN_UP)))
    tools += _horn_plate_holes((0, 0, P.YAW_HORN_UP), (0, 0, 1))
    # one round recess for all five screw heads (separate counterbores would leave 0.8 mm webs)
    tools.append(cyl(P.HORN_SCREW_PCD_R + 3.1, (0, 0, P.TURNTABLE_T - 0.01), (0, 0, P.TURNTABLE_T + P.WALL + 1)))
    return single_solid(cut(body, *tools))


def wrist_bracket() -> cq.Solid:
    x_front = P.WRIST_L - P.WRIST_FRONT_T
    plates = _clevis_plates(x_front + 0.01, P.WRIST_PLATE_H)
    front = rbox(x_front, P.WRIST_L, P.Y_IPLATE[0], P.Y_HPLATE[1], -P.WRIST_PLATE_H, P.WRIST_PLATE_H,
                 r=3.0, edges="|X")
    front = cut(front, *_horn_plate_holes((P.WRIST_L, 0, 0), (1, 0, 0)))
    return single_solid(union(plates, front))


# ---------------------------------------------------------------- gripper body
def _grip_frame():
    """Shared numbers for the gripper sleeve (frame = roll joint)."""
    hy = S.L / 2 + P.POCKET_CLR          # servo L along y
    hz = S.W / 2 + P.POCKET_CLR
    yc = S.cx                             # both case centres sit at y = cx (outputs on y = 0)
    zg = P.GRIP_SERVO_Z
    x_roll = (S.horn_t, S.stack)                                       # roll case x range
    x_grip = (P.PINION_X[0] - S.stack, P.PINION_X[0] - S.horn_t)       # gripper case x range
    return hy, hz, yc, zg, x_roll, x_grip


def gripper_body() -> cq.Solid:
    hy, hz, yc, zg, x_roll, x_grip = _grip_frame()
    w = P.SLEEVE_W
    x0, x1 = min(x_roll[0], x_grip[0]), max(x_roll[1], x_grip[1])
    top = zg + hz + P.SLEEVE_TOP_T
    sleeve = rbox(x0, x1, yc - hy - w, yc + hy + w, -hz - w, top, r=3.0, edges="|X")
    pockets = [rbox(x_roll[0] - 1, x_roll[1], yc - hy, yc + hy, -hz, hz, r=2.2, edges="|X"),
               rbox(x_grip[0] - 1, x_grip[1], yc - hy, yc + hy, zg - hz, zg + hz, r=2.2, edges="|X")]
    rack_lo_bot = zg - P.PINION_R - P.RACK_T
    rack_hi_top = zg + P.PINION_R + P.RACK_T
    guide_lo = (rack_lo_bot - P.GUIDE_CLR - 3.35, rack_lo_bot - P.GUIDE_CLR)
    guide_hi = (rack_hi_top + P.GUIDE_CLR, rack_hi_top + P.GUIDE_CLR + 3.0)
    yw = P.JAW_TRAVEL + 10.0
    xb1 = P.RACK_X[0] - 0.3
    back = rbox(x1 - 0.01, xb1, -yw, yw, min(guide_lo[0], -hz - w), guide_hi[1], r=3.0, edges="|X")
    guides = [rbox(xb1 - 0.01, P.RACK_X[1], -yw, yw, *guide_lo, r=1.0, edges="|X"),
              rbox(xb1 - 0.01, P.RACK_X[1], -yw, yw, *guide_hi, r=1.0, edges="|X")]
    body = union(sleeve, back, *guides)
    tools = list(pockets)
    tools.append(cyl(S.horn_r + 0.6, (x_grip[1] - 0.5, 0, zg), (P.RACK_X[0] + 1, 0, zg)))      # gripper horn
    tools.append(cyl(P.BOSS_CLR_R, (x_roll[1] - 0.5, 0, 0), (xb1 + 0.01, 0, 0)))  # roll boss (through the back wall only: a blind end would be a 0.6 mm skin)
    tools.append(cyl(P.BOSS_CLR_R, (x_grip[0] - S.boss_h - 1.0, 0, zg), (x_grip[0] + 0.5, 0, zg)))  # grip boss
    # connector windows on the -y end for both servos
    yo = yc - hy
    for zc, (xa, xb) in ((0.0, x_roll), (zg, x_grip)):
        xs = xb - 14.0 if (xa, xb) == x_roll else xa + 8.0
        tools.append(rbox(xs - 1.0, xs + 7.0, yo - w - 1, yo + 0.01, zc - 4.0, zc + 4.0, r=1.0, edges="|Y"))
    tools += gripper_cap_holes()
    # camera-mount screw holes (M3, tapped into the thickened sleeve top)
    for xh in P.CAM_SCREW_X:
        tools.append(cyl(P.M3_R, (xh, 0, top - 3.5), (xh, 0, top + 1)))
    return single_solid(cut(body, *tools))


def camera_mount() -> cq.Solid:
    """Wrist camera mount: sits on the gripper sleeve top, carries a 32 x 32 mm USB camera board on a
    face tilted 15 deg down so the fingertips are centred in view. 4 x M2 board screws on a 28 mm square,
    drilled normal to the tilted face; 2 x M3 to the sleeve."""
    z0 = P.GRIP_SERVO_Z + S.W / 2 + P.POCKET_CLR + P.SLEEVE_TOP_T
    base_ = rbox(6, 36, -15, 15, z0, z0 + 3, r=4.0, edges="|Z")
    up_x = (6.0, 14.0)
    upright = rbox(*up_x, -18, 18, z0 + 3 - 0.01, z0 + 3 + P.CAM_FACE_H, r=3.0, edges="|X")
    t = math.tan(math.radians(P.CAM_TILT))
    x_f0 = 16.5
    wedge = (cq.Workplane("XZ", origin=(0, 18, 0))
             .polyline([(up_x[1] - 0.01, z0 + 3), (x_f0, z0 + 3), (x_f0 + P.CAM_FACE_H * t, z0 + 3 + P.CAM_FACE_H),
                        (up_x[1] - 0.01, z0 + 3 + P.CAM_FACE_H)]).close().extrude(36).val())
    body = union(base_, upright, wedge)
    tools = []
    for xh in P.CAM_SCREW_X:
        tools.append(cyl(1.7, (xh, 0, z0 - 1), (xh, 0, z0 + 4)))
    m = camera_board_frame()
    n = m[:3, 2]
    for du in (-14, 14):
        for dv in (-14, 14):
            c = m[:3, 3] + du * m[:3, 0] + dv * m[:3, 1]
            tools.append(cyl(1.0, c - 6 * n, c + 1 * n))
    return single_solid(cut(body, *tools))


def camera_board_frame() -> np.ndarray:
    """Frame of the camera board's back face centre: X up the tilted face, Y = +y, Z = face normal."""
    z0 = P.GRIP_SERVO_Z + S.W / 2 + P.POCKET_CLR + P.SLEEVE_TOP_T
    a = math.radians(P.CAM_TILT)
    zc = z0 + 3 + P.CAM_FACE_H / 2
    xc = 16.5 + (P.CAM_FACE_H / 2) * math.tan(a)
    u = np.array([math.sin(a), 0, math.cos(a)])
    nrm = np.array([math.cos(a), 0, -math.sin(a)])
    return _frame((xc, 0, zc), u, (0, 1, 0), nrm) if np.linalg.det(np.c_[u, [0, 1, 0], nrm]) > 0 else \
        _frame((xc, 0, zc), u, (0, -1, 0), nrm)


def camera_board() -> cq.Shape:
    """Purchased 32 x 32 mm USB camera board with an M12 lens holder and lens, in board-frame
    coordinates (back face at z = 0, lens along +z)."""
    t = 1.6
    pcb = rbox(-16, 16, -16, 16, 0, t, r=2.0, edges="|Z")
    holes = [cyl(1.1, (du, dv, -1), (du, dv, t + 1)) for du in (-14, 14) for dv in (-14, 14)]
    pcb = cut(pcb, *holes)
    holder = rbox(-7.5, 7.5, -7.5, 7.5, t - 0.01, t + 5.5, r=1.0, edges="|Z")
    barrel = cyl(6.0, (0, 0, t + 5.5 - 0.01), (0, 0, t + 13.0))
    ring_ = cyl(7.0, (0, 0, t + 13.0 - 0.01), (0, 0, t + 15.0))
    glass = cyl(4.0, (0, 0, t + 15.0 - 0.01), (0, 0, t + 15.4))
    conn = box(-4.0, 4.0, -15.5, -11.0, t - 0.01, t + 3.0)
    return union(pcb, holder, barrel, ring_, glass, conn)


def camera_screws() -> cq.Compound:
    """4 x M2 x 6 socket board screws (board frame) — heads on the board's front face."""
    out = []
    for du in (-14, 14):
        for dv in (-14, 14):
            out += _socket_screw((du, dv, 1.6), (0, 0, 1), 6.0, r=1.0, head_r=1.9, head_h=1.6).Solids()
    return cq.Compound.makeCompound(out)


def cam_mount_screws() -> cq.Compound:
    """2 x M3 x 6 socket screws holding the camera mount to the gripper sleeve (gripper frame)."""
    z0 = P.GRIP_SERVO_Z + S.W / 2 + P.POCKET_CLR + P.SLEEVE_TOP_T
    out = []
    for xh in P.CAM_SCREW_X:
        out += _socket_screw((xh, 0, z0 + 3), (0, 0, 1), 6.0).Solids()
    return cq.Compound.makeCompound(out)


# ---------------------------------------------------------------- gripper drive (rack & pinion)
def _involute_tooth_pts(rb, r0, r1, n=12):
    """Points (r, angle) of an involute from radius r0 to r1 on base circle rb."""
    pts = []
    for i in range(n + 1):
        r = r0 + (r1 - r0) * i / n
        r = max(r, rb)
        t = math.sqrt((r / rb) ** 2 - 1)
        pts.append((r, t - math.atan(t)))
    return pts


def _pinion_profile(m, z, alpha_deg=20.0, backlash=P.GEAR_BACKLASH):
    """Closed wire of an involute spur gear in the YZ plane (centre at origin): splines for the
    flanks, true arcs for tips and roots."""
    a = math.radians(alpha_deg)
    rp = m * z / 2
    rb = rp * math.cos(a)
    ra, rf = rp + m, rp - 1.25 * m
    inv = lambda alpha: math.tan(alpha) - alpha
    # half tooth angle at the pitch circle, thinned by backlash/2 on each flank
    s = math.pi * m / 2 - backlash
    half_p = s / (2 * rp) + inv(a)        # angle from tooth centre to involute origin on base circle
    flank = _involute_tooth_pts(rb, max(rb, rf), ra, 16)
    edges = []
    pitch = 2 * math.pi / z
    P2 = lambda r, th: cq.Vector(0, r * math.cos(th), r * math.sin(th))
    for k in range(z):
        c = -math.pi / 2 + k * pitch       # tooth 0 points straight down (-z) at q = 0
        right = [P2(r, c - (half_p - phi)) for r, phi in flank]      # from root side to tip
        left = [P2(r, c + (half_p - phi)) for r, phi in flank]
        # radial root segment if base circle > root circle
        r_start = right[0].Length
        if r_start > rf + 1e-6:
            th_r = c - half_p
            edges.append(cq.Edge.makeLine(P2(rf, th_r), right[0]))
        edges.append(cq.Edge.makeSpline(right))
        th_tip_r = c - (half_p - flank[-1][1]); th_tip_l = c + (half_p - flank[-1][1])
        edges.append(cq.Edge.makeThreePointArc(right[-1], P2(ra, c), left[-1]))
        edges.append(cq.Edge.makeSpline(list(reversed(left))))
        if r_start > rf + 1e-6:
            edges.append(cq.Edge.makeLine(left[0], P2(rf, c + half_p)))
        # root arc to the next tooth
        th0 = c + half_p
        th1 = c + pitch - half_p
        edges.append(cq.Edge.makeThreePointArc(P2(rf, th0), P2(rf, (th0 + th1) / 2), P2(rf, th1)))
    return cq.Wire.assembleEdges(edges)


def _rack(y0, y1, lower: bool, teeth: bool) -> cq.Shape:
    """Rack bar between y0..y1 (jaw frame at q = 0). Straight-flank 20 deg teeth (a true involute rack),
    phased so a pinion tooth points straight down at q = 0 (lower rack has a gap at y = 0, upper rack a
    tooth, since the pinion has an odd tooth count). Root fillets are arcs."""
    zg = P.GRIP_SERVO_Z
    m = P.GEAR_MODULE; p = math.pi * m
    if lower:
        z_pitch, z_back = zg - P.PINION_R, zg - P.PINION_R - P.RACK_T
    else:
        z_pitch, z_back = zg + P.PINION_R, zg + P.PINION_R + P.RACK_T
    if not teeth:
        return box(*P.RACK_X, y0, y1, z_back, z_pitch)
    s = 1 if lower else -1                       # direction from rack body towards the pinion
    ha, hf = m, 1.25 * m
    zt, zr = z_pitch + s * ha, z_pitch - s * hf
    ta = math.tan(math.radians(20))
    half_s = (p / 2 - P.GEAR_BACKLASH) / 2       # half tooth thickness at the pitch line
    off = p / 2 if lower else 0.0                # tooth centre phase
    k0, k1 = math.ceil((y0 + 3 - off) / p), math.floor((y1 - 3 - off) / p)
    pts = [(y0, z_back), (y0, zr)]
    for k in range(k0, k1 + 1):
        c = off + k * p
        pts += [(c - half_s - hf * ta, zr), (c - half_s + ha * ta, zt), (c + half_s - ha * ta, zt), (c + half_s + hf * ta, zr)]
    pts += [(y1, zr), (y1, z_back)]
    prof = cq.Workplane("YZ", origin=(P.RACK_X[0], 0, 0)).polyline(pts).close().extrude(P.RACK_X[1] - P.RACK_X[0])
    return prof.val()


def _finger_profile(side: int):
    """(x, z) outline of a finger, CCW. Jaw A (+y) is driven by the lower rack, jaw B (-y) by the upper
    one, so B has a spine up to its rack."""
    zg = P.GRIP_SERVO_Z
    x0, x1 = P.FINGER_X0, P.FINGER_X1
    zlo, zhi = P.FINGER_Z
    tip = [(x0, zlo), (x1 - 3, zlo), (x1, zlo + 3), (x1, zhi - 3), (x1 - 3, zhi)]
    if side > 0:
        zr0 = zg - P.PINION_R - P.RACK_T
        return tip + [(x0, zhi)], (zr0, zg - P.PINION_R)
    zr1 = zg + P.PINION_R + P.RACK_T
    return tip + [(x0 + 6.5, zhi), (x0 + 6.5, zr1), (x0, zr1)], (zg + P.PINION_R, zr1)


def jaw(side: int, teeth: bool = False) -> cq.Solid:
    """side=+1: jaw A (+y), driven by the LOWER rack; side=-1: jaw B (-y), UPPER rack.
    Closed position (q=0): the pad faces meet at y=0. Pads are separate TPU parts keyed into a
    0.8 mm pocket."""
    margin = P.JAW_TRAVEL + 6.0
    f_in, f_out = P.PAD_T, P.PAD_T + P.FINGER_T
    yf = (f_in, f_out) if side > 0 else (-f_out, -f_in)
    rk = _rack(-margin, f_out, lower=True, teeth=teeth) if side > 0 else _rack(-f_out, margin, lower=False, teeth=teeth)
    outline, (zr0, zr1) = _finger_profile(side)
    finger = (cq.Workplane("XZ", origin=(0, yf[1], 0)).polyline(outline).close().extrude(yf[1] - yf[0]).val())
    finger = fillet_all(finger, 2.5, "|Y")
    root = box(P.RACK_X[1] - 0.01, P.FINGER_X0 + 2.0, *yf, zr0, zr1)
    body = union(rk, root, finger)
    # pad key pocket, 0.8 deep in the gripping face
    y_face = yf[0] if side > 0 else yf[1]
    d = 0.8 * side
    pocket = rbox(P.PAD_X[0] + 1, P.PAD_X[1] - 1, y_face, y_face + d, P.PAD_Z[0] + 1, P.PAD_Z[1] - 1, r=2.0, edges="|Y")
    return single_solid(cut(body, pocket))


def pad(side: int) -> cq.Solid:
    """TPU pad: 2 mm slab with a 0.8 mm key that fills the finger's pocket (glued)."""
    y = (0.0, P.PAD_T) if side > 0 else (-P.PAD_T, 0.0)
    slab = rbox(*P.PAD_X, *y, *P.PAD_Z, r=2.5, edges="|Y")
    yk = (P.PAD_T - 0.01, P.PAD_T + 0.8) if side > 0 else (-P.PAD_T - 0.8, -P.PAD_T + 0.01)
    key = rbox(P.PAD_X[0] + 1, P.PAD_X[1] - 1, *yk, P.PAD_Z[0] + 1, P.PAD_Z[1] - 1, r=2.0, edges="|Y")
    # grip ribs on the contact face (0.4 mm deep grooves across the pad)
    y_face = y[0] if side > 0 else y[1]
    d = 0.4 * side
    grooves = [box(P.PAD_X[0] - 1, P.PAD_X[1] + 1, y_face - 0.01 * side, y_face + d, zc - 0.5, zc + 0.5)
               for zc in np.arange(-6.0, 6.01, 3.0)]   # symmetric about the TCP
    return single_solid(cut(union(slab, key), *grooves))


def pinion(teeth: bool = False) -> cq.Solid:
    zg = P.GRIP_SERVO_Z
    x0, x1 = P.PINION_X
    if not teeth:
        g = cyl(P.PINION_R, (x0, 0, zg), (x1, 0, zg))
    else:
        w = _pinion_profile(P.GEAR_MODULE, P.PINION_TEETH)
        f = cq.Face.makeFromWires(w)
        g = cq.Solid.extrudeLinear(f, cq.Vector(x1 - x0, 0, 0)).moved(cq.Location(cq.Vector(x0, 0, zg)))
    tools = _horn_plate_holes((x0, 0, zg), (1, 0, 0))
    # counterbores so the screw heads sit below the pinion face (the jaw spine slides past it)
    # one round recess for the screw heads (separate counterbores would leave < 1 mm webs)
    tools.append(cyl(P.HORN_SCREW_PCD_R + 3.1, (x0 + P.PINION_SEAT, 0, zg), (x1 + 1, 0, zg)))
    return single_solid(cut(g, *tools))


def gripper_cap() -> cq.Solid:
    """Back cap of the gripper sleeve: retains both servos (roll case top and gripper case bottom
    face it), with a window for the roll horn and the gripper servo's rear boss. 4 x M2 into the
    sleeve side walls, heads counterbored."""
    hy, hz, yc, zg, x_roll, x_grip = _grip_frame()
    w = P.SLEEVE_W
    x1 = min(x_roll[0], x_grip[0])
    x0 = x1 - P.GRIP_CAP_T
    top = zg + hz + P.SLEEVE_TOP_T
    cap = rbox(x0, x1, yc - hy - w, yc + hy + w, -hz - w, top, r=3.0, edges="|X")
    cap = fillet_all(cap, 0.8, "<X")
    tools = [cyl(S.horn_r + 0.6, (x0 - 1, 0, 0), (x1 + 1, 0, 0)),
             cyl(P.BOSS_CLR_R, (x0 - 1, 0, zg), (x1 + 1, 0, zg))]
    for y, z in gripper_cap_screw_yz():
        tools.append(cyl(1.2, (x0 - 1, y, z), (x1 + 1, y, z)))
    return single_solid(cut(cap, *tools))


def gripper_cap_screw_yz():
    hy, hz, yc, zg, _, _ = _grip_frame()
    # positions chosen outside the circle swept by the wrist front plate's corners (r ~ 31 mm), since
    # the cap rotates with the gripper in front of that plate
    ym, yp = yc - hy - P.SLEEVE_W / 2, yc + hy + P.SLEEVE_W / 2
    return [(ym, -8.0), (ym, zg + 8.0), (yp, zg + 8.0)]


def gripper_cap_screws() -> cq.Compound:
    hy, hz, yc, zg, x_roll, x_grip = _grip_frame()
    x1 = min(x_roll[0], x_grip[0])
    x_head = x1 - P.GRIP_CAP_T
    out = []
    for y, z in gripper_cap_screw_yz():
        h = _socket_screw((x_head, y, z), (-1, 0, 0), 0.0, r=1.0, head_r=1.9, head_h=1.3)
        sh = cyl(1.0 - P.SHANK_CLR, (x_head - 0.01, y, z), (x_head + 8.0, y, z))
        out.append(single_solid(union(h, sh)))
    return cq.Compound.makeCompound(out)


def gripper_cap_holes() -> list[cq.Shape]:
    """Tapped M2 holes in the sleeve end for the cap screws (line-to-line, modelled as r = 1.0)."""
    hy, hz, yc, zg, x_roll, x_grip = _grip_frame()
    x1 = min(x_roll[0], x_grip[0])
    return [cyl(1.0, (x1 - 1, y, z), (x1 + 8.0, y, z)) for y, z in gripper_cap_screw_yz()]


def cb_pulley() -> cq.Solid:
    """Counterbalance cable pulley (purchased): 12 mm OD, grooved, on the post's axle (shoulder frame)."""
    c = P.CB_PULLEY
    za = P.SHOULDER_Z + P.CB["a"]
    y0, y1 = P.CB_PLANE_Y - c["t"] / 2, P.CB_PLANE_Y + c["t"] / 2
    wheel = cyl(c["r"], (0, y0, za), (0, y1, za))
    groove = ring(c["r"] - 0.9, c["r"] + 1, (0, P.CB_PLANE_Y - 0.6, za), (0, P.CB_PLANE_Y + 0.6, za))
    bore = cyl(c["axle_r"] - P.SHANK_CLR, (0, y0 - 1, za), (0, y1 + 1, za))   # runs on the pin
    return single_solid(cut(wheel, groove, bore))


def cb_axle() -> cq.Solid:
    """M3 shoulder pin through the post and pulley, head on the outer cheek."""
    za = P.SHOULDER_Z + P.CB["a"]
    y0, y1 = P.CB["post_y"]
    pin = cyl(P.CB_PULLEY["axle_r"] - P.SHANK_CLR, (0, y0, za), (0, y1 + 0.01, za))
    head = cyl(2.75, (0, y1, za), (0, y1 + 2.0, za))
    return single_solid(union(pin, head))


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
    "gripper_cap": (gripper_cap, "gripper", "PETG"),
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
