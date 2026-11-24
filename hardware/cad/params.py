"""Parametric dimensions for the printed arm. Units: millimetres, grams.

Everything that is a kinematic dimension is read from spec/kinematics.yaml (the single source of
truth); everything else (wall thickness, clearances, servo envelope) is a design parameter here.

SERVO: the STS3215 envelope, output-axis offset, rear boss and horn come from the Feetech datasheet
(see Servo). Cosmetic case detail is not to drawing; check a real servo with calipers before printing.
"""
from __future__ import annotations

from dataclasses import dataclass

from spec_loader import joint, load_spec, mm

_spec = load_spec()


# ---------------------------------------------------------------- kinematic dimensions (from spec)
YAW_Z = mm(joint("base_yaw")["origin"][2])            # base plate -> yaw horn face        (40)
SHOULDER_Z = mm(joint("shoulder_pitch")["origin"][2])  # yaw horn face -> shoulder axis     (60)
UPPER_ARM_L = mm(joint("elbow_pitch")["origin"][0])    # shoulder axis -> elbow axis        (200)
FOREARM_L = mm(joint("wrist_pitch")["origin"][0])      # elbow axis -> wrist pitch axis     (180)
WRIST_L = mm(joint("wrist_roll")["origin"][0])         # wrist pitch axis -> roll horn face (45)
TCP_L = mm(_spec["tool"]["tcp"][0])                    # roll horn face -> TCP              (65)
GRIP_LIMITS = tuple(_spec["gripper"]["limits"])        # rad
GRIP_OPEN_GAP = mm(_spec["gripper"]["opening_m_at_max"])  # 60 mm at 1.60 rad


# ---------------------------------------------------------------- servo (Feetech STS3215, datasheet)
@dataclass(frozen=True)
class Servo:
    """STS3215 envelope from the Feetech product specification (rev. A/0, 2020-04-10, section 9
    'Outside dimension' and section 10 'Accessories'):
    case 45.23 x 24.73 x 35 mm; output spline 25T / OD 5.9 mm, 12.5 mm from the case centre along the
    length; rear idler boss coaxial with the output (dia ~6, 4.1 mm proud of the case bottom);
    aluminium horn dia 19.95 mm, 4 x M3 tapped on a 14 mm PCD, 2.5 mm flange + hub, 4.5 mm overall.
    Fine detail (ear shape, screw bosses) is styling; every mating dimension above is from the drawing."""
    L: float = 45.23     # case length
    W: float = 24.73     # case width
    H: float = 35.0      # case height (bottom face to top face), horn axis direction
    horn_r: float = 19.95 / 2   # horn flange radius
    horn_t: float = 4.5         # horn overall height (hub on the servo side + 2.5 mm flange)
    horn_flange_t: float = 2.5
    horn_hub_r: float = 4.5
    horn_offset: float = 12.5   # output axis is this far from the case centre, towards +X (servo-local)
    boss_r: float = 3.0         # rear idler boss radius (dia 6)
    boss_h: float = 4.1         # rear idler boss height below the case bottom
    mass_g: float = 55.0  # approx. STS3215 mass (verify on a scale)

    @property
    def stack(self) -> float:
        return self.H + self.horn_t

    @property
    def cx(self) -> float:
        """Case centre along servo-local X (the output axis is at X = 0)."""
        return -self.horn_offset


SERVO = Servo()

# ---------------------------------------------------------------- general print rules
WALL_MIN = 2.0          # minimum printable wall (checked by tests)
CLR = 0.5               # running clearance between parts that move relative to each other
POCKET_CLR = 0.2        # servo pocket clearance per side
WALL = 3.0              # nominal cradle wall
PLATE_T = 4.0           # clevis side-plate thickness
CAP_T = 3.5             # servo bottom cap (idler side); holds 4 x M2 low-head screws into the case bottom

# ---------------------------------------------------------------- pitch-joint layer stack (y, joint frame)
# Servo horn axis = joint +y. Horn mating face at y = +stack/2.
Y_HORN = SERVO.stack / 2                      # 19.0  horn face (child horn plate bolts here)
Y_BODY_TOP = Y_HORN - SERVO.horn_t            # 16.0  case top
Y_BODY_BOT = Y_HORN - SERVO.stack             # -19.0 case bottom
Y_CAP_BOT = Y_BODY_BOT - CAP_T                # -21.5 parent cap outer face
# child clevis plates (outer layer)
Y_HPLATE = (Y_HORN, Y_HORN + PLATE_T)                        # (19, 23)
Y_IPLATE = (Y_CAP_BOT - CLR - PLATE_T, Y_CAP_BOT - CLR)      # (-26, -22)
# parent inner layer (cradle / neck / pedestal) must stay strictly inside this band
Y_INNER = (Y_CAP_BOT, Y_BODY_TOP)                            # (-21.5, 16)
# Idler side: the parent's printed stub carries a 6700-2RS bearing (10 x 15 x 4) pressed into the
# child's idler plate. The bearing floats axially on the stub (non-locating side); the child is located
# axially by the horn plate bolted to the servo horn (locating side).
IDLER_BEARING = dict(name="6701-2RS (12x18x4)", d=12.0, D=18.0, B=4.0, mass_g=3.5)  # 12 mm bore leaves a 2.6 mm stub wall around the servo boss
STUB_R = IDLER_BEARING["d"] / 2          # 6.0
IDLER_BORE_R = IDLER_BEARING["D"] / 2    # 9.0 (bearing seat in the child idler plate)
BOSS_CLR_R = 3.4        # clearance hole/pocket for the servo's rear boss in cap and stub
HORN_BORE_R = 3.0       # centre bore in the horn plate: holds the M3 spline-screw head
HORN_SCREW_PCD_R = 7.0  # 14 mm PCD (datasheet)
HORN_SCREW_R = 1.7      # M3 clearance
M3_R = 1.5              # M3 shank, modelled line-to-line in tapped/threaded holes
R_CLEVIS = 26.0         # clevis disc radius around the joint axis

# cradle box around a pitch servo (joint frame: L along x, W along z)
CRADLE_X = SERVO.L / 2 + POCKET_CLR + WALL    # 25.8 (half-length; cradle centred on the case, not the axis)
CRADLE_Z = SERVO.W / 2 + POCKET_CLR + WALL    # 15.55


@dataclass(frozen=True)
class LinkGeom:
    length: float       # joint-axis to joint-axis
    plate_h: float      # half-height of the twin side plates
    plate_end: float    # x where the outer-layer plates stop (must be clear of the next joint's sweep)
    bridge_len: float   # length of the full-width bridge that joins plates to the inner neck
    neck_h: float       # half-height of the inner-layer neck


UPPER_ARM = LinkGeom(UPPER_ARM_L, plate_h=14.0, plate_end=UPPER_ARM_L - 78.0, bridge_len=14.0, neck_h=11.0)
FOREARM = LinkGeom(FOREARM_L, plate_h=13.0, plate_end=FOREARM_L - 50.0, bridge_len=14.0, neck_h=10.0)

# ---------------------------------------------------------------- base / yaw
BEARING = dict(name="61816-2RS (thin-section, 80x100x10)", d=80.0, D=100.0, B=10.0, mass_g=150.0)
# (was 61812, 60 mm bore: the real STS3215 case, offset 12.5 mm from its output, reaches r = 37.5 mm)
BASE_R = BEARING["D"] / 2 + 3.5   # bearing seat + 3.5 mm wall
BASE_TOP = YAW_Z - 0.5            # base housing top sits 0.5 below the turntable (no rubbing)
BASE_PLATE = (140.0, 120.0, 6.0)  # x, y, thickness
CLAMP_TAB = dict(y_out=95.0, x_half=20.0, t=8.0, hole_r=2.75)  # M5 clearance, or C-clamp pad
TURNTABLE_T = 7.0
# The yaw horn sits in a counterbore in the turntable underside, so the horn face is YAW_HORN_UP above
# the yaw frame origin (the turntable underside / bearing top, z = YAW_Z). This keeps the real 39.5 mm
# servo+horn stack, a 4.5 mm pocket floor and the rear boss above the desk without changing the spec
# kinematics (the spec origin is the bearing, not the horn).
YAW_HORN_UP = 4.0
HORN_CB_R = SERVO.horn_r + 0.4   # turntable counterbore for the horn flange
TURNTABLE_R = BEARING["d"] / 2 + 6.0   # covers the inner race shoulder; smaller = more shoulder-down range
SPIGOT_R = (BEARING["d"] / 2 - 2.4, BEARING["d"] / 2 - 0.1)  # inner-race spigot ring

# ---------------------------------------------------------------- shoulder counterbalance
CB = dict(
    a=40.0,           # pulley anchor A height above shoulder axis (shoulder bracket post)
    b=70.0,           # pin B distance along the upper arm from shoulder axis
    post_y=(27.5, 35.5),   # clear of the M3 horn-screw heads on the upper-arm horn plate (y <= 26.75)
    post_x=(-10.0, 10.0),
    pin_r=4.0,
    pin_y=(Y_HPLATE[1], 37.0),
)

# ---------------------------------------------------------------- wrist bracket
WRIST_PLATE_H = 16.0
WRIST_FRONT_T = 5.0
SLEEVE_W = 6.5          # gripper servo-sleeve side walls (carry the M2 cap screws with >= 2 mm wall)
SLEEVE_TOP_T = 6.0      # sleeve top wall (camera-mount M3 screws thread 3.5 mm into it)

# ---------------------------------------------------------------- gripper (frame = roll joint)
GEAR_MODULE = 1.5
PINION_TEETH = 25
PINION_R = GEAR_MODULE * PINION_TEETH / 2     # 18.75 pitch radius -> 30 mm/jaw at 1.60 rad
GRIP_SERVO_Z = SERVO.W / 2 + POCKET_CLR + 2.0 + POCKET_CLR + SERVO.W / 2  # stacked above roll servo
RACK_X = (45.0, 53.0)   # moved +4 mm (2026-11-24) so the roll servo's rear boss clears the lower rack
RACK_T = 8.0            # rack bar thickness (z)
PINION_X = (45.0, 52.0)   # bolted to the gripper servo horn face (x=45)
FINGER_X0 = 53.8        # finger body starts here (root tab from RACK_X[1])
FINGER_X1 = 80.0
FINGER_T = 6.0          # finger thickness (y), excluding pad
PAD_T = 2.0             # TPU pad thickness
PAD_X = (TCP_L - 10.0, TCP_L + 10.0)
PAD_Z = (-10.0, 10.0)
FINGER_ZMIN = -12.0
JAW_TRAVEL = PINION_R * GRIP_LIMITS[1]       # per jaw, at full open

DENSITY = dict(PETG=1.27, TPU=1.21)  # g/cm3
INFILL_EFF = 0.40                     # effective solid fraction (walls + infill)
PRINT_BED = (220.0, 220.0, 250.0)

# ---------------------------------------------------------------- 2026-11-24 polish additions
GEAR_BACKLASH = 0.10      # mm per flank at the pitch line (pinion and racks each thinned)
FINGER_Z = (-12.0, 12.0)  # finger body z range (covers the pad, PAD_Z, with 2 mm margin)
CAM_TILT = 15.0           # deg, camera face tilt (down towards the fingertips)
CAM_FACE_H = 38.0         # tilted face height (z)
CAM_SCREW_X = (22.0, 32.0)
PINION_SEAT = 3.9         # horn-screw head seat depth in the pinion (heads flush with its face)
SERVO_SCREW = dict(r=1.0, head_r=1.9, head_h=1.3, cb=1.5, len=6.0, inset=3.3)  # M2 low-head into case bottom
CB_PULLEY = dict(r=6.0, t=2.6, axle_r=1.5, name="12 mm grooved idler pulley on M3 shoulder pin")
CB_PULLEY_SLOT = (29.9, 33.1)
CB_PLANE_Y = 31.5        # cable plane (pulley mid-plane = pin groove)
GRIP_CAP_T = 3.0          # gripper back cap (retains both gripper-body servos)
SHANK_CLR = 0.05          # radial gap between a modelled screw shank and its tapped hole (avoids coincident faces)
GUIDE_CLR = 0.2           # rack-to-guide running clearance (was 0.5)
