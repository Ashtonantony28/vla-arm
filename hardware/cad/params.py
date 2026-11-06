"""Parametric dimensions for the printed arm. Units: millimetres, grams.

Everything that is a kinematic dimension is read from spec/kinematics.yaml (the single source of
truth); everything else (wall thickness, clearances, servo envelope) is a design parameter here.

SERVO ENVELOPE WARNING: the STS3215-class servo envelope below (45.2 x 24.7 x 35 mm, horn on top
centre) is an APPROXIMATION for layout only. The real STS3215 output shaft is not necessarily at the
case centre, the case has mounting ears/screw bosses, and horn diameter/thickness vary by horn type.
Measure a real servo with calipers and update SERVO before printing anything.
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


# ---------------------------------------------------------------- servo placeholder (APPROXIMATE)
@dataclass(frozen=True)
class Servo:
    L: float = 45.2      # case length (horn centred along L in this placeholder)
    W: float = 24.7      # case width
    H: float = 35.0      # case height, horn axis direction
    horn_r: float = 10.0  # horn disc radius (approx.)
    horn_t: float = 3.0   # horn disc thickness (approx.)
    horn_offset: float = 0.0  # output-shaft offset from case centre along L; 0 = placeholder
    mass_g: float = 55.0  # approx. STS3215 mass (verify on a scale)

    @property
    def stack(self) -> float:
        return self.H + self.horn_t


SERVO = Servo()

# ---------------------------------------------------------------- general print rules
WALL_MIN = 2.0          # minimum printable wall (checked by tests)
CLR = 0.5               # running clearance between parts that move relative to each other
POCKET_CLR = 0.2        # servo pocket clearance per side
WALL = 3.0              # nominal cradle wall
PLATE_T = 4.0           # clevis side-plate thickness
CAP_T = 2.5             # servo bottom cap (idler side) thickness

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
STUB_R = 4.0            # idler stub on parent (coaxial with servo output), through child idler bore
IDLER_BORE_R = STUB_R + 0.4
HORN_BORE_R = 3.0       # centre bore in the horn plate (horn screw access)
HORN_SCREW_PCD_R = 7.0  # approx. horn screw pattern radius (verify against real horn)
HORN_SCREW_R = 1.3      # M2.5 clearance-ish
R_CLEVIS = 26.0         # clevis disc radius around the joint axis

# cradle box around a pitch servo (joint frame: L along x, W along z)
CRADLE_X = SERVO.L / 2 + POCKET_CLR + WALL    # 25.8
CRADLE_Z = SERVO.W / 2 + POCKET_CLR + WALL    # 15.55


@dataclass(frozen=True)
class LinkGeom:
    length: float       # joint-axis to joint-axis
    plate_h: float      # half-height of the twin side plates
    plate_end: float    # x where the outer-layer plates stop (must be clear of the next joint's sweep)
    bridge_len: float   # length of the full-width bridge that joins plates to the inner neck
    neck_h: float       # half-height of the inner-layer neck


UPPER_ARM = LinkGeom(UPPER_ARM_L, plate_h=14.0, plate_end=UPPER_ARM_L - 78.0, bridge_len=14.0, neck_h=11.0)
FOREARM = LinkGeom(FOREARM_L, plate_h=13.0, plate_end=FOREARM_L - 40.0, bridge_len=14.0, neck_h=10.0)

# ---------------------------------------------------------------- base / yaw
BEARING = dict(name="61812-2RS (thin-section, 60x78x10)", d=60.0, D=78.0, B=10.0, mass_g=110.0)
BASE_R = 42.5          # bearing seat + 3.5 mm wall; kept small so the upper arm can pitch down further
BASE_TOP = YAW_Z - 0.5            # base housing top sits 0.5 below the turntable (no rubbing)
BASE_PLATE = (140.0, 120.0, 6.0)  # x, y, thickness
CLAMP_TAB = dict(y_out=95.0, x_half=20.0, t=8.0, hole_r=2.75)  # M5 clearance, or C-clamp pad
TURNTABLE_T = 6.0
TURNTABLE_R = 36.0       # covers the inner race (r <= 33) + counterbalance post foot; smaller = more shoulder-down range
SPIGOT_R = (BEARING["d"] / 2 - 2.4, BEARING["d"] / 2 - 0.1)  # inner-race spigot ring

# ---------------------------------------------------------------- shoulder counterbalance
CB = dict(
    a=40.0,           # pulley anchor A height above shoulder axis (shoulder bracket post)
    b=70.0,           # pin B distance along the upper arm from shoulder axis
    post_y=(26.0, 34.0),
    post_x=(-10.0, 10.0),
    pin_r=4.0,
    pin_y=(Y_HPLATE[1], 35.0),
)

# ---------------------------------------------------------------- wrist bracket
WRIST_PLATE_H = 16.0
WRIST_FRONT_T = 5.0

# ---------------------------------------------------------------- gripper (frame = roll joint)
GEAR_MODULE = 1.5
PINION_TEETH = 25
PINION_R = GEAR_MODULE * PINION_TEETH / 2     # 18.75 pitch radius -> 30 mm/jaw at 1.60 rad
GRIP_SERVO_Z = SERVO.W / 2 + POCKET_CLR + 2.0 + POCKET_CLR + SERVO.W / 2  # stacked above roll servo
RACK_X = (41.0, 49.0)
RACK_T = 8.0            # rack bar thickness (z)
PINION_X = (41.0, 48.0)   # bolted to the gripper servo horn face (x=41)
FINGER_X0 = 49.8        # finger body starts here (root tab from RACK_X[1])
FINGER_X1 = 78.0
FINGER_T = 6.0          # finger thickness (y), excluding pad
PAD_T = 2.0             # TPU pad thickness
PAD_X = (TCP_L - 10.0, TCP_L + 10.0)
PAD_Z = (-10.0, 10.0)
FINGER_ZMIN = -12.0
JAW_TRAVEL = PINION_R * GRIP_LIMITS[1]       # per jaw, at full open

DENSITY = dict(PETG=1.27, TPU=1.21)  # g/cm3
INFILL_EFF = 0.40                     # effective solid fraction (walls + infill)
PRINT_BED = (220.0, 220.0, 250.0)
