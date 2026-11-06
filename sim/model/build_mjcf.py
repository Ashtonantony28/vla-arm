"""Generate the MuJoCo MJCF for the VLA arm + tabletop scene from spec/kinematics.yaml.

    python -m sim.model.build_mjcf            # writes sim/model/vla_arm.xml
    python -m sim.model.build_mjcf --check    # exit 1 if the committed XML is stale

Every geometric/inertial/limit number of the arm comes from the spec. Numbers that the spec
does not define (link radii, finger shape, object sizes, actuator gain, armature) are named
constants below with a comment on where they come from.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from xml.sax.saxutils import quoteattr

import numpy as np

from sim.spec import ACTION_NAMES, Spec, load_spec

OUT_PATH = Path(__file__).with_name("vla_arm.xml")

# --- simulation constants (not in the spec) ------------------------------------------------
TIMESTEP = 1.0 / 600.0          # 20 physics substeps per 30 Hz control step
# Servo position loop: torque saturates at stall after this much position error. The STS3215
# P-loop saturates within ~1 deg; this keeps gravity sag at full stretch below ~0.5 deg.
SERVO_SATURATION_ERR_RAD = 0.02
# Reflected rotor inertia of a ~1:345 geared servo (rotor ~1e-7 kg m^2 * 345^2 ~ 0.012).
SERVO_ARMATURE = 0.01
JOINT_DAMPING = 0.05            # gearbox viscous friction, N m s / rad
# Gripper: firmware limits gripper holding torque to the continuous-current rating so the
# servo does not cook while holding an object (stall * I_cont / I_stall).
GRIPPER_LIMIT_TO_CONTINUOUS = True

LINK_RADIUS = {"upper_arm": 0.022, "forearm": 0.020, "wrist": 0.022}
FINGER_LEN, FINGER_HALF_THICK, FINGER_HALF_HEIGHT = 0.050, 0.006, 0.012
PALM_HALF = (0.015, 0.038, 0.022)

BLOCK_HALF = 0.015              # 30 mm cubes
BLOCK_MASS = 0.030              # PLA cube, ~33 g
BOWL_RADIUS, BOWL_WALL_H, BOWL_WALL_T = 0.058, 0.032, 0.004
CUP_RADIUS, CUP_HALF_H, CUP_MASS = 0.030, 0.035, 0.050
TARGET_RADIUS = 0.035

PARK_X = 3.0                    # inactive objects are parked off-table at x = PARK_X

COLORS = {
    "red": "0.85 0.15 0.15 1",
    "blue": "0.15 0.3 0.85 1",
    "green": "0.2 0.7 0.3 1",
    "yellow": "0.95 0.8 0.15 1",
    "cup": "0.9 0.9 0.92 1",
    "target": "0.6 0.2 0.7 1",
    "arm": "0.25 0.25 0.28 1",
    "servo": "0.1 0.1 0.1 1",
    "finger": "0.95 0.5 0.1 1",
}
BLOCKS = ["red", "blue"]
BOWLS = ["green", "yellow"]


def f(*xs: float) -> str:
    return " ".join(f"{x:.6g}" for x in xs)


def look_at_xyaxes(pos, target, up=(0.0, 0.0, 1.0)) -> str:
    pos, target, up = (np.asarray(v, float) for v in (pos, target, up))
    fwd = target - pos
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    upv = np.cross(right, fwd)
    return f(*right, *upv)


def link_kv(kp: float, inertia: float) -> float:
    """Critically damped servo loop for a crude effective inertia."""
    return 2.0 * math.sqrt(kp * inertia)


def effective_inertias(spec: Spec) -> list[float]:
    """Point-mass estimate of the inertia each joint sees at zero pose (+ armature)."""
    j = {jt.name: jt for jt in spec.joints}
    m = spec.masses
    # distance of each link's COM from the base along x at zero pose
    x_sh = 0.0
    x_ua = j["elbow_pitch"].origin[0] / 2
    x_fa = j["elbow_pitch"].origin[0] + j["wrist_pitch"].origin[0] / 2
    x_wr = j["elbow_pitch"].origin[0] + j["wrist_pitch"].origin[0] + j["wrist_roll"].origin[0] / 2
    x_gr = x_wr + j["wrist_roll"].origin[0] / 2 + spec.tcp[0] / 2
    chain = [(m["upper_arm"], x_ua), (m["forearm"], x_fa), (m["wrist"], x_wr), (m["gripper"], x_gr)]
    out = []
    for name, pivot in [("base_yaw", 0.0), ("shoulder_pitch", 0.0),
                        ("elbow_pitch", j["elbow_pitch"].origin[0]),
                        ("wrist_pitch", j["elbow_pitch"].origin[0] + j["wrist_pitch"].origin[0])]:
        out.append(SERVO_ARMATURE + sum(mm * (x - pivot) ** 2 for mm, x in chain if x > pivot))
    out.append(SERVO_ARMATURE + 1e-4)  # wrist roll: tiny link inertia about x
    return out


def build(spec: Spec) -> str:
    J = {jt.name: jt for jt in spec.joints}
    m = spec.masses
    kp = spec.stall_torque / SERVO_SATURATION_ERR_RAD
    inertias = effective_inertias(spec)

    L_ua = J["elbow_pitch"].origin[0]
    L_fa = J["wrist_pitch"].origin[0]
    L_wr = J["wrist_roll"].origin[0]
    tcp = spec.tcp
    finger_travel = 0.5 * spec.gripper_opening_at_max          # per finger, metres
    grip_gear = spec.gripper_limits[1] / finger_travel           # actuator "rad" per finger metre
    grip_force = spec.stall_torque * (spec.max_cont_current / spec.stall_current
                                      if GRIPPER_LIMIT_TO_CONTINUOUS else 1.0)
    grip_kp = grip_force / 0.1                                   # saturates at 0.1 rad error
    grip_inertia = SERVO_ARMATURE  # rotary armature reflected to finger slide below
    finger_armature = SERVO_ARMATURE * grip_gear ** 2           # servo rotor inertia reflected to the slide

    def joint_xml(name: str, extra: str = "") -> str:
        jt = J[name]
        return (f'<joint name="{name}" type="hinge" axis="{f(*jt.axis)}" range="{f(*jt.limits)}" '
                f'armature="{SERVO_ARMATURE}" damping="{JOINT_DAMPING}" limited="true" {extra}/>')

    base_h = J["base_yaw"].origin[2]
    yaw_to_sh = J["shoulder_pitch"].origin[2]
    finger_x0 = tcp[0] + 0.010 - FINGER_LEN / 2  # fingertips 10 mm beyond the TCP

    # wrist camera on the gripper body, behind the palm, looking 40 deg "down" toward the jaws
    a = math.radians(40)
    wrist_cam_xy = f(0, -1, 0, math.sin(a), 0, math.cos(a))
    scene_pos = (0.62, -0.32, 0.44)
    scene_xy = look_at_xyaxes(scene_pos, (0.18, 0.02, 0.05))

    bowl_walls = []
    n_wall = 16
    for i in range(n_wall):
        th = 2 * math.pi * i / n_wall
        r = BOWL_RADIUS - BOWL_WALL_T
        half_len = BOWL_RADIUS * math.tan(math.pi / n_wall) + 0.001
        bowl_walls.append((r * math.cos(th), r * math.sin(th), th, half_len))

    def bowl_xml(color: str) -> str:
        walls = "\n".join(
            f'        <geom type="box" size="{f(BOWL_WALL_T, hl, BOWL_WALL_H / 2)}" '
            f'pos="{f(x, y, BOWL_WALL_H / 2)}" euler="{f(0, 0, th)}" class="obj" rgba="{COLORS[color]}"/>'
            for x, y, th, hl in bowl_walls)
        return f'''    <body name="{color}_bowl" mocap="true" pos="{f(PARK_X, 0.4 + 0.3 * BOWLS.index(color), 0)}">
        <geom name="{color}_bowl_bottom" type="cylinder" size="{f(BOWL_RADIUS, 0.002)}" pos="0 0 0.002" class="obj" rgba="{COLORS[color]}"/>
{walls}
        <site name="{color}_bowl_center" pos="0 0 0.004" size="0.004" rgba="0 0 0 0"/>
    </body>'''

    def block_xml(color: str) -> str:
        return f'''    <body name="{color}_block" pos="{f(PARK_X, -0.4 - 0.3 * BLOCKS.index(color), BLOCK_HALF)}">
      <freejoint name="{color}_block_free"/>
      <geom name="{color}_block_geom" type="box" size="{f(BLOCK_HALF, BLOCK_HALF, BLOCK_HALF)}" mass="{BLOCK_MASS}" class="obj" rgba="{COLORS[color]}"/>
    </body>'''

    act_lines = []
    for name, inertia in zip(ACTION_NAMES[:5], inertias):
        jt = J[name]
        act_lines.append(
            f'    <position name="{name}" joint="{name}" kp="{kp:.6g}" kv="{link_kv(kp, inertia):.4g}" '
            f'ctrlrange="{f(*jt.limits)}" forcerange="{f(-spec.stall_torque, spec.stall_torque)}" forcelimited="true"/>')
    act_lines.append(
        f'    <position name="gripper" joint="finger_l" gear="{grip_gear:.6g}" kp="{grip_kp:.6g}" '
        f'kv="{link_kv(grip_kp, grip_inertia):.4g}" ctrlrange="{f(*spec.gripper_limits)}" '
        f'forcerange="{f(-grip_force, grip_force)}" forcelimited="true"/>')
    actuators = "\n".join(act_lines)

    xml = f'''<!-- AUTO-GENERATED by sim/model/build_mjcf.py from spec/kinematics.yaml. Do not edit by hand. -->
<mujoco model="vla_arm">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{TIMESTEP:.8g}" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <size memory="20M"/>
  <visual>
    <global offwidth="640" offheight="480"/>
    <quality shadowsize="2048"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5" specular="0.1 0.1 0.1"/>
  </visual>
  <default>
    <default class="arm">
      <geom contype="2" conaffinity="1" rgba="{COLORS["arm"]}" friction="0.8 0.02 0.001"/>
    </default>
    <default class="obj">
      <geom contype="1" conaffinity="3" condim="4" friction="1.2 0.02 0.001" solref="0.004 1" solimp="0.95 0.99 0.001"/>
    </default>
    <default class="finger">
      <geom contype="2" conaffinity="1" condim="4" friction="1.5 0.02 0.001" solref="0.004 1" solimp="0.95 0.99 0.001" rgba="{COLORS["finger"]}"/>
    </default>
  </default>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.78 0.74 0.68" rgb2="0.72 0.68 0.62" width="256" height="256"/>
    <material name="table" texture="grid" texrepeat="6 6" reflectance="0.05"/>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.6 0.65 0.7" rgb2="0.3 0.3 0.35" width="64" height="64"/>
  </asset>
  <worldbody>
    <light name="key" pos="0.6 -0.4 1.2" dir="-0.4 0.3 -1" diffuse="0.6 0.6 0.6" castshadow="true"/>
    <light name="fill" pos="-0.3 0.5 1.0" dir="0.3 -0.4 -1" diffuse="0.25 0.25 0.25" castshadow="false"/>
    <!-- table top is the z=0 plane (base plate sits on it); collision plane is infinite -->
    <geom name="table" type="plane" size="0.6 0.6 0.01" pos="0.2 0 0" material="table" contype="1" conaffinity="3" friction="0.6 0.01 0.001"/>
    <camera name="scene" pos="{f(*scene_pos)}" xyaxes="{scene_xy}" fovy="50"/>

    <body name="base_link" pos="0 0 0">
      <geom name="base_plate" type="cylinder" size="{f(0.06, base_h / 2)}" pos="{f(0, 0, base_h / 2)}" mass="{m["base_link"]}" class="arm"/>
      <site name="base" pos="0 0 0" size="0.005" group="4"/>
      <body name="shoulder_link" pos="{f(*J["base_yaw"].origin)}">
        {joint_xml("base_yaw")}
        <geom name="turret" type="cylinder" size="{f(0.035, yaw_to_sh / 2)}" pos="{f(0, 0, yaw_to_sh / 2)}" mass="{m["shoulder_link"]}" class="arm" rgba="{COLORS["servo"]}"/>
        <body name="upper_arm" pos="{f(*J["shoulder_pitch"].origin)}">
          {joint_xml("shoulder_pitch")}
          <geom name="upper_arm_geom" type="capsule" fromto="{f(0, 0, 0, L_ua, 0, 0)}" size="{LINK_RADIUS["upper_arm"]}" mass="{m["upper_arm"]}" class="arm"/>
          <body name="forearm" pos="{f(*J["elbow_pitch"].origin)}">
            {joint_xml("elbow_pitch")}
            <geom name="forearm_geom" type="capsule" fromto="{f(0, 0, 0, L_fa, 0, 0)}" size="{LINK_RADIUS["forearm"]}" mass="{m["forearm"]}" class="arm"/>
            <body name="wrist" pos="{f(*J["wrist_pitch"].origin)}">
              {joint_xml("wrist_pitch")}
              <geom name="wrist_geom" type="box" size="{f(L_wr / 2, LINK_RADIUS["wrist"], LINK_RADIUS["wrist"])}" pos="{f(L_wr / 2, 0, 0)}" mass="{m["wrist"]}" class="arm" rgba="{COLORS["servo"]}"/>
              <body name="gripper" pos="{f(*J["wrist_roll"].origin)}">
                {joint_xml("wrist_roll")}
                <geom name="palm" type="box" size="{f(*PALM_HALF)}" pos="{f(PALM_HALF[0], 0, 0)}" mass="{m["gripper"] * 0.7:.4g}" class="arm"/>
                <site name="tcp" pos="{f(*tcp)}" size="0.004" rgba="1 0 1 0.5" group="4"/>
                <camera name="wrist" pos="{f(0.015, 0, PALM_HALF[2] + 0.022)}" xyaxes="{wrist_cam_xy}" fovy="80"/>
                <body name="finger_l" pos="0 0 0">
                  <joint name="finger_l" type="slide" axis="0 1 0" range="{f(0, finger_travel)}" armature="{finger_armature:.4g}" damping="1"/>
                  <geom name="finger_l_geom" type="box" size="{f(FINGER_LEN / 2, FINGER_HALF_THICK, FINGER_HALF_HEIGHT)}" pos="{f(finger_x0, FINGER_HALF_THICK, 0)}" mass="{m["gripper"] * 0.15:.4g}" class="finger"/>
                </body>
                <body name="finger_r" pos="0 0 0">
                  <joint name="finger_r" type="slide" axis="0 -1 0" range="{f(0, finger_travel)}" armature="{finger_armature:.4g}" damping="1"/>
                  <geom name="finger_r_geom" type="box" size="{f(FINGER_LEN / 2, FINGER_HALF_THICK, FINGER_HALF_HEIGHT)}" pos="{f(finger_x0, -FINGER_HALF_THICK, 0)}" mass="{m["gripper"] * 0.15:.4g}" class="finger"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>

{chr(10).join(block_xml(c) for c in BLOCKS)}
{chr(10).join(bowl_xml(c) for c in BOWLS)}
    <body name="cup" pos="{f(PARK_X, 1.4, CUP_HALF_H)}">
      <freejoint name="cup_free"/>
      <geom name="cup_geom" type="cylinder" size="{f(CUP_RADIUS, CUP_HALF_H)}" mass="{CUP_MASS}" class="obj" rgba="{COLORS["cup"]}"/>
    </body>
    <body name="target" mocap="true" pos="{f(PARK_X, 1.8, 0)}">
      <geom name="target_geom" type="cylinder" size="{f(TARGET_RADIUS, 0.0005)}" pos="0 0 0.0005" contype="0" conaffinity="0" rgba="{COLORS["target"]}"/>
    </body>
  </worldbody>

  <contact>
    <exclude body1="finger_l" body2="finger_r"/>
    <exclude body1="gripper" body2="finger_l"/>
    <exclude body1="gripper" body2="finger_r"/>
  </contact>
  <equality>
    <joint name="finger_coupling" joint1="finger_r" joint2="finger_l" polycoef="0 1 0 0 0" solref="0.005 1"/>
  </equality>
  <actuator>
{actuators}
  </actuator>
</mujoco>
'''
    return xml


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="fail if the written XML is stale")
    ap.add_argument("--out", type=Path, default=OUT_PATH)
    args = ap.parse_args(argv)
    xml = build(load_spec())
    if args.check:
        current = args.out.read_text() if args.out.exists() else ""
        if current != xml:
            print(f"{args.out} is stale; run python -m sim.model.build_mjcf", file=sys.stderr)
            return 1
        print("up to date")
        return 0
    args.out.write_text(xml)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
