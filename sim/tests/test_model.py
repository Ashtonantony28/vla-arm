"""MJCF generation, zero pose, actuator directions, FK/IK consistency with MuJoCo."""
import math

import mujoco
import numpy as np
import pytest

from sim.kinematics import TopDownIK, fk
from sim.model import build_mjcf
from sim.spec import ARM_JOINTS, load_spec


@pytest.fixture(scope="module")
def model():
    return mujoco.MjModel.from_xml_string(build_mjcf.build(load_spec()))


def test_xml_is_up_to_date():
    assert build_mjcf.main(["--check"]) == 0, "run: python -m sim.model.build_mjcf"


def test_loads_and_dimensions(model):
    assert model.nu == 6
    assert [model.actuator(i).name for i in range(6)] == ARM_JOINTS + ["gripper"]
    for cam in ("wrist", "scene"):
        assert model.camera(cam).id >= 0
    for b in ("red_block", "blue_block", "green_bowl", "yellow_bowl", "cup", "target"):
        assert model.body(b).id >= 0
    assert model.geom("table").id >= 0


def test_masses_match_spec(model):
    spec = load_spec()
    body_of = {"base_link": "base_link", "shoulder_link": "shoulder_link", "upper_arm": "upper_arm",
               "forearm": "forearm", "wrist": "wrist"}
    for k, b in body_of.items():
        assert model.body_mass[model.body(b).id] == pytest.approx(spec.masses[k], rel=1e-6)
    grip = sum(model.body_mass[model.body(b).id] for b in ("gripper", "finger_l", "finger_r"))
    assert grip == pytest.approx(spec.masses["gripper"], rel=1e-3)


def test_zero_pose_tcp(model):
    d = mujoco.MjData(model)
    mujoco.mj_forward(model, d)
    tcp = d.site("tcp").xpos - d.site("base").xpos
    assert tcp[0] == pytest.approx(0.490, abs=1e-6)
    assert tcp[1] == pytest.approx(0.0, abs=1e-6)
    assert tcp[2] == pytest.approx(0.100, abs=1e-6)


def test_actuator_limits_match_servo(model):
    spec = load_spec()
    for i in range(5):
        assert model.actuator_forcerange[i, 1] == pytest.approx(spec.stall_torque)
        assert tuple(model.actuator_ctrlrange[i]) == pytest.approx(spec.joints[i].limits)
    assert tuple(model.actuator_ctrlrange[5]) == pytest.approx(spec.gripper_limits)


def _tcp_after(model, q6):
    d = mujoco.MjData(model)
    d.qpos[:5] = q6[:5]
    d.ctrl[:] = q6
    for _ in range(600):
        mujoco.mj_step(model, d)
    return d


@pytest.mark.parametrize("j", range(5))
def test_actuator_moves_documented_direction(model, j):
    """Positive command on each actuator moves its own joint positive, and the TCP the way the
    spec's sign convention says (yaw: +x toward +y; pitch: down; roll: jaw line rotates about x)."""
    base = np.array([0.0, -0.8, 0.9, 0.5, 0.0, 0.8])  # TCP well above the table
    d0 = _tcp_after(model, base)
    q = base.copy()
    q[j] += 0.2
    d1 = _tcp_after(model, q)
    jid = model.joint(ARM_JOINTS[j]).qposadr[0]
    assert d1.qpos[jid] - d0.qpos[jid] == pytest.approx(0.2, abs=0.02)
    others = [model.joint(n).qposadr[0] for k, n in enumerate(ARM_JOINTS) if k != j]
    assert np.allclose(d1.qpos[others], d0.qpos[others], atol=0.02)
    p0, p1 = d0.site("tcp").xpos, d1.site("tcp").xpos
    R0 = d0.site("tcp").xmat.reshape(3, 3)
    R1 = d1.site("tcp").xmat.reshape(3, 3)
    name = ARM_JOINTS[j]
    if name == "base_yaw":
        assert math.atan2(p1[1], p1[0]) > math.atan2(p0[1], p0[0]) + 0.15
    elif name in ("shoulder_pitch", "elbow_pitch"):
        assert p1[2] < p0[2] - 0.01, "positive pitch must move the TCP down"
    elif name == "wrist_pitch":
        # gripper x-axis tilts toward -z
        assert R1[2, 0] < R0[2, 0] - 0.1
    elif name == "wrist_roll":
        # gripper y axis rotates toward +z about +x (right-hand rule)
        assert R1[2, 1] > R0[2, 1] + 0.1


def test_gripper_positive_opens(model):
    d_closed = _tcp_after(model, np.array([0, -0.3, 0.9, 0.9, 0, 0.0]))
    d_open = _tcp_after(model, np.array([0, -0.3, 0.9, 0.9, 0, 1.6]))
    gap = lambda d: d.qpos[model.joint("finger_l").qposadr[0]] + d.qpos[model.joint("finger_r").qposadr[0]]
    assert gap(d_closed) < 0.002
    assert gap(d_open) == pytest.approx(load_spec().gripper_opening_at_max, abs=0.003)


def test_fk_matches_mujoco(model):
    rng = np.random.default_rng(0)
    d = mujoco.MjData(model)
    for _ in range(20):
        q = rng.uniform(-1, 1, 5)
        d.qpos[:5] = q
        mujoco.mj_forward(model, d)
        p, R = fk(q)
        assert np.allclose(p, d.site("tcp").xpos, atol=1e-9)
        assert np.allclose(R, d.site("tcp").xmat.reshape(3, 3), atol=1e-9)


def test_topdown_ik_roundtrip():
    ik = TopDownIK()
    rng = np.random.default_rng(1)
    for _ in range(50):
        r, th = rng.uniform(0.15, 0.33), rng.uniform(-1.0, 1.0)
        z = rng.uniform(0.015, 0.15)
        jaw = rng.uniform(-math.pi, math.pi)
        p = np.array([r * math.cos(th), r * math.sin(th), z])
        q = ik.solve(p, jaw_angle=jaw)
        pp, R = fk(q)
        assert np.allclose(pp, p, atol=1e-9)
        assert R[2, 0] == pytest.approx(-1.0, abs=1e-9)  # pointing straight down
        jaw_line = math.atan2(R[1, 1], R[0, 1])
        diff = (jaw_line - jaw + math.pi / 2) % math.pi - math.pi / 2
        assert abs(diff) < 1e-9
