import math

import numpy as np
import pytest

from armctl import kinematics as K
from armctl.safety import SafetyConfig, SafetyShim

# a comfortable pose: TCP ~ (0.29, 0, 0.30) region
HOME = np.array([0.0, -1.2, 1.4, 0.6, 0.0, 0.5])


def test_home_is_safe():
    s = SafetyShim()
    assert s.workspace_violations(HOME) == []
    r = s.filter(HOME, 0.1)
    assert not r.modified and np.allclose(r.q_out, HOME)


def test_joint_limit_clamp_with_reason():
    s = SafetyShim()
    q = HOME.copy()
    q[0] = 3.5  # base beyond 2.6
    q[5] = -0.3  # gripper below 0
    r = s.filter(q, 0.1)
    assert r.q_out[0] == pytest.approx(2.6) and r.q_out[5] == pytest.approx(0.0)
    assert any("joint 0 clamped" in x for x in r.reasons)
    assert any("joint 5 clamped" in x for x in r.reasons)


def test_limit_margin():
    s = SafetyShim(config=SafetyConfig(limit_margin=0.1))
    q = HOME.copy()
    q[0] = 2.6
    assert s.filter(q, 0.1).q_out[0] == pytest.approx(2.5)


def test_velocity_limit_per_step():
    s = SafetyShim()
    s.reset(HOME)
    q = HOME.copy()
    q[0] += 1.0  # 1 rad in 50 ms = 20 rad/s >> vmax 3
    r = s.filter(q, 0.05)
    assert r.q_out[0] - HOME[0] == pytest.approx(3.0 * 0.05)
    assert any("joint 0 step" in x for x in r.reasons)
    # subsequent steps keep closing the gap
    r2 = s.filter(q, 0.05)
    assert r2.q_out[0] - HOME[0] == pytest.approx(0.3)


def test_vel_scale():
    s = SafetyShim(config=SafetyConfig(vel_scale=0.5))
    s.reset(HOME)
    q = HOME.copy()
    q[3] += 1.0
    assert s.filter(q, 0.1).q_out[3] - HOME[3] == pytest.approx(3.5 * 0.5 * 0.1)


def test_table_plane_rejects_or_scales():
    s = SafetyShim()
    q_table = K.ik([0.25, 0.0, -0.02], math.pi / 2, q_ref=HOME)  # TCP below the table
    q_table = np.append(q_table, 0.5)
    assert any("tcp z=" in v for v in s.workspace_violations(q_table))
    # no previous command -> reject
    r = s.filter(q_table, 1.0)
    assert r.rejected and "workspace" in r.reasons[-1]
    # from a safe previous command -> pulled back to just above the table
    q_above = np.append(K.ik([0.25, 0.0, 0.05], math.pi / 2, q_ref=HOME), 0.5)
    s.reset(q_above)
    r = s.filter(q_table, 5.0)
    assert not r.rejected and r.modified
    z = K.fk(r.q_out)[2, 3]
    assert 0.01 - 1e-6 <= z < 0.02
    assert any("scaled step" in x for x in r.reasons)


def test_wrist_below_table():
    s = SafetyShim()
    q = np.array([0.0, 0.6, 0.0, -1.2, 0.0, 0.0])  # arm pitched down, wrist low, tool up
    pts = K.joint_positions(q)
    assert pts["wrist"][2] < 0.01 < pts["tcp"][2] or pts["wrist"][2] < 0.01
    assert any(v.startswith("wrist") for v in s.workspace_violations(q))


def test_base_cylinder_keepout():
    s = SafetyShim()
    q_in = np.array([0.0, 0.195, -2.27, -0.95, 0.0, 0.0])  # arm folded back, TCP over the base
    v = s.workspace_violations(q_in)
    assert [x for x in v if "keep-out" in x] == v and v
    s.reset(HOME)
    r = s.filter(q_in, 5.0)
    assert not r.rejected
    assert math.hypot(*K.fk(r.q_out)[:2, 3]) >= 0.07 - 1e-6


def test_path_check_catches_dip_between_safe_endpoints():
    # both endpoints safe, but the joint-space straight line passes below the table
    s = SafetyShim(config=SafetyConfig(path_samples=20))
    a = np.append(K.ik([0.30, -0.15, 0.03], math.pi / 2, q_ref=HOME), 0)
    b = np.append(K.ik([0.30, 0.15, 0.03], math.pi / 2, q_ref=HOME), 0)
    mids = [s.workspace_violations(a + (b - a) * t) for t in np.linspace(0, 1, 21)]
    assert not s.workspace_violations(a) and not s.workspace_violations(b)
    s.reset(a)
    r = s.filter(b, 5.0)
    if any(mids):
        assert r.modified and "workspace" in r.reasons[-1]
    else:  # geometry happens to stay above the plane: command passes untouched
        assert not r.modified


def test_recover_from_unsafe_start():
    s = SafetyShim()
    q_rest = np.array([0.0, 0.5, 0.5, 0.5, 0.0, 0.0])  # slumped onto the table
    assert s.workspace_violations(q_rest)
    s.reset(q_rest)
    worse = q_rest.copy()
    worse[1] += 0.2  # push further into the table: rejected
    assert s.filter(worse, 0.1).rejected
    better = q_rest.copy()
    better[1] -= 0.2  # lift the shoulder a bit (still low, but less bad): allowed
    r = s.filter(better, 0.1)
    assert not r.rejected and "recovery" in r.reasons[-1]
    # a big step to a safe pose is velocity-limited; keep stepping until in the workspace
    for _ in range(100):
        r = s.filter(HOME, 0.1)
        assert not r.rejected
        if not s.workspace_violations(r.q_out):
            break
    assert s.workspace_violations(s.last) == []


def test_nan_rejected():
    r = SafetyShim().filter([0, 0, 0, float("nan"), 0, 0], 0.1)
    assert r.rejected


def test_bad_shape():
    with pytest.raises(ValueError):
        SafetyShim().filter([0, 0, 0], 0.1)
