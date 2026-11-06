import math

import numpy as np
import pytest

from armctl import kinematics as K

M = K.default_model()
RNG = np.random.default_rng(1234)


def random_q(n, margin=0.0):
    return RNG.uniform(M.lower + margin, M.upper - margin, size=(n, 5))


def test_model_loaded_from_spec():
    assert M.upper_arm == pytest.approx(0.200)
    assert M.forearm == pytest.approx(0.180)
    assert M.wrist_to_tcp == pytest.approx(0.110)
    assert M.shoulder_height == pytest.approx(0.100)
    assert M.gripper.limits == (0.0, 1.6)


def test_fk_zero_pose():
    T = K.fk(np.zeros(5))
    np.testing.assert_allclose(T[:3, 3], [0.490, 0.0, 0.100], atol=1e-12)
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-12)


def test_fk_sign_conventions():
    # positive pitch = down
    assert K.fk([0, 0.3, 0, 0, 0])[2, 3] < 0.1
    # shoulder -pi/2 = upper arm straight up
    pts = K.joint_positions([0, -math.pi / 2, 0, 0, 0])
    np.testing.assert_allclose(pts["elbow"], [0, 0, 0.300], atol=1e-12)
    # positive yaw moves +x toward +y
    assert K.fk([0.5, 0, 0, 0, 0])[1, 3] > 0
    # wrist roll 0: jaws open/close along world y -> tool y axis = world y
    np.testing.assert_allclose(K.fk([0, 0, 0, 0, 0])[:3, 1], [0, 1, 0], atol=1e-12)
    # roll is about the tool x axis: does not move the TCP
    np.testing.assert_allclose(K.fk([0, 0, 0, 0, 1.0])[:3, 3], [0.49, 0, 0.1], atol=1e-12)


def test_fk_elbow_up_pose():
    T = K.fk([0, -math.pi / 2, math.pi / 2, 0, 0])  # upper arm up, forearm forward
    np.testing.assert_allclose(T[:3, 3], [0.180 + 0.110, 0, 0.300], atol=1e-12)


def test_jacobian_vs_finite_differences():
    eps = 1e-6
    for q in random_q(200):
        J = K.jacobian(q)
        T0 = K.fk(q)
        Jn = np.zeros((6, 5))
        for i in range(5):
            dq = np.zeros(5)
            dq[i] = eps
            Tp, Tm = K.fk(q + dq), K.fk(q - dq)
            Jn[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * eps)
            Jn[3:, i] = (K.rot_log(Tp[:3, :3] @ T0[:3, :3].T) - K.rot_log(Tm[:3, :3] @ T0[:3, :3].T)) / (2 * eps)
        np.testing.assert_allclose(J, Jn, atol=1e-7)


def test_rot_log_roundtrip():
    for _ in range(200):
        w = RNG.normal(size=3)
        w *= RNG.uniform(0, math.pi - 1e-3) / np.linalg.norm(w)
        np.testing.assert_allclose(K.rot_log(K.axis_angle(w, np.linalg.norm(w))), w, atol=1e-9)


def ik_stats(n=5000):
    """FK(IK(FK(q))) over random in-limit configurations."""
    ok = 0
    recovered = 0
    max_pos = 0.0
    max_rot = 0.0
    branches = {}
    for q in random_q(n):
        T = K.fk(q)
        res = K.solve_ik(T)
        if not res.ok:
            continue
        ok += 1
        for s in res.solutions:
            assert M.within_limits(s.q), (s.branch, s.q)
            Ts = K.fk(s.q)
            pe = np.linalg.norm(Ts[:3, 3] - T[:3, 3])
            re = np.linalg.norm(K.rot_log(Ts[:3, :3] @ T[:3, :3].T))
            max_pos, max_rot = max(max_pos, pe), max(max_rot, re)
            branches[s.branch] = branches.get(s.branch, 0) + 1
        if any(np.allclose(s.q, q, atol=1e-6) for s in res.solutions):
            recovered += 1
    return dict(n=n, ok=ok, recovered=recovered, max_pos=max_pos, max_rot=max_rot, branches=branches)


def test_ik_roundtrip_random_reachable():
    s = ik_stats(5000)
    print("\nIK stats:", s)
    assert s["ok"] == s["n"], s  # every FK-generated pose is reachable
    assert s["recovered"] >= 0.999 * s["n"], s  # the generating config is among the branches
    assert s["max_pos"] < 1e-9
    assert s["max_rot"] < 1e-8
    assert s["branches"].get("front/elbow_up", 0) > 100
    assert s["branches"].get("front/elbow_down", 0) > 100
    assert s["branches"].get("back/elbow_up", 0) + s["branches"].get("back/elbow_down", 0) > 10


def test_ik_xyz_pitch_roll_targets():
    # Cartesian targets on a grid in front of the arm, gripper pointing straight down
    hits = 0
    total = 0
    for x in np.linspace(0.12, 0.33, 8):
        for y in np.linspace(-0.2, 0.2, 5):
            for z in (0.02, 0.06, 0.12):
                total += 1
                res = K.solve_ik_xyz([x, y, z], math.pi / 2, 0.3)
                if not res.ok:
                    assert res.reason.startswith("unreachable")
                    continue
                hits += 1
                for s in res.solutions:
                    T = K.fk(s.q)
                    np.testing.assert_allclose(T[:3, 3], [x, y, z], atol=1e-9)
                    np.testing.assert_allclose(T[:3, 0], [0, 0, -1], atol=1e-8)  # pointing down
    assert hits > 0.8 * total


def test_ik_both_elbow_branches_distinct():
    res = K.solve_ik_xyz([0.25, 0.0, 0.10], 0.3)
    names = {s.branch for s in res.solutions}
    assert {"front/elbow_up", "front/elbow_down"} <= names
    up = next(s for s in res.solutions if s.branch == "front/elbow_up")
    dn = next(s for s in res.solutions if s.branch == "front/elbow_down")
    assert K.joint_positions(up.q)["elbow"][2] > K.joint_positions(dn.q)["elbow"][2]


def test_ik_unreachable_reports_reason():
    res = K.solve_ik_xyz([0.8, 0, 0.1], 0.0)
    assert not res.ok and "reach" in res.reason
    with pytest.raises(K.Unreachable):
        K.ik([0.8, 0, 0.1], 0.0)
    # roll beyond the +-3.0 limit is rejected with a joint-limit reason
    res = K.solve_ik_xyz([0.3, 0, 0.1], 0.5, roll=3.1)
    assert not res.ok and "wrist_roll" in res.reason


def test_ik_best_picks_closest():
    res = K.solve_ik_xyz([0.25, 0.0, 0.10], 0.3)
    for s in res.solutions:
        assert res.best(s.q + 1e-3).branch == s.branch


def test_dls_refine_from_perturbed_seed():
    fails = 0
    for q in random_q(300, margin=0.2):
        T = K.fk(q)
        sol = K.solve_ik_numeric(T, q + RNG.normal(scale=0.15, size=5))
        if sol.pos_err > 1e-6 or sol.rot_err > 1e-5:
            fails += 1
    assert fails <= 3


def test_dls_unreachable_orientation_gives_least_squares():
    # ask for a tool axis pointing sideways (out of the arm plane): not reachable exactly
    T = K.make_pose([0.3, 0.0, 0.1], K.rot_z(math.pi / 2))
    res = K.solve_ik(T)
    assert res.ok  # position reachable, orientation projected
    assert all(s.pos_err < 1e-6 for s in res.solutions)
    assert all(s.rot_err > 0.5 for s in res.solutions)


def test_gripper_conversion():
    assert K.gripper_rad_to_opening(0.0) == 0.0
    assert K.gripper_rad_to_opening(1.6) == pytest.approx(0.060)
    assert K.gripper_rad_to_opening(0.8) == pytest.approx(0.030)
    assert K.gripper_rad_to_opening(5.0) == pytest.approx(0.060)  # clamped
    for o in np.linspace(0, 0.06, 13):
        assert K.gripper_rad_to_opening(K.gripper_opening_to_rad(o)) == pytest.approx(o)
    assert K.gripper_opening_to_rad(1.0) == pytest.approx(1.6)


def test_wrap():
    for a in np.linspace(-10, 10, 101):
        w = K.wrap(a)
        assert -math.pi < w <= math.pi
        assert math.isclose(math.cos(w), math.cos(a), abs_tol=1e-12)
