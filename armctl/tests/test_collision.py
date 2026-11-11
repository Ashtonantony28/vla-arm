"""Capsule collision model: geometry primitives, CAD findings, speed."""

import time

import numpy as np
import pytest

from armctl.collision import LINKS, CollisionModel, _sd_box_c, _sd_zcyl_c, _seg_seg_dist

HOME = np.array([0.0, -1.2, 1.4, 0.6, 0.0, 0.5])


@pytest.fixture(scope="module")
def cm():
    return CollisionModel()


def first_contact(j, lo, hi, n=4001, base=None, **kw):
    m = CollisionModel(**kw)
    q = np.zeros(6) if base is None else np.asarray(base, float)
    qs = np.linspace(lo, hi, n)
    Q = np.tile(q, (n, 1))
    Q[:, j] = qs
    c = m.clearance(Q)
    bad = np.flatnonzero(~c.ok)
    return (float(qs[bad[0]]), c.worst[bad[0]]) if len(bad) else (None, "")


# ---------------------------------------------------------------- primitives
def test_seg_seg_matches_brute_force():
    rng = np.random.default_rng(0)
    s = np.linspace(0, 1, 401)
    for _ in range(300):
        p1, q1, p2, q2 = rng.normal(0, 0.1, (4, 3))
        if rng.random() < 0.2:
            q2 = p2 + (q1 - p1) * rng.uniform(-2, 2)  # parallel
        if rng.random() < 0.1:
            q1 = p1.copy()  # point
        A = p1 + np.outer(s, q1 - p1)
        Bs = p2 + np.outer(s, q2 - p2)
        brute = np.min(np.linalg.norm(A[:, None] - Bs[None], axis=-1))
        d = _seg_seg_dist(p1, q1, p2, q2)
        assert d <= brute + 1e-12
        assert d == pytest.approx(brute, abs=6e-4)  # brute force grid resolution


def test_signed_distances():
    c, h = np.array([0.0, 0.0, 0.0]), np.array([1.0, 2.0, 3.0])
    assert _sd_box_c(2.0, 0.0, 0.0, c, h) == pytest.approx(1.0)
    assert _sd_box_c(0.0, 0.0, 0.0, c, h) == pytest.approx(-1.0)
    assert _sd_box_c(2.0, 3.0, 0.0, c, h) == pytest.approx(np.sqrt(2))
    xy = np.array([0.0, 0.0])
    assert _sd_zcyl_c(0.5, 0.0, 0.0, xy, 0.4, 0.0, 0.1) == pytest.approx(0.1)
    assert _sd_zcyl_c(0.0, 0.0, 0.0, xy, 0.4, 0.0, 0.1) == pytest.approx(-0.1)
    assert _sd_zcyl_c(0.0, 0.0, 0.3, xy, 0.4, 0.0, 0.1) == pytest.approx(0.2)


def test_segment_static_is_a_tight_lower_bound(cm):
    rng = np.random.default_rng(1)
    n = 400
    a = rng.uniform([-0.15, -0.15, -0.05], [0.15, 0.15, 0.1], (n, 3))
    b = a + rng.normal(0, 0.06, (n, 3))
    s = np.linspace(0, 1, 4001)
    for box in (True, False):
        ks = np.flatnonzero((cm.st_kind == 0) == box)
        k = rng.choice(ks, n)
        got = cm._seg_static(a, b, k, box)
        P = a[:, None] + (b - a)[:, None] * s[None, :, None]
        if box:
            ref = _sd_box_c(P[..., 0], P[..., 1], P[..., 2], cm.st_c[k][:, None], cm.st_h[k][:, None])
        else:
            ref = _sd_zcyl_c(P[..., 0], P[..., 1], P[..., 2], cm.st_xy[k][:, None], cm.st_r[k][:, None],
                             cm.st_zc[k][:, None], cm.st_hz[k][:, None])
        ref = ref.min(axis=1)
        assert np.all(got <= ref + 1e-9)  # never overestimates the clearance
        L = np.linalg.norm(b - a, axis=1)
        assert np.all(got >= ref - 0.0016 * L - 1e-9)  # within half a final grid step


def test_capsule_frames_match_kinematics(cm):
    from armctl.kinematics import _chain

    rng = np.random.default_rng(2)
    Q = rng.uniform(cm.model.lower6, cm.model.upper6, (20, 6))
    R, t = cm.link_frames(Q)
    for b in range(20):
        fr = _chain(cm.model, Q[b])
        for k in range(5):
            assert np.allclose(R[b, k], fr[k][:3, :3]) and np.allclose(t[b, k], fr[k][:3, 3])
    # jaws: shifted +-18.75 mm/rad along the gripper y axis
    s = 0.01875 * Q[:, 5]
    g = LINKS.index("gripper")
    assert np.allclose(t[:, g + 1] - t[:, g], R[:, g, :, 1] * s[:, None])


# ---------------------------------------------------------------- behaviour
def test_nominal_poses_are_clear(cm):
    c = cm.check(np.zeros(6))
    assert c.ok[0] and c.hits == [[]]
    assert cm.check(HOME).ok[0]


def test_reproduces_cad_wrist_into_base(cm):
    """CAD report: elbow all-pairs free range ends at +2.363 rad (bisection resolution 0.02,
    so contact lies in [2.363, 2.383]) with the shoulder level: the wrist meets the base."""
    q, what = first_contact(2, 0.0, 2.7, check_table=False)
    assert 2.33 <= q <= 2.383, q
    assert "wrist" in what and "base" in what


def test_reproduces_cad_shoulder_into_base(cm):
    """CAD report: shoulder all-pairs free range ends at +0.802 (contact in [0.802, 0.822]);
    the capsule model is conservative (fitted to enclose every CAD point)."""
    q, what = first_contact(1, 0.0, 1.3, check_table=False)
    assert 0.76 <= q <= 0.822, q
    assert "upper_arm" in what and "base" in what


def test_arm_reaches_table_at_full_extension():
    """With the arm stretched out (elbow/wrist 0) the gripper touches the table at shoulder
    ~+0.165 rad (CAD point cloud: 0.176). The elbow *axis* would only reach z=0 at
    asin(0.1/0.2) = 0.524 rad."""
    q, what = first_contact(1, 0.0, 1.3, check_self=False, check_base=False)
    assert 0.14 <= q <= 0.177, q
    assert "table" in what and ("gripper" in what or "jaw" in what)


def test_self_collision_gripper_folded_onto_forearm():
    q, what = first_contact(3, 0.0, 1.9, base=[0, -1.2, 1.4, 0, 0, 0.5], check_table=False, check_base=False)
    # the wrist pitch range is free in the CAD (all pairs, others at 0); with the elbow bent
    # the gripper still clears the forearm over the whole range
    assert q is None, (q, what)
    # but with the elbow folded back and the arm raised, pitching the wrist down brings the
    # gripper onto the shoulder bracket
    q, what = first_contact(3, 0.0, 1.9, base=[0, -0.82, 2.29, 0, 0, 1.0], check_table=False, check_base=False)
    assert q is not None and "shoulder" in what, (q, what)


def test_margin_and_details(cm):
    m = CollisionModel(margin=0.02, check_table=False)
    c = m.check(np.zeros(6))  # forearm/gripper are ~9 mm apart at zero pose
    assert not c.ok[0]
    assert any("forearm" in h and "gripper" in h for h, _ in c.hits[0])
    ref = CollisionModel(check_table=False).check(np.zeros(6)).min_clearance[0]
    assert c.min_clearance[0] == pytest.approx(ref - 0.02, abs=1e-9)


def test_batch_matches_single(cm):
    rng = np.random.default_rng(3)
    Q = rng.uniform(cm.model.lower6, cm.model.upper6, (40, 6))
    c = cm.clearance(Q)
    for b in range(40):
        one = cm.clearance(Q[b : b + 1])
        assert one.min_clearance[0] == pytest.approx(c.min_clearance[b])


def test_exact_below_value_is_exact(cm):
    """Clearances below exact_below are exact, regardless of culling."""
    rng = np.random.default_rng(4)
    Q = rng.uniform(cm.model.lower6, cm.model.upper6, (200, 6))
    loose = CollisionModel(exact_below=1.0).clearance(Q).min_clearance
    tight = cm.clearance(Q).min_clearance
    m = loose < cm.exact_below
    assert np.allclose(tight[m], loose[m])
    assert np.all(tight[~m] >= cm.exact_below - 1e-12)


def test_chunk_benchmark(cm):
    """A 50-step action chunk is checked in well under 2 ms on an idle machine (README has the
    numbers); the assertion here is a loose regression guard for loaded CI machines."""
    q0 = np.array([0.2, -0.5, 1.2, 0.8, 0.3, 0.8])
    q1 = np.array([0.6, -0.1, 1.6, 1.1, -0.4, 0.2])
    Q = q0 + (q1 - q0) * np.linspace(0, 1, 50)[:, None]
    for _ in range(20):
        cm.clearance(Q)
    ts = []
    for _ in range(200):
        t0 = time.perf_counter()
        cm.clearance(Q)
        ts.append(time.perf_counter() - t0)
    med = float(np.median(ts))
    print(f"50-pose chunk: median {med * 1e3:.3f} ms")
    assert med < 6e-3
