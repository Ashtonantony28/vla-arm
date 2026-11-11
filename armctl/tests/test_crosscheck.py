"""Cross-check armctl's FK and geometric Jacobian against Pinocchio and ikpy (both from PyPI:
``pip install pin ikpy``; skipped when absent).

Two independent model sources:
* the URDF generated from spec/kinematics.yaml (:mod:`armctl.urdf`), loaded by Pinocchio and ikpy;
* the sim's MuJoCo model sim/model/vla_arm.xml (hand-written MJCF), loaded by Pinocchio, so a
  mismatch between the spec, armctl and the sim twin would also show up here.
"""

from pathlib import Path

import numpy as np
import pytest

from armctl import kinematics as K
from armctl.urdf import to_urdf

MJCF = Path(__file__).resolve().parents[2] / "sim" / "model" / "vla_arm.xml"
N = 400


def random_q(n, seed=0):
    m = K.default_model()
    return np.random.default_rng(seed).uniform(m.lower, m.upper, size=(n, 5))


def _pin_check(pin, model, nq_extra=0):
    data = model.createData()
    fid = model.getFrameId("tcp")
    worst_p = worst_r = worst_j = 0.0
    for q in random_q(N):
        qq = np.concatenate([q, np.zeros(nq_extra)])
        pin.forwardKinematics(model, data, qq)
        pin.updateFramePlacements(model, data)
        T = data.oMf[fid].homogeneous
        T_ref = K.fk(q)
        worst_p = max(worst_p, np.abs(T[:3, 3] - T_ref[:3, 3]).max())
        worst_r = max(worst_r, np.abs(T[:3, :3] - T_ref[:3, :3]).max())
        J = pin.computeFrameJacobian(model, data, qq, fid, pin.LOCAL_WORLD_ALIGNED)[:, :5]
        worst_j = max(worst_j, np.abs(J - K.jacobian(q)).max())
    return worst_p, worst_r, worst_j


def test_pinocchio_urdf_fk_and_jacobian():
    pin = pytest.importorskip("pinocchio")
    model = pin.buildModelFromXML(to_urdf())
    assert model.nq == 5 and [model.names[i] for i in range(1, 6)] == [j.name for j in K.default_model().joints]
    wp, wr, wj = _pin_check(pin, model)
    assert wp < 1e-12 and wr < 1e-12 and wj < 1e-12, (wp, wr, wj)


def test_pinocchio_sim_mjcf_fk_and_jacobian():
    pin = pytest.importorskip("pinocchio")
    if not hasattr(pin, "buildModelFromMJCF") or not MJCF.exists():
        pytest.skip("no MJCF parser / sim model")
    model = pin.buildModelFromMJCF(str(MJCF))
    assert [model.names[i] for i in range(1, 6)] == [j.name for j in K.default_model().joints]
    wp, wr, wj = _pin_check(pin, model, nq_extra=model.nq - 5)  # + the two finger slides
    # the MJCF stores the geometry as decimal text (e.g. "0.18"): agreement to float precision
    assert wp < 1e-9 and wr < 1e-9 and wj < 1e-9, (wp, wr, wj)


def test_jacobian_matches_pinocchio_finite_difference_free():
    """Pinocchio's Jacobian time-derivative-free check: J @ dq equals the FK twist (via log6)."""
    pin = pytest.importorskip("pinocchio")
    model = pin.buildModelFromXML(to_urdf())
    data = model.createData()
    fid = model.getFrameId("tcp")
    rng = np.random.default_rng(3)
    for q in random_q(50, seed=3):
        dq = rng.normal(size=5) * 1e-6
        pin.framesForwardKinematics(model, data, q)
        M0 = data.oMf[fid].copy()
        pin.framesForwardKinematics(model, data, q + dq)
        M1 = data.oMf[fid]
        # world-aligned twist at the TCP
        v = M1.translation - M0.translation
        w = pin.log3(M1.rotation @ M0.rotation.T)
        np.testing.assert_allclose(K.jacobian(q) @ dq, np.r_[v, w], atol=1e-11)


def test_ikpy_urdf_fk(tmp_path):
    ikpy_chain = pytest.importorskip("ikpy.chain")
    path = tmp_path / "vla_arm.urdf"
    path.write_text(to_urdf())
    chain = ikpy_chain.Chain.from_urdf_file(str(path), base="base_link", tip="tcp")
    # ikpy links: origin link, 5 revolute, tcp fixed
    active = [l.name for l in chain.links]
    assert active[1:6] == [j.name for j in K.default_model().joints], active
    worst = 0.0
    for q in random_q(N, seed=1):
        T = chain.forward_kinematics(np.r_[0.0, q, 0.0])
        worst = max(worst, np.abs(T - K.fk(q)).max())
    assert worst < 1e-12, worst


@pytest.mark.filterwarnings("ignore:initial_position gives numbers")
def test_ikpy_ik_solution_is_consistent_with_armctl_fk(tmp_path):
    """ikpy's numeric IK, seeded near an armctl analytic IK solution, lands on a pose that armctl's FK
    agrees with (a consistency check of the shared chain, not of either solver)."""
    ikpy_chain = pytest.importorskip("ikpy.chain")
    path = tmp_path / "vla_arm.urdf"
    path.write_text(to_urdf())
    chain = ikpy_chain.Chain.from_urdf_file(
        str(path), base="base_link", tip="tcp", active_links_mask=[0.0] + [True] * 5 + [0.0]
    )
    home = np.array([0.0, -1.2, 1.4, 0.6, 0.0])
    for target in ([0.25, 0.05, 0.10], [0.20, -0.10, 0.05], [0.30, 0.0, 0.20]):
        q_sol = chain.inverse_kinematics(target, initial_position=np.r_[0.0, home, 0.0])[1:6]
        np.testing.assert_allclose(K.fk(q_sol)[:3, 3], target, atol=1e-3)
