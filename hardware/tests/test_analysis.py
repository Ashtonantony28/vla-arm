"""Hand-checked cases for the torque / spring / current / bus-timing calculations."""
import math

import numpy as np
import pytest

from analysis.budget import (G, ArmModel, BusTiming, ServoRating, SpringDesign, gravity_torques,
                             minimax_balance, spring_torque)

L = (0.200, 0.180, 0.110)
ZERO = {"upper_arm": 0.0, "forearm": 0.0, "wrist": 0.0, "gripper": 0.0}
MID = {"upper_arm": (0.1, 0.0), "forearm": (0.09, 0.0), "wrist": (0.02, 0.0), "gripper": (0.07, 0.0)}


def test_single_mass_on_upper_arm():
    m = ArmModel(L, dict(ZERO, upper_arm=1.0), MID)
    t = gravity_torques(m)
    assert t["shoulder_pitch"] == pytest.approx(0.981)      # 1 kg x 9.81 x 0.1 m
    assert t["elbow_pitch"] == pytest.approx(0.0)


def test_payload_only_stretched():
    m = ArmModel(L, ZERO, MID, payload=0.25)
    t = gravity_torques(m)
    assert t["shoulder_pitch"] == pytest.approx(0.25 * 9.81 * 0.49)   # 1.2017
    assert t["elbow_pitch"] == pytest.approx(0.25 * 9.81 * 0.29)      # 0.7112
    assert t["wrist_pitch"] == pytest.approx(0.25 * 9.81 * 0.11)      # 0.2698


def test_pitch_down_reduces_moment_arm():
    m = ArmModel(L, ZERO, MID, payload=1.0)
    assert gravity_torques(m, q2=math.pi / 2)["shoulder_pitch"] == pytest.approx(0.0, abs=1e-12)
    assert gravity_torques(m, q2=math.pi / 3)["shoulder_pitch"] == pytest.approx(9.81 * 0.49 * 0.5)


def test_elbow_folded_back_gives_negative_shoulder_torque():
    m = ArmModel(L, ZERO, MID, payload=0.25)
    # forearm folded straight back: TCP at 0.2 - 0.29 = -0.09 m
    assert gravity_torques(m, q3=math.pi)["shoulder_pitch"] == pytest.approx(-0.25 * 9.81 * 0.09)


def test_com_z_offset_rotates_with_link():
    m = ArmModel(L, dict(ZERO, gripper=1.0), dict(MID, gripper=(0.0, 0.05)))
    # wrist pitched 90 deg down: a +z COM offset moves to +x by 0.05 m from the wrist axis
    t = gravity_torques(m, q4=math.pi / 2)
    assert t["wrist_pitch"] == pytest.approx(9.81 * 0.05)


def test_spring_torque_is_k_a_b_cos():
    assert spring_torque(2.0, 0.0) == pytest.approx(2.0)
    assert spring_torque(2.0, math.pi / 3) == pytest.approx(1.0)


def test_zero_length_spring_geometry_matches_cos_law():
    """Independent check: torque of the force k*(A-B) about O equals k*a*b*cos(q)."""
    a, b, k = 0.04, 0.07, 400.0
    for q in np.linspace(-1.8, 1.3, 9):
        A = np.array([0.0, a])                         # (x, z)
        B = np.array([b * math.cos(q), -b * math.sin(q)])
        F = k * (A - B)                                # pulls B toward A
        # torque about +y: tau_y = z*Fx - x*Fz; lifting the arm is -tau_y
        tau_y = B[1] * F[0] - B[0] * F[1]
        assert -tau_y == pytest.approx(k * a * b * math.cos(q), rel=1e-9, abs=1e-12)


def test_minimax_trivial():
    K, w = minimax_balance(np.array([1.0, 3.0]), np.array([1.0, 1.0]))
    assert K == pytest.approx(2.0, abs=1e-6)
    assert w == pytest.approx(1.0, abs=1e-6)


def test_spring_design_numbers():
    sd = SpringDesign(K=1.0, a=0.04, b=0.07).bounds((-1.8, 0.8))
    assert sd.k == pytest.approx(1.0 / (0.04 * 0.07))          # 357.14 N/m
    assert sd.ab_min == pytest.approx(0.03)                      # vertical-up pose in range: b - a
    ang = math.pi / 2 + 0.8
    assert sd.ab_max == pytest.approx(math.sqrt(0.04 ** 2 + 0.07 ** 2 - 2 * 0.04 * 0.07 * math.cos(ang)), rel=1e-4)
    r = sd.realisation(0.8)
    assert r["Fi_N"] == pytest.approx(sd.k * 0.8 * 0.03)
    assert r["extension_min_mm"] > 0                              # spring never slack


def test_servo_current_model():
    s = ServoRating(2.9, 2.7, 1.0)
    assert s.cont_torque == pytest.approx(1.16)
    assert s.current_for(2.9) == pytest.approx(2.7)
    assert s.current_for(0.0) == pytest.approx(s.idle_current)
    assert s.current_for(10.0) == pytest.approx(2.7)              # clamps at stall


def test_bus_timing_hand_count():
    b = BusTiming()
    assert b.byte_us == pytest.approx(10.0)
    assert b.sync_write_len() == 26           # 8 + 6 x (1 + 2)
    assert BusTiming(write_bytes_per_servo=6).sync_write_len() == 50
    assert b.status_len() == 21
    r = b.budget()
    assert r["total_us"] == pytest.approx(260 + 80 + 2 + 100 + 210 + 2 + 30)
    assert r["total_us"] < 1000
    assert BusTiming(write_bytes_per_servo=6).budget()["total_us"] < 1000


def test_report_generates_and_flags_payload():
    from analysis import report
    txt = report.build(q2_mech_max=0.80)
    assert "PSU recommendation" in txt and "Counterbalance options" in txt
    # sanity: the spec-mass shoulder torque at full reach is between 2 and 3 N*m (hand estimate 2.6)
    mods, _ = report.models()
    t = gravity_torques(mods["spec masses"])["shoulder_pitch"]
    assert 2.0 < t < 3.0
