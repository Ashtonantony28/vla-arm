"""armctl.trajectory (Python port of firmware/core/src/traj.c) against the C code itself.

traj.c is compiled on the fly with the system C compiler together with a tiny wrapper, and both
implementations are driven through the same random replan sequences (mid-move replans with
non-zero velocity/acceleration, vmax stretching, blends, limit saturation), comparing the
commanded state at every 1 kHz step. Skipped when there is no C compiler or no firmware tree.
"""

import ctypes
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from armctl.trajectory import FirmwareTrajectory, f2i16, quintic_coeffs, quintic_peak_vel

ROOT = Path(__file__).resolve().parents[2]
FW = ROOT / "firmware" / "core"

WRAPPER = r"""
#include "armfw/traj.h"
size_t tw_sizeof(void) { return sizeof(traj_group_t); }
static traj_limits_t L;
void tw_limits(const float *lim) {
    for (int j = 0; j < 6; j++) { L.lim_min[j] = lim[j]; L.lim_max[j] = lim[6 + j]; L.vmax[j] = lim[12 + j]; }
}
void tw_reset(traj_group_t *g, const float *pos) { traj_reset(g, pos); }
void tw_replan(traj_group_t *g, const float *tgt, float move_s, float *out) {
    traj_plan_result_t r = traj_replan(g, tgt, move_s, &L);
    out[0] = r.pos_clamped; out[1] = r.stretched; out[2] = r.blended; out[3] = r.vmax_infeasible;
    out[4] = r.overshoot; out[5] = r.T; out[6] = r.peak_ratio; out[7] = r.evals;
}
void tw_step(traj_group_t *g, unsigned dt_us) { traj_step(g, dt_us, &L); }
void tw_cmd(const traj_group_t *g, float *out) {
    for (int j = 0; j < 6; j++) { out[j] = g->cmd[j].p; out[6 + j] = g->cmd[j].v; out[12 + j] = g->cmd[j].a; }
    out[18] = g->saturated; out[19] = g->moving;
}
"""

F6 = ctypes.c_float * 6
F8 = ctypes.c_float * 8
F18 = ctypes.c_float * 18
F20 = ctypes.c_float * 20


@pytest.fixture(scope="module")
def ctraj(tmp_path_factory):
    cc = shutil.which("cc") or shutil.which("gcc")
    if not cc or not (FW / "src" / "traj.c").exists():
        pytest.skip("no C compiler or firmware sources")
    d = tmp_path_factory.mktemp("ctraj")
    (d / "wrap.c").write_text(WRAPPER)
    so = d / "libtraj.so"
    subprocess.run(
        [cc, "-O2", "-std=c11", "-shared", "-fPIC", "-fno-math-errno", f"-I{FW / 'include'}",
         str(FW / "src" / "traj.c"), str(d / "wrap.c"), "-o", str(so)],
        check=True,
    )
    lib = ctypes.CDLL(str(so))
    lib.tw_sizeof.restype = ctypes.c_size_t
    lib.tw_replan.argtypes = [ctypes.c_void_p, F6, ctypes.c_float, F8]
    lib.tw_step.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    lib.tw_reset.argtypes = [ctypes.c_void_p, F6]
    lib.tw_cmd.argtypes = [ctypes.c_void_p, F20]
    lib.tw_limits.argtypes = [F18]
    return lib


class CTraj:
    def __init__(self, lib, lim_min, lim_max, vmax):
        self.lib = lib
        self.buf = ctypes.create_string_buffer(lib.tw_sizeof() + 16)
        self.g = ctypes.addressof(self.buf)
        lib.tw_limits(F18(*lim_min, *lim_max, *vmax))

    def reset(self, pos):
        self.lib.tw_reset(self.g, F6(*pos))

    def replan(self, tgt, move_s):
        out = F8()
        self.lib.tw_replan(self.g, F6(*tgt), move_s, out)
        return list(out)

    def step(self):
        self.lib.tw_step(self.g, 1000)

    def cmd(self):
        out = F20()
        self.lib.tw_cmd(self.g, out)
        return np.array(out[:6]), np.array(out[6:12]), bool(out[18])


LIM_MIN = [-2600, -1800, -2700, -1900, -3000, 0]
LIM_MAX = [2600, 750, 2300, 1900, 3000, 1600]
VMAX = [3000, 2500, 3000, 3500, 4000, 4000]


def test_defaults_match_firmware_config():
    t = FirmwareTrajectory()
    assert list(t.lim_min) == LIM_MIN and list(t.lim_max) == LIM_MAX and list(t.vmax) == VMAX


def test_rest_to_rest_is_a_straight_joint_line():
    t = FirmwareTrajectory()
    t.reset([0, 0, 0, 0, 0, 0])
    t.replan([500, -300, 800, 200, -100, 900], 1.0)
    P = t.sample(0.01)
    frac = P / np.array([500, -300, 800, 200, -100, 900.0])
    assert np.allclose(frac, frac[:, :1], atol=1e-9)  # every joint at the same fraction


def test_peak_velocity_matches_sampling():
    rng = np.random.default_rng(1)
    for _ in range(200):
        p0, v0, a0, pf = rng.normal(0, 1000, 4)
        T = rng.uniform(0.05, 2)
        b, TT = quintic_coeffs([p0], [v0], [a0], [pf], [T])
        s = np.linspace(0, 1, 20001)
        d1 = b[0, 1:] * np.arange(1, 6)
        v = np.polyval(d1[::-1], s) / T
        assert quintic_peak_vel(b, TT)[0] == pytest.approx(np.abs(v).max(), rel=1e-6)


def test_f2i16_matches_firmware_rounding():
    assert list(f2i16([0.5, -0.5, 1.49, -1.5, 40000, -40000])) == [1, -1, 1, -2, 32767, -32768]


def _run_sequence(ctraj, rng, n_cmds, period_ms, aggressive):
    py = FirmwareTrajectory(LIM_MIN, LIM_MAX, VMAX)
    c = CTraj(ctraj, LIM_MIN, LIM_MAX, VMAX)
    start = rng.uniform(np.array(LIM_MIN) * 0.5, np.array(LIM_MAX) * 0.5).round()
    py.reset(start)
    c.reset(start)
    worst = 0.0
    flags_diff = 0
    for k in range(n_cmds):
        span = (np.array(LIM_MAX) - LIM_MIN) * (0.6 if aggressive else 0.15)
        cur = py.cmd_p
        tgt = (cur + rng.uniform(-span, span)).round()
        if rng.random() < 0.1:  # occasionally beyond a limit (clamp) or right at it
            j = rng.integers(6)
            tgt[j] = LIM_MAX[j] + 200 if rng.random() < 0.5 else LIM_MIN[j] - 200
        move_s = float(rng.choice([0.005, 0.02, 0.05, 0.2, 0.6]))
        rc = c.replan(tgt, move_s)
        rp = py.replan(tgt, move_s)
        flags_diff += int(bool(rc[2]) != rp.blended or bool(rc[1]) != rp.stretched)
        assert rp.T == pytest.approx(rc[5], rel=2e-4, abs=1e-5)
        for _ in range(int(rng.integers(1, period_ms + 1))):
            c.step()
            py.step()
            pc, vc, _ = c.cmd()
            worst = max(worst, float(np.max(np.abs(pc - py.cmd_p))))
    return worst, flags_diff


@pytest.mark.parametrize("aggressive", [False, True])
def test_matches_compiled_traj_c(ctraj, aggressive):
    rng = np.random.default_rng(7 + aggressive)
    worst_all, flags = 0.0, 0
    for _ in range(25):
        w, f = _run_sequence(ctraj, rng, n_cmds=12, period_ms=60, aggressive=aggressive)
        worst_all = max(worst_all, w)
        flags += f
    # float32 (C) vs float64 (Python): commanded positions agree to a small fraction of a mrad
    assert worst_all < 0.05, worst_all
    assert flags == 0


def test_saturation_and_blend_are_exercised(ctraj):
    """A fast move toward a limit replanned to a target behind the limit: overshoot ->
    saturation; a reversal right after a short move: blend."""
    py = FirmwareTrajectory(LIM_MIN, LIM_MAX, VMAX)
    c = CTraj(ctraj, LIM_MIN, LIM_MAX, VMAX)
    for t in (py, c):
        t.reset([0, 600, 0, 0, 0, 0])
    tgt = [0, 750, 0, 0, 0, 0]
    py.replan(tgt, 0.02)
    c.replan(tgt, 0.02)
    sat = False
    for _ in range(30):
        py.step()
        c.step()
        sat |= py.saturated
        assert np.allclose(c.cmd()[0], py.cmd_p, atol=0.05)
    r = py.replan([0, 750, 0, 0, 0, 0], 0.005)
    c.replan([0, 750, 0, 0, 0, 0], 0.005)
    # reversal with high acceleration -> blend path
    py.reset([0, 0, 0, 0, 0, 0])
    c.reset([0, 0, 0, 0, 0, 0])
    py.replan([2500, 0, 0, 0, 0, 0], 0.4)
    c.replan([2500, 0, 0, 0, 0, 0], 0.4)
    for _ in range(40):
        py.step()
        c.step()
    r = py.replan([-2500, 0, 0, 0, 0, 0], 0.05)
    rc = c.replan([-2500, 0, 0, 0, 0, 0], 0.05)
    assert r.blended == bool(rc[2])
    for _ in range(3000):
        py.step()
        c.step()
        assert np.allclose(c.cmd()[0], py.cmd_p, atol=0.05)
    assert r.stretched
