"""armctl's trajectory twin (armctl.trajectory) against the real firmware simulator.

The firmware runs in-process through libarmfw's virtual-time simulator (ctypes), driven by
armctl's own frame encoder. After every 1 ms control step the board's STATE frames are decoded
and their ``cmd_pos_crc`` (CRC over this cycle's six commanded setpoints, int16 mrad) is compared
with the CRC of the setpoints armctl.trajectory predicts for the same step. STATE carries
measured positions, not the commanded ones, so the CRC is the only view of the board's
trajectory; matching it means the twin reproduces the board's setpoints to the mrad at every
sample, through mid-move replans, vmax stretching, blends and limit saturation.

Build first:  cmake -S firmware -B firmware/build && cmake --build firmware/build
"""
import ctypes
import itertools
import struct
from pathlib import Path

import numpy as np
import pytest

from armctl import protocol as P
from armctl.trajectory import FirmwareTrajectory, f2i16

ROOT = Path(__file__).resolve().parents[2]
LIB = ROOT / "firmware" / "build" / "libarmfw.so"
pytestmark = pytest.mark.skipif(not LIB.exists(), reason="libarmfw.so not built")


class Sim:
    def __init__(self):
        self.lib = lib = ctypes.CDLL(str(LIB))
        lib.armfw_sim_sizeof.restype = ctypes.c_size_t
        lib.armfw_sim_host_write.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
        lib.armfw_sim_host_read.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
        lib.armfw_sim_host_read.restype = ctypes.c_size_t
        lib.armfw_sim_step.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        lib.armfw_sim_init.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        lib.armfw_sim_mode.argtypes = [ctypes.c_void_p]
        self.buf = ctypes.create_string_buffer(lib.armfw_sim_sizeof() + 64)
        self.p = ctypes.c_void_p((ctypes.addressof(self.buf) + 15) & ~15)
        lib.armfw_sim_init(self.p, 0)
        self.dec = P.FrameDecoder()
        self.seq = itertools.count(1)
        self.rx = ctypes.create_string_buffer(65536)
        self.steps = 0


    def send(self, msg):
        data = P.encode(msg, next(self.seq) & 0xFF)
        self.lib.armfw_sim_host_write(self.p, data, len(data))

    def step(self):
        """One control period; returns the decoded board messages emitted during it. A PING every
        40 ms keeps the watchdog fed (it does not touch the trajectory)."""
        self.steps += 1
        if self.steps % 40 == 0:
            self.send(P.Ping())
        self.lib.armfw_sim_step(self.p, 1)
        n = self.lib.armfw_sim_host_read(self.p, self.rx, len(self.rx))
        return [f.decode() for f in self.dec.feed(self.rx.raw[:n])]


def crc_candidates(cmd_mrad):
    """CRCs of the rounded setpoints; joints within 0.02 mrad of a rounding boundary contribute
    both roundings (float32 on the board vs float64 here; the board's polynomial evaluation in float32 is good to
    ~1e-2 mrad at |p| ~ 2000 mrad)."""
    base = f2i16(cmd_mrad)
    frac = np.abs(np.asarray(cmd_mrad) - np.trunc(cmd_mrad))
    amb = [j for j in range(6) if abs(frac[j] - 0.5) < 0.02]
    out = set()
    for alt in itertools.product(*[(0, 1) for _ in amb]):
        v = base.copy()
        for j, a in zip(amb, alt):
            if a:
                v[j] = int(np.floor(cmd_mrad[j])) if v[j] == int(np.ceil(cmd_mrad[j])) else int(np.ceil(cmd_mrad[j]))
        out.add(P.commanded_pos_crc(tuple(int(x) for x in v)))
    return out


def set_targets(sim, twin, tgt_mrad, move_ms):
    sim.send(P.SetTargets(move_ms, tuple(int(x) for x in tgt_mrad), P.FLAG_TORQUE_ENABLE))
    return twin.replan(np.asarray(tgt_mrad, float), move_ms / 1000.0)


def run_steps(sim, twin, n, stats):
    for _ in range(n):
        msgs = sim.step()
        twin.step()
        for m in msgs:
            if isinstance(m, P.Nack):
                raise AssertionError(f"NACK {m}")
            if isinstance(m, P.State):
                assert m.mode == P.Mode.ACTIVE, m.mode
                stats["states"] += 1
                if m.cmd_pos_crc in crc_candidates(twin.cmd_p):
                    stats["match"] += 1
                else:
                    stats["mismatch"].append((stats["states"], twin.cmd_p.copy()))


def start(sim):
    """Boot, torque on (command follows the measured position), move to an exact integer pose."""
    for _ in range(20):
        sim.step()
    twin = FirmwareTrajectory()
    home = np.array([0, -800, 1200, 400, 0, 800])
    sim.send(P.SetTargets(600, tuple(int(x) for x in home), P.FLAG_TORQUE_ENABLE))
    for _ in range(1500):
        msgs = sim.step()
        assert not any(isinstance(m, P.Nack) for m in msgs)
    msgs = []
    while not any(isinstance(m, P.State) for m in msgs):
        msgs = sim.step()
    st = [m for m in msgs if isinstance(m, P.State)][-1]
    assert st.mode == P.Mode.ACTIVE
    assert st.cmd_pos_crc == P.commanded_pos_crc(tuple(int(x) for x in home))  # at rest on target
    twin.reset(home)
    return twin


def test_cmd_crc_tracks_twin_through_replans():
    sim = Sim()
    twin = start(sim)
    rng = np.random.default_rng(3)
    stats = {"states": 0, "match": 0, "mismatch": []}
    lo, hi = twin.lim_min, twin.lim_max
    blended = stretched = overshoot = 0
    for k in range(120):
        span = (hi - lo) * (0.5 if k % 3 == 0 else 0.1)
        tgt = np.clip(twin.cmd_p + rng.uniform(-span, span), lo - 100, hi + 100).round()
        move_ms = int(rng.choice([5, 20, 40, 100, 300]))
        r = set_targets(sim, twin, tgt, move_ms)
        blended += r.blended
        stretched += r.stretched
        overshoot += r.overshoot
        run_steps(sim, twin, int(rng.integers(3, 80)), stats)
    run_steps(sim, twin, 3000, stats)  # let the last move finish
    assert stats["states"] > 1000
    # every STATE matches (within the float32 rounding ambiguity handled by crc_candidates)
    assert not stats["mismatch"], stats["mismatch"][:5]
    assert stretched > 20 and blended >= 1, (stretched, blended, overshoot)


def test_teleop_stream_twin_matches():
    """Teleop-style stream: a new target every 10 ms with move_ms = 20, so the board always
    replans from a mid-move state (non-zero v, a)."""
    sim = Sim()
    twin = start(sim)
    stats = {"states": 0, "match": 0, "mismatch": []}
    home = twin.cmd_p.copy()
    for k in range(300):
        t = k * 0.01
        tgt = home + np.array([600, 300, -400, 500, 900, 400]) * np.sin(2 * np.pi * 0.7 * t)
        set_targets(sim, twin, tgt.round(), 20)
        run_steps(sim, twin, 10, stats)
    run_steps(sim, twin, 500, stats)
    assert stats["states"] >= 680
    assert not stats["mismatch"], stats["mismatch"][:5]
