"""Scripted (privileged-state) experts for each task.

Plan = sequence of Cartesian keyframes (TCP position, world jaw angle, gripper command, duration):
approach -> descend -> grasp -> lift -> transfer -> lower -> release -> retreat.
Between keyframes the TCP follows a min-jerk profile in Cartesian space; joint targets come from
the sim-local closed-form IK (sim/kinematics.py). Small random noise on waypoints, timing and the
emitted joint targets gives the demos some variety.

Phases are planned lazily (when they start) from the *current* object poses, so the second pick
of `sort` sees where the block actually is.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from sim.envs.tabletop import TabletopEnv
from sim.kinematics import IKError, TopDownIK
from sim.model.build_mjcf import BLOCK_HALF

GRIP_OPEN = 1.25      # rad -> ~47 mm jaw gap for a 30 mm block
GRIP_CLOSED = 0.0     # command fully closed; the block stops the jaws (force-limited servo)
Z_HOVER = 0.10
Z_TRAVEL = 0.11
Z_GRASP = BLOCK_HALF
Z_PLACE_BOWL = 0.004 + BLOCK_HALF + 0.012
Z_PLACE_STACK = 3 * BLOCK_HALF + 0.004


def min_jerk(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


@dataclass
class Keyframe:
    pos: np.ndarray
    jaw: float
    grip: float
    duration: float


def _nearest_equiv(angle: float, ref: float, period: float = math.pi / 2) -> float:
    return angle + period * round((ref - angle) / period)


class ScriptedExpert:
    def __init__(self, env: TabletopEnv, seed: int | None = None, noise: bool = True):
        self.env = env
        self.ik = TopDownIK(env.spec)
        self.rng = np.random.default_rng(seed)
        self.noise = noise
        self.reset()

    # -- public API ----------------------------------------------------------------------------
    def reset(self):
        self._phases = list(self._phase_generators())
        self._segments: list[Keyframe] = []
        self._seg_idx = 0
        self._seg_t = 0
        self._last: Keyframe | None = None
        self._jaw_offset = 0.0  # jaw angle minus held block yaw
        self.done = False

    def act(self, obs=None) -> np.ndarray:
        if self._last is None:
            self._last = self._current_keyframe()
        while self._seg_idx >= len(self._segments):
            if not self._phases:
                self.done = True
                return self._joint_targets(self._last)
            self._segments.extend(self._phases.pop(0)())
        seg = self._segments[self._seg_idx]
        n = max(1, int(round(seg.duration * 30)))
        self._seg_t += 1
        s = min_jerk(self._seg_t / n)
        prev = self._last
        kf = Keyframe(prev.pos + (seg.pos - prev.pos) * s, prev.jaw + (seg.jaw - prev.jaw) * s,
                      prev.grip + (seg.grip - prev.grip) * s, 0)
        if self._seg_t >= n:
            self._last = seg
            self._seg_idx += 1
            self._seg_t = 0
        a = self._joint_targets(kf)
        if self.noise:
            a[:5] += self.rng.normal(0, 0.002, 5)
        return a

    # -- planning ------------------------------------------------------------------------------
    def _phase_generators(self):
        env = self.env
        name = env.task_name
        if name == "pick_place":
            yield lambda: self._pick("red_block")
            yield lambda: self._place_in_bowl("green_bowl")
        elif name == "stack":
            yield lambda: self._pick("red_block")
            yield lambda: self._place_on_block("blue_block")
        elif name == "sort":
            # nearer block first
            order = sorted(["red", "blue"],
                           key=lambda c: np.linalg.norm(env.object_pose(f"{c}_block")[0][:2] - env.tcp_pos()[:2]))
            for c in order:
                yield (lambda c=c: self._pick(f"{c}_block"))
                yield (lambda c=c: self._place_in_bowl(f"{env.assignment[c]}_bowl"))
        yield lambda: [Keyframe(self._last_planned().pos, self._last_planned().jaw, GRIP_OPEN, 0.3)]

    def _last_planned(self) -> Keyframe:
        return self._segments[-1] if self._segments else self._current_keyframe()

    def _dur(self, base: float) -> float:
        return base * (self.rng.uniform(0.85, 1.15) if self.noise else 1.0)

    def _xy_noise(self, sigma: float) -> np.ndarray:
        return np.r_[self.rng.normal(0, sigma, 2), 0.0] if self.noise else np.zeros(3)

    def _pick(self, block: str) -> list[Keyframe]:
        p, yaw = self.env.object_pose(block)
        prev = self._last_planned()
        base_yaw = math.atan2(p[1], p[0])
        # choose the face pair whose jaw angle gives the smallest wrist roll
        jaw0 = base_yaw + math.pi / 2
        jaw = _nearest_equiv(yaw, jaw0)
        self._jaw_offset = jaw - yaw
        g = p + self._xy_noise(0.002)
        return [
            Keyframe(np.r_[g[:2], Z_HOVER], jaw, GRIP_OPEN, self._dur(1.3 if prev.pos[2] > 0.09 else 1.0)),
            Keyframe(np.r_[g[:2], Z_GRASP], jaw, GRIP_OPEN, self._dur(0.8)),
            Keyframe(np.r_[g[:2], Z_GRASP], jaw, GRIP_CLOSED, self._dur(0.5)),
            Keyframe(np.r_[g[:2], Z_TRAVEL], jaw, GRIP_CLOSED, self._dur(0.7)),
        ]

    def _place_in_bowl(self, bowl: str) -> list[Keyframe]:
        p, _ = self.env.object_pose(bowl)
        prev = self._last_planned()
        t = p + self._xy_noise(0.004)
        jaw = prev.jaw + (math.atan2(t[1], t[0]) - math.atan2(prev.pos[1], prev.pos[0]))  # keep roll
        return self._release_at(t, jaw, Z_PLACE_BOWL)

    def _place_on_block(self, block: str) -> list[Keyframe]:
        p, yaw = self.env.object_pose(block)
        prev = self._last_planned()
        jaw = _nearest_equiv(yaw + self._jaw_offset, prev.jaw)
        t = p + self._xy_noise(0.0015)
        return self._release_at(t, jaw, Z_PLACE_STACK)

    def _release_at(self, t, jaw, z_place) -> list[Keyframe]:
        return [
            Keyframe(np.r_[t[:2], Z_TRAVEL], jaw, GRIP_CLOSED, self._dur(1.3)),
            Keyframe(np.r_[t[:2], z_place], jaw, GRIP_CLOSED, self._dur(0.7)),
            Keyframe(np.r_[t[:2], z_place], jaw, GRIP_OPEN, self._dur(0.4)),
            Keyframe(np.r_[t[:2], Z_HOVER + 0.02], jaw, GRIP_OPEN, self._dur(0.6)),
        ]

    # -- kinematics ----------------------------------------------------------------------------
    def _current_keyframe(self) -> Keyframe:
        from sim.kinematics import fk
        q = self.env.joint_state()
        p, R = fk(q)
        jaw = math.atan2(R[1, 1], R[0, 1])
        jaw = _nearest_equiv(jaw, math.atan2(p[1], p[0]) + math.pi / 2, math.pi)
        return Keyframe(p, jaw, float(q[5]), 0)

    def _joint_targets(self, kf: Keyframe) -> np.ndarray:
        try:
            q = self.ik.solve(kf.pos, jaw_angle=None, check_limits=False)
        except IKError:
            q = self.env.joint_state()[:5].astype(float)
        q[4] = self.ik.roll_for_jaw(q[0], kf.jaw)
        a = np.r_[q, kf.grip]
        return np.clip(a, self.env.lo, self.env.hi)


def run_episode(env: TabletopEnv, seed: int, noise: bool = True, extra_steps: int = 10,
                on_step=None) -> tuple[bool, int]:
    """Roll the expert out once. `on_step(obs, action)` is called before each env.step.
    Continues `extra_steps` after success so recorded demos show the arm settling."""
    obs, _ = env.reset(seed=seed)
    expert = ScriptedExpert(env, seed=seed + 10_000, noise=noise)
    success, after = False, 0
    for t in range(env.max_steps):
        a = expert.act(obs).astype(np.float32)
        if on_step is not None:
            on_step(obs, a)
        obs, _, terminated, truncated, info = env.step(a)
        success = success or info["is_success"]
        if success:
            after += 1
            if after > extra_steps:
                break
        if truncated:
            break
    return success, t + 1
