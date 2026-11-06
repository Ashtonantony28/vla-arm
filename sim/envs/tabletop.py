"""Gymnasium-style tabletop env for the VLA arm.

obs = {
  "observation.state":          float32[6]  joint positions (rad), order sim.spec.ACTION_NAMES,
                                            gripper in servo rad (0 closed .. 1.6 open)
  "observation.images.wrist":   uint8[H,W,3]
  "observation.images.scene":   uint8[H,W,3]
  "task":                       str         natural-language instruction
}
action = float32[6] absolute joint-position targets (same order/units as the state), at 30 Hz.
Between control steps the setpoint is ramped linearly at sim rate (like the firmware's setpoint
interpolation) and rate-limited to the spec vmax per joint.
"""
from __future__ import annotations

import math
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import gymnasium as gym
import mujoco
import numpy as np

from sim.envs import tasks as T
from sim.kinematics import TopDownIK
from sim.model.build_mjcf import BLOCK_HALF, CUP_HALF_H, PARK_X
from sim.spec import ACTION_NAMES, load_spec

MODEL_PATH = Path(__file__).resolve().parents[1] / "model" / "vla_arm.xml"
CAMERAS = ("wrist", "scene")
CONTROL_HZ = 30
HOME_TCP = (0.20, 0.0, 0.17)


class TabletopEnv(gym.Env):
    metadata = {"render_fps": CONTROL_HZ}

    def __init__(self, task: str = "pick_place", image_size: int | tuple[int, int] = 224,
                 cameras=CAMERAS, render_images: bool = True, max_steps: int | None = None,
                 instruction_split: str = "train", seed: int | None = None):
        if task not in T.TASKS:
            raise ValueError(f"unknown task {task}; options {list(T.TASKS)}")
        self.task_name = task
        self.spec = load_spec()
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.data = mujoco.MjData(self.model)
        self.h, self.w = (image_size, image_size) if isinstance(image_size, int) else image_size
        self.cameras = tuple(cameras)
        self.render_images = render_images
        self.instruction_split = instruction_split
        self.n_substeps = int(round(1.0 / CONTROL_HZ / self.model.opt.timestep))
        self.max_steps = max_steps or T.TASKS[task].max_steps
        self.ik = TopDownIK(self.spec)
        self.lo, self.hi, self.vmax = self.spec.lower, self.spec.upper, self.spec.vmax
        self._grip_gear = 1.0 / self.spec.finger_travel_per_rad
        self._renderer = None
        self._rng = np.random.default_rng(seed)

        m = self.model
        self._arm_qadr = np.array([m.joint(n).qposadr[0] for n in ACTION_NAMES[:5]])
        self._arm_dadr = np.array([m.joint(n).dofadr[0] for n in ACTION_NAMES[:5]])
        self._finger_qadr = m.joint("finger_l").qposadr[0]
        self._finger_r_qadr = m.joint("finger_r").qposadr[0]
        self._finger_geoms = {m.geom("finger_l_geom").id, m.geom("finger_r_geom").id}
        self.action_space = gym.spaces.Box(self.lo.astype(np.float32), self.hi.astype(np.float32))
        obs_spaces = {"observation.state": gym.spaces.Box(-np.inf, np.inf, (6,), np.float32),
                      "task": gym.spaces.Text(max_length=256)}
        if render_images:
            for c in self.cameras:
                obs_spaces[f"observation.images.{c}"] = gym.spaces.Box(0, 255, (self.h, self.w, 3), np.uint8)
        self.observation_space = gym.spaces.Dict(obs_spaces)
        self.task_spec = T.TASKS[task]
        self.instruction = ""
        self.layout: dict = {}
        self.assignment: dict[str, str] = {}
        self.t = 0

    # ------------------------------------------------------------------------------------------
    def home_qpos(self) -> np.ndarray:
        q = self.ik.solve(HOME_TCP, jaw_angle=math.pi / 2)
        return np.concatenate([q, [1.2]])

    def reset(self, seed: int | None = None, options: dict | None = None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        rng = self._rng
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        ts = self.task_spec
        self.assignment = T.assign_sort(rng) if ts.name == "sort" else {}
        self.layout = T.sample_layout(ts, rng)
        self.instruction = T.instruction_for(ts.name, rng, self.assignment, self.instruction_split)
        # park everything, then place active objects
        for i, n in enumerate(["red_block", "blue_block"]):
            self._set_free(n, (PARK_X, -0.4 - 0.3 * i, BLOCK_HALF), 0.0)
        self._set_free("cup", (PARK_X, 1.4, CUP_HALF_H), 0.0)
        for i, n in enumerate(["green_bowl", "yellow_bowl"]):
            self._set_mocap(n, (PARK_X, 0.4 + 0.3 * i, 0.0), 0.0)
        self._set_mocap("target", (PARK_X, 1.8, 0.0), 0.0)
        for n, (x, y, yaw) in self.layout.items():
            if n.endswith("_block"):
                self._set_free(n, (x, y, BLOCK_HALF + 1e-4), yaw)
            elif n == "cup":
                self._set_free(n, (x, y, CUP_HALF_H + 1e-4), yaw)
            else:
                self._set_mocap(n, (x, y, 0.0), yaw)
        q0 = self.home_qpos()
        q0[:5] += rng.uniform(-0.04, 0.04, 5)
        d.qpos[self._arm_qadr] = q0[:5]
        d.qpos[self._finger_qadr] = d.qpos[self._finger_r_qadr] = q0[5] / self._grip_gear
        d.ctrl[:] = q0
        self._target = q0.copy()
        mujoco.mj_forward(m, d)
        for _ in range(60):  # 0.1 s settle
            mujoco.mj_step(m, d)
        self.t = 0
        return self._obs(), self._info()

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64).reshape(6), self.lo, self.hi)
        m, d = self.model, self.data
        start = d.ctrl.copy()
        dt = m.opt.timestep
        for k in range(1, self.n_substeps + 1):
            ramp = start + (a - start) * (k / self.n_substeps)
            d.ctrl[:] = d.ctrl + np.clip(ramp - d.ctrl, -self.vmax * dt, self.vmax * dt)
            mujoco.mj_step(m, d)
        self.t += 1
        info = self._info()
        success = info["is_success"]
        terminated = bool(success)
        truncated = self.t >= self.max_steps
        return self._obs(), float(success), terminated, truncated, info

    # ------------------------------------------------------------------------------------------
    def joint_state(self) -> np.ndarray:
        q = self.data.qpos[self._arm_qadr]
        g = self.data.qpos[self._finger_qadr] * self._grip_gear
        return np.concatenate([q, [g]]).astype(np.float32)

    def object_pose(self, name: str) -> tuple[np.ndarray, float]:
        """World position and yaw of a body."""
        b = self.data.body(name)
        R = b.xmat.reshape(3, 3)
        return b.xpos.copy(), math.atan2(R[1, 0], R[0, 0])

    def tcp_pos(self) -> np.ndarray:
        return self.data.site("tcp").xpos.copy()

    def touching_fingers(self, body: str) -> bool:
        bid = self.model.body(body).id
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = c.geom1, c.geom2
            if (g1 in self._finger_geoms and self.model.geom_bodyid[g2] == bid) or \
               (g2 in self._finger_geoms and self.model.geom_bodyid[g1] == bid):
                return True
        return False

    def check_success(self) -> bool:
        name = self.task_name
        red, _ = self.object_pose("red_block")
        blue, _ = self.object_pose("blue_block")
        if name == "pick_place":
            bowl, _ = self.object_pose("green_bowl")
            return T.block_in_bowl(red, bowl) and not self.touching_fingers("red_block")
        if name == "stack":
            return T.block_on_block(red, blue) and not self.touching_fingers("red_block")
        if name == "sort":
            ok = True
            for color, pos in (("red", red), ("blue", blue)):
                bowl, _ = self.object_pose(f"{self.assignment[color]}_bowl")
                ok &= T.block_in_bowl(pos, bowl) and not self.touching_fingers(f"{color}_block")
            return bool(ok)
        raise AssertionError(name)

    def render_camera(self, cam: str) -> np.ndarray:
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, self.h, self.w)
        self._renderer.update_scene(self.data, camera=cam)
        return self._renderer.render().copy()

    def render(self):
        return self.render_camera("scene")

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    # ------------------------------------------------------------------------------------------
    def _obs(self) -> dict:
        obs = {"observation.state": self.joint_state(), "task": self.instruction}
        if self.render_images:
            for c in self.cameras:
                obs[f"observation.images.{c}"] = self.render_camera(c)
        return obs

    def _info(self) -> dict:
        return {"is_success": self.check_success(), "t": self.t, "task_name": self.task_name,
                "assignment": dict(self.assignment)}

    def _set_free(self, body: str, pos, yaw: float):
        j = self.model.body(body).jntadr[0]
        adr = self.model.jnt_qposadr[j]
        self.data.qpos[adr:adr + 3] = pos
        self.data.qpos[adr + 3:adr + 7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        dadr = self.model.jnt_dofadr[j]
        self.data.qvel[dadr:dadr + 6] = 0

    def _set_mocap(self, body: str, pos, yaw: float):
        mid = self.model.body(body).mocapid[0]
        self.data.mocap_pos[mid] = pos
        self.data.mocap_quat[mid] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]


def make_env(task: str = "pick_place", **kw) -> TabletopEnv:
    return TabletopEnv(task=task, **kw)
