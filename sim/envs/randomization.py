"""Domain randomisation for TabletopEnv (PLAN 5: "vary lighting between sessions").

Everything is applied by editing MjModel fields in place at reset (no XML change), from a random
stream that is *separate* from the layout/instruction stream: for a given seed the object layout and
the instruction are the same with or without randomisation, so randomised and nominal evals use the
same 20 start poses.

What is randomised (`DRConfig`, all ranges are uniform):
* lighting: key-light direction (tilted up to `light_tilt_deg` from nominal), key/fill intensity
  and colour temperature, headlight ambient/diffuse;
* textures/colours: table tint (low saturation, so it never looks like a task colour), checker
  repeat, arm shade; object colours jittered *inside their colour class* so the instruction still
  names them unambiguously (red stays red, blue stays blue, green/yellow bowls stay green/yellow);
  the distractor cup and target mark get a random low-saturation colour;
* camera pose jitter: position +-`*_cam_pos_mm`, orientation +-`*_cam_rot_deg` about a random axis;
* action noise: Gaussian on the 5 arm joint targets inside `env.step` (the commanded action that a
  recorder stores is the clean one -- DART-style noise injection, so demos contain small recoveries).
"""
from __future__ import annotations

import colorsys
import math
from dataclasses import asdict, dataclass, replace

import mujoco
import numpy as np


@dataclass(frozen=True)
class DRConfig:
    lighting: bool = True
    colors: bool = True
    camera: bool = True
    action_noise: float = 0.0          # rad, std of noise added to the 5 arm joint targets
    light_tilt_deg: float = 25.0
    light_scale: tuple = (0.55, 1.35)  # key-light intensity multiplier
    fill_scale: tuple = (0.4, 1.6)
    color_temp: float = 0.12           # +- per-channel tint of light colour
    headlight_ambient: tuple = (0.2, 0.5)
    headlight_diffuse: tuple = (0.3, 0.65)
    scene_cam_pos_mm: float = 5.0
    scene_cam_rot_deg: float = 2.0
    wrist_cam_pos_mm: float = 2.0
    wrist_cam_rot_deg: float = 1.5


PRESETS = {
    "none": None,
    "visual": DRConfig(),
    "default": DRConfig(action_noise=0.004),
}

# hue ranges (HSV hue in [0,1)) per colour class; red wraps around 0.
COLOR_CLASSES = {
    "red": dict(h=(-0.025, 0.025), s=(0.65, 0.95), v=(0.6, 0.95)),
    "blue": dict(h=(0.58, 0.66), s=(0.6, 0.9), v=(0.55, 0.95)),
    "green": dict(h=(0.29, 0.40), s=(0.55, 0.85), v=(0.45, 0.8)),
    "yellow": dict(h=(0.12, 0.16), s=(0.7, 0.95), v=(0.8, 1.0)),
}
BODY_CLASS = {"red_block": "red", "blue_block": "blue", "green_bowl": "green", "yellow_bowl": "yellow"}
NEUTRAL_BODIES = ("cup", "target")
ARM_BODIES_EXCLUDE = set(BODY_CLASS) | set(NEUTRAL_BODIES) | {"world"}


def resolve(randomize) -> DRConfig | None:
    if randomize is None or randomize is False:
        return None
    if randomize is True:
        return PRESETS["default"]
    if isinstance(randomize, str):
        if randomize not in PRESETS:
            raise ValueError(f"unknown randomisation preset {randomize}; options {list(PRESETS)}")
        return PRESETS[randomize]
    if isinstance(randomize, DRConfig):
        return randomize
    if isinstance(randomize, dict):
        return replace(PRESETS["default"], **randomize)
    raise TypeError(f"bad randomize argument {randomize!r}")


def hsv_to_rgb(h: float, s: float, v: float) -> np.ndarray:
    return np.array(colorsys.hsv_to_rgb(h % 1.0, s, v))


def rgb_hue_class(rgb) -> str | None:
    """Which task colour class an RGB triple falls in (None if outside all of them)."""
    h, s, v = colorsys.rgb_to_hsv(*[float(x) for x in rgb[:3]])
    for name, c in COLOR_CLASSES.items():
        lo, hi = c["h"]
        hh = h - 1.0 if lo < 0 and h > 0.5 else h
        if lo - 1e-6 <= hh <= hi + 1e-6 and s >= c["s"][0] - 1e-6:
            return name
    return None


def _quat_mul(a, b):
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(a, float), np.asarray(b, float))
    return out


def _small_rotation(rng, max_deg):
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    ang = math.radians(rng.uniform(-max_deg, max_deg))
    return np.r_[math.cos(ang / 2), math.sin(ang / 2) * axis]


class DomainRandomizer:
    def __init__(self, model: mujoco.MjModel, cfg: DRConfig):
        self.m = model
        self.cfg = cfg
        m = model
        self.nominal = dict(
            light_dir=m.light_dir.copy(), light_diffuse=m.light_diffuse.copy(),
            light_specular=m.light_specular.copy(),
            head_amb=m.vis.headlight.ambient.copy(), head_dif=m.vis.headlight.diffuse.copy(),
            geom_rgba=m.geom_rgba.copy(), mat_rgba=m.mat_rgba.copy(), mat_texrepeat=m.mat_texrepeat.copy(),
            cam_pos=m.cam_pos.copy(), cam_quat=m.cam_quat.copy(),
        )
        body_of = m.geom_bodyid
        self.body_geoms: dict[str, np.ndarray] = {}
        for b in list(BODY_CLASS) + list(NEUTRAL_BODIES):
            self.body_geoms[b] = np.nonzero(body_of == m.body(b).id)[0]
        self.arm_geoms = np.array([g for g in range(m.ngeom)
                                   if m.body(int(body_of[g])).name not in ARM_BODIES_EXCLUDE])
        self.table_mat = m.material("table").id
        self.cam_ids = {m.camera(i).name: i for i in range(m.ncam)}
        self.last: dict = {}

    def restore(self):
        m, n = self.m, self.nominal
        m.light_dir[:] = n["light_dir"]
        m.light_diffuse[:] = n["light_diffuse"]
        m.light_specular[:] = n["light_specular"]
        m.vis.headlight.ambient[:] = n["head_amb"]
        m.vis.headlight.diffuse[:] = n["head_dif"]
        m.geom_rgba[:] = n["geom_rgba"]
        m.mat_rgba[:] = n["mat_rgba"]
        m.mat_texrepeat[:] = n["mat_texrepeat"]
        m.cam_pos[:] = n["cam_pos"]
        m.cam_quat[:] = n["cam_quat"]

    def apply(self, rng: np.random.Generator) -> dict:
        """Randomise the model in place; returns a JSON-able record of what was drawn."""
        self.restore()
        m, c, n = self.m, self.cfg, self.nominal
        rec: dict = {}
        # Draw every random number regardless of which groups are enabled, so toggling one group
        # does not change the others' draws for the same seed.
        tilt = _small_rotation(rng, c.light_tilt_deg)
        key_s, fill_s = rng.uniform(*c.light_scale), rng.uniform(*c.fill_scale)
        tint = 1.0 + rng.uniform(-c.color_temp, c.color_temp, 3)
        h_amb, h_dif = rng.uniform(*c.headlight_ambient), rng.uniform(*c.headlight_diffuse)
        obj_cols = {b: hsv_to_rgb(rng.uniform(*COLOR_CLASSES[k]["h"]), rng.uniform(*COLOR_CLASSES[k]["s"]),
                                  rng.uniform(*COLOR_CLASSES[k]["v"])) for b, k in BODY_CLASS.items()}
        neutral = {b: hsv_to_rgb(rng.uniform(0, 1), rng.uniform(0, 0.25), rng.uniform(0.35, 0.95))
                   for b in NEUTRAL_BODIES}
        table = hsv_to_rgb(rng.uniform(0, 1), rng.uniform(0, 0.22), rng.uniform(0.7, 1.15))
        texrep = rng.uniform(3, 10)
        arm_shade = rng.uniform(0.75, 1.3)
        cam_draws = {name: (rng.uniform(-1, 1, 3), _small_rotation(rng, 1.0)) for name in sorted(self.cam_ids)}

        if c.lighting:
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, tilt)
            d0 = n["light_dir"][0]
            d = R.reshape(3, 3) @ d0
            if d[2] > -0.3:  # keep the key light pointing down at the table
                d[2] = -0.3
            m.light_dir[0] = d / np.linalg.norm(d)
            m.light_diffuse[0] = np.clip(n["light_diffuse"][0] * key_s * tint, 0, 1)
            if m.nlight > 1:
                m.light_diffuse[1] = np.clip(n["light_diffuse"][1] * fill_s * tint, 0, 1)
            m.vis.headlight.ambient[:] = h_amb
            m.vis.headlight.diffuse[:] = h_dif
            rec.update(key_dir=m.light_dir[0].round(4).tolist(), key_scale=float(key_s),
                       fill_scale=float(fill_s), tint=tint.round(4).tolist(),
                       headlight=[float(h_amb), float(h_dif)])
        if c.colors:
            for b, rgb in {**obj_cols, **neutral}.items():
                m.geom_rgba[self.body_geoms[b], :3] = rgb
            m.mat_rgba[self.table_mat, :3] = np.clip(table, 0, 1.2)
            m.mat_texrepeat[self.table_mat] = texrep
            m.geom_rgba[self.arm_geoms, :3] = np.clip(n["geom_rgba"][self.arm_geoms, :3] * arm_shade, 0, 1)
            rec.update(colors={b: v.round(4).tolist() for b, v in {**obj_cols, **neutral}.items()},
                       table=table.round(4).tolist(), texrepeat=float(texrep), arm_shade=float(arm_shade))
        if c.camera:
            for name, (dp, q_unit) in cam_draws.items():
                i = self.cam_ids[name]
                pos_mm = c.wrist_cam_pos_mm if name == "wrist" else c.scene_cam_pos_mm
                rot_deg = c.wrist_cam_rot_deg if name == "wrist" else c.scene_cam_rot_deg
                m.cam_pos[i] = n["cam_pos"][i] + dp * pos_mm * 1e-3
                # rescale the unit-degree rotation draw to rot_deg
                ang1 = 2 * math.atan2(np.linalg.norm(q_unit[1:]), q_unit[0])
                axis = q_unit[1:] / max(np.linalg.norm(q_unit[1:]), 1e-12)
                ang = ang1 * rot_deg
                dq = np.r_[math.cos(ang / 2), math.sin(ang / 2) * axis]
                m.cam_quat[i] = _quat_mul(n["cam_quat"][i], dq)
                rec[f"cam_{name}"] = {"dpos_mm": (dp * pos_mm).round(3).tolist(),
                                      "rot_deg": float(math.degrees(ang))}
        self.last = rec
        return rec


def config_dict(cfg: DRConfig | None) -> dict | None:
    return None if cfg is None else asdict(cfg)
