"""Domain randomisation: seeded, reproducible, bounded, and keeps task colours identifiable."""
import colorsys
import math

import mujoco
import numpy as np
import pytest

from sim.envs.randomization import COLOR_CLASSES, PRESETS, DRConfig, rgb_hue_class
from sim.envs.tabletop import MODEL_PATH, TabletopEnv


def _rgba(env, body):
    m = env.model
    g = np.nonzero(m.geom_bodyid == m.body(body).id)[0][0]
    return m.geom_rgba[g, :3].copy()


def test_same_seed_same_images_and_params():
    a = TabletopEnv("pick_place", image_size=32, randomize="default")
    b = TabletopEnv("pick_place", image_size=32, randomize="default")
    oa, _ = a.reset(seed=11)
    a.reset(seed=12)  # advance, then come back
    oa2, _ = a.reset(seed=11)
    ob, _ = b.reset(seed=11)
    for k in ("observation.images.wrist", "observation.images.scene"):
        assert np.array_equal(oa[k], ob[k]) and np.array_equal(oa[k], oa2[k])
    assert a.dr_params == b.dr_params
    oc, _ = b.reset(seed=13)
    assert not np.array_equal(oa["observation.images.scene"], oc["observation.images.scene"])
    a.close()
    b.close()


def test_layout_and_instruction_independent_of_randomisation():
    plain = TabletopEnv("sort", render_images=False)
    dr = TabletopEnv("sort", render_images=False, randomize="default")
    for s in range(5):
        plain.reset(seed=s)
        dr.reset(seed=s)
        assert plain.layout == dr.layout and plain.instruction == dr.instruction
        assert np.allclose(plain.joint_state(), dr.joint_state())


def test_nominal_env_is_unrandomised():
    env = TabletopEnv("pick_place", render_images=False)
    ref = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    env.reset(seed=3)
    assert env.dr_params == {}
    assert np.array_equal(env.model.geom_rgba, ref.geom_rgba)
    assert np.array_equal(env.model.cam_pos, ref.cam_pos)
    assert np.array_equal(env.model.light_diffuse, ref.light_diffuse)


def test_colours_stay_in_their_class_and_distractors_are_neutral():
    env = TabletopEnv("sort", render_images=False, randomize="visual")
    seen_red = []
    for s in range(60):
        env.reset(seed=s)
        for body, cls in (("red_block", "red"), ("blue_block", "blue"), ("green_bowl", "green"),
                          ("yellow_bowl", "yellow")):
            assert rgb_hue_class(_rgba(env, body)) == cls, (s, body, _rgba(env, body))
        for body in ("cup", "target"):
            assert rgb_hue_class(_rgba(env, body)) is None
        tbl = env.model.mat_rgba[env.model.material("table").id, :3]
        assert colorsys.rgb_to_hsv(*np.clip(tbl, 0, 1))[1] <= 0.25  # low-saturation table
        seen_red.append(_rgba(env, "red_block"))
    assert np.std(np.array(seen_red), axis=0).max() > 0.02  # it actually varies


def test_camera_and_light_jitter_bounded():
    cfg = PRESETS["default"]
    env = TabletopEnv("pick_place", render_images=False, randomize=cfg)
    nom_pos = env._dr.nominal["cam_pos"].copy()
    nom_quat = env._dr.nominal["cam_quat"].copy()
    nom_dir = env._dr.nominal["light_dir"][0]
    moved = 0.0
    for s in range(30):
        env.reset(seed=s)
        for i in range(env.model.ncam):
            name = env.model.camera(i).name
            mm = cfg.wrist_cam_pos_mm if name == "wrist" else cfg.scene_cam_pos_mm
            deg = cfg.wrist_cam_rot_deg if name == "wrist" else cfg.scene_cam_rot_deg
            d = np.abs(env.model.cam_pos[i] - nom_pos[i])
            assert d.max() <= mm * 1e-3 + 1e-9
            moved = max(moved, d.max())
            dot = abs(float(np.dot(env.model.cam_quat[i], nom_quat[i])))
            assert math.degrees(2 * math.acos(min(1.0, dot))) <= deg + 1e-6
        kd = env.model.light_dir[0]
        assert kd[2] < 0
        assert math.degrees(math.acos(np.clip(np.dot(kd, nom_dir), -1, 1))) <= cfg.light_tilt_deg + 20
    assert moved > 1e-4


def test_action_noise_seeded():
    def rollout(randomize, seed):
        env = TabletopEnv("pick_place", render_images=False, randomize=randomize)
        obs, _ = env.reset(seed=seed)
        a = obs["observation.state"].copy()
        for _ in range(5):
            obs, *_ = env.step(a)
        return obs["observation.state"]

    noisy = dict(lighting=False, colors=False, camera=False, action_noise=0.01)
    s1, s2 = rollout(noisy, 4), rollout(noisy, 4)
    assert np.array_equal(s1, s2)
    assert not np.allclose(rollout(noisy, 4), rollout(None, 4), atol=1e-4)
    assert not np.allclose(s1, rollout(noisy, 5), atol=1e-6)


def test_group_toggles_do_not_shift_other_draws():
    a = TabletopEnv("pick_place", render_images=False, randomize=DRConfig(lighting=False))
    b = TabletopEnv("pick_place", render_images=False, randomize=DRConfig())
    a.reset(seed=21)
    b.reset(seed=21)
    assert a.dr_params["colors"] == b.dr_params["colors"]
    assert a.dr_params["cam_scene"] == b.dr_params["cam_scene"]
    assert "key_dir" not in a.dr_params


def test_bad_preset():
    with pytest.raises(ValueError):
        TabletopEnv("pick_place", render_images=False, randomize="nope")


def test_color_classes_disjoint():
    names = list(COLOR_CLASSES)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            (a0, a1), (b0, b1) = COLOR_CLASSES[a]["h"], COLOR_CLASSES[b]["h"]
            assert a1 < b0 or b1 < a0
