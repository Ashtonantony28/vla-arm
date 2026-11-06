"""Env reset/step shapes, success checks, expert sanity."""
import numpy as np
import pytest

from sim.envs.tabletop import TabletopEnv
from sim.envs.tasks import PARAPHRASES, TASKS
from sim.expert.scripted import run_episode
from sim.metrics import wilson_interval


@pytest.mark.parametrize("task", list(TASKS))
def test_reset_step_shapes(task):
    env = TabletopEnv(task, image_size=48)
    obs, info = env.reset(seed=1)
    assert obs["observation.state"].shape == (6,) and obs["observation.state"].dtype == np.float32
    for c in ("wrist", "scene"):
        im = obs[f"observation.images.{c}"]
        assert im.shape == (48, 48, 3) and im.dtype == np.uint8 and im.std() > 1
    assert isinstance(obs["task"], str) and obs["task"]
    assert not info["is_success"], "task must not start solved"
    obs2, r, term, trunc, info = env.step(obs["observation.state"])
    assert obs2["observation.state"].shape == (6,)
    assert r == 0.0 and not term and not trunc
    assert env.observation_space["observation.state"].shape == (6,)
    env.close()


def test_holding_still_does_not_succeed():
    env = TabletopEnv("pick_place", render_images=False, max_steps=30)
    obs, _ = env.reset(seed=3)
    for _ in range(30):
        obs, _, term, trunc, info = env.step(obs["observation.state"])
    assert not info["is_success"] and trunc


def test_reset_is_deterministic_per_seed():
    env = TabletopEnv("sort", render_images=False)
    env.reset(seed=7)
    a = (dict(env.layout), env.instruction)
    env.reset(seed=8)
    env.reset(seed=7)
    assert (dict(env.layout), env.instruction) == a


def test_sort_instruction_matches_assignment():
    env = TabletopEnv("sort", render_images=False)
    for s in range(10):
        env.reset(seed=s)
        for color, bowl in env.assignment.items():
            words = env.instruction.split()
            # the bowl colour named right after the block colour is its assigned bowl
            i = words.index(color)
            assert bowl in words[i:i + 6]


def test_paraphrases_exist():
    for task, p in PARAPHRASES.items():
        assert len(p) >= 4


@pytest.mark.parametrize("task", list(TASKS))
def test_expert_solves_and_success_fires(task):
    env = TabletopEnv(task, render_images=False)
    oks = [run_episode(env, s, extra_steps=0)[0] for s in range(3)]
    assert all(oks)
    assert env.check_success()


def test_success_check_rejects_block_on_table():
    env = TabletopEnv("stack", render_images=False)
    env.reset(seed=0)
    assert not env.check_success()


def test_wilson():
    lo, hi = wilson_interval(10, 20)
    assert 0.29 < lo < 0.30 and 0.70 < hi < 0.71
    assert wilson_interval(0, 20)[0] == 0.0
