"""Policy smoke tests: one training step lowers loss, inference shapes, eval end to end."""
import numpy as np
import pytest
import torch

pytest.importorskip("lerobot")

from policies.data import collate  # noqa: E402
from policies.text_encoder import HashedBowEncoder  # noqa: E402
from policies.train import add_language, build, train  # noqa: E402
from sim.record import record  # noqa: E402


@pytest.fixture(scope="module")
def tiny_ds(tmp_path_factory):
    root = tmp_path_factory.mktemp("data") / "ds"
    record("pick_place", 2, root, image_size=32, verbose=False)
    return root


@pytest.mark.parametrize("kind", ["act", "act_lang", "diffusion"])
def test_training_step_decreases_loss(tiny_ds, kind):
    torch.manual_seed(0)
    cfg, ds, policy, pre, post, enc = build(kind, [tiny_ds], augment_shift=0, text_encoder="hashed_bow",
                                            chunk_size=10)
    if kind == "act":
        assert cfg.chunk_size == 10 and cfg.temporal_ensemble_coeff is not None
    batch = pre(add_language(collate([ds[i] for i in range(0, 64, 8)]), enc))
    opt = torch.optim.Adam(policy.parameters(), 1e-3)
    policy.train()
    torch.manual_seed(1)
    l0 = policy.forward(batch)[0].item()
    for _ in range(5):
        torch.manual_seed(1)  # same VAE / diffusion noise each time so losses are comparable
        loss, _ = policy.forward(batch)
        opt.zero_grad()
        loss.backward()
        opt.step()
    torch.manual_seed(1)
    assert policy.forward(batch)[0].item() < l0


def test_act_chunk_shape(tiny_ds):
    cfg, ds, policy, pre, post, enc = build("act", [tiny_ds], augment_shift=0, chunk_size=50)
    policy.eval()
    b = pre(collate([ds[0]]))
    with torch.no_grad():
        chunk = policy.predict_action_chunk(b)
        assert chunk.shape == (1, 50, 6)
        a = post(policy.select_action(b))
    assert a.shape == (1, 6)


def test_hashed_text_encoder():
    enc = HashedBowEncoder()
    e = enc.encode(["put the red block in the green bowl and the blue block in the yellow bowl",
                    "put the blue block in the green bowl and the red block in the yellow bowl",
                    "put the red block in the green bowl and the blue block in the yellow bowl"])
    assert e.shape == (3, 256)
    assert np.allclose(e[0], e[2])
    assert not np.allclose(e[0], e[1]), "sort assignment must change the embedding"


def test_train_and_eval_end_to_end(tiny_ds, tmp_path):
    from policies.eval_sim import evaluate

    r = train("act_lang", [tiny_ds], tmp_path / "run", steps=2, batch_size=4, text_encoder="hashed_bow",
              chunk_size=10, verbose=False)
    s = evaluate(r["checkpoint"], "pick_place", episodes=2, max_steps=10, verbose=False)
    assert s["episodes"] == 2 and len(s["per_episode"]) == 2
    assert 0.0 <= s["wilson95"][0] <= s["wilson95"][1] <= 1.0


# ---- wave 2: from-scratch small CNN backbone, EE state, augmentation, strict reload -----------

def test_small_cnn_shapes():
    from policies.backbones import SmallCNN

    for stride, hw in ((8, 12), (16, 6)):
        y = SmallCNN(stride=stride)(torch.zeros(2, 3, 96, 96))
        assert y.shape[-2:] == (hw, hw)


def test_state_transform_matches_mujoco_tcp():
    from policies.state import transform_state
    from sim.envs.tabletop import TabletopEnv

    env = TabletopEnv("pick_place", render_images=False)
    env.reset(seed=2)
    s = transform_state(env.joint_state(), "joints_ee")
    assert s.shape == (9,)
    assert np.allclose(s[6:], env.tcp_pos(), atol=1e-4)
    assert transform_state(np.zeros((4, 3, 6)), "joints_ee").shape == (4, 3, 9)


def test_color_jitter_and_shift_are_shared_across_window(tiny_ds):
    from policies.data import ChunkDataset

    ds = ChunkDataset([tiny_ds], list(range(4)), [-1, 0], augment_shift=3, color_jitter=0.3,
                      state_mode="joints_ee")
    item = ds[20]
    img = item["observation.images.wrist"]
    assert img.shape[0] == 2 and 0.0 <= img.min() and img.max() <= 1.0
    assert item["observation.state"].shape == (2, 9)
    assert ds.stats()["observation.state"]["mean"].shape == (9,)
    # same aug on both frames: at an episode start the window repeats the first frame
    first = ds[int(ds.ep_start[0])]["observation.images.wrist"]
    assert torch.allclose(first[0], first[1])


@pytest.mark.parametrize("kind", ["act", "diffusion"])
def test_small_cnn_train_save_strict_reload(tiny_ds, tmp_path, kind):
    from policies.eval_sim import evaluate, load_policy

    bb = {"name": "small_cnn", "stride": 16, "width": 16}
    r = train(kind, [tiny_ds], tmp_path / kind, steps=2, batch_size=4, chunk_size=16, verbose=False,
              backbone=bb, state_mode="joints_ee", color_jitter=0.2, val_roots=[tiny_ds], val_every=1)
    assert r["final_val_loss"] is not None
    policy, pre, post, enc, meta = load_policy(r["checkpoint"])
    assert meta["backbone"] == bb and meta["state_mode"] == "joints_ee"
    assert any("backbone.net" in n for n, _ in policy.named_parameters())
    s = evaluate(r["checkpoint"], "pick_place", episodes=1, max_steps=5, verbose=False, randomize="default")
    assert s["episodes"] == 1 and s["per_episode"][0]["category"] in ("missed_grasp", "dropped", "wrong_object")
