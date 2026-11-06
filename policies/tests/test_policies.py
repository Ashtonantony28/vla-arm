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
