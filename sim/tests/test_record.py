"""Dataset round-trip: record 2 episodes with the LeRobot writer, reload, check shapes/fps/stats."""
import json

import numpy as np
import pytest

lerobot = pytest.importorskip("lerobot")

from sim.record import record  # noqa: E402


def test_record_roundtrip(tmp_path):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = tmp_path / "ds"
    s = record("pick_place", 2, root, image_size=32, verbose=False)
    assert s["episodes"] == 2
    ds = LeRobotDataset("local/ds", root=root)
    assert ds.num_episodes == 2
    assert ds.fps == 30
    assert len(ds) == s["frames"]
    item = ds[0]
    assert item["observation.state"].shape == (6,)
    assert item["action"].shape == (6,)
    for c in ("wrist", "scene"):
        assert item[f"observation.images.{c}"].shape == (3, 32, 32)
    assert item["task"] in ("put the red block in the bowl", "drop the red cube into the bowl",
                            "place the red block inside the bowl")
    # stats are consistent with the actual data
    acts = np.stack([ds[i]["action"].numpy() for i in range(len(ds))])
    st = ds.meta.stats["action"]
    assert np.allclose(st["mean"], acts.mean(0), atol=1e-4)
    assert np.allclose(st["min"], acts.min(0), atol=1e-5)
    assert np.allclose(st["max"], acts.max(0), atol=1e-5)
    info = json.loads((root / "meta" / "info.json").read_text())
    assert info["robot_type"] == "vla_arm_sim" and info["fps"] == 30
    assert len((root / "meta" / "sim_episodes.jsonl").read_text().splitlines()) == 2
    assert list(root.rglob("*.mp4")), "videos must be MP4 files"
