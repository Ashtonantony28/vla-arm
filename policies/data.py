"""Training data: LeRobot datasets -> in-memory action-chunk samples.

The source of truth is a LeRobot dataset on disk (sim/record.py or the real robot's recorder).
For small datasets on CPU, decoding MP4 frames for every random sample dominates the step time,
so `ChunkDataset` decodes every frame once (sequential pass through `LeRobotDataset`) and keeps
uint8 tensors in RAM: 30 episodes x ~170 frames x 2 cams x 96x96x3 ~ 100 MB.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
from torch.utils.data import Dataset


def load_lerobot(root: str | Path, episodes: list[int] | None = None):
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(root)
    return LeRobotDataset(repo_id=f"local/{root.name}", root=root, episodes=episodes)


class ChunkDataset(Dataset):
    """Samples = (obs at t, actions t..t+chunk-1 with padding mask), LeRobot key names."""

    def __init__(self, roots: list[str | Path], action_delta: list[int], obs_delta: list[int] | None = None,
                 episodes: list[int] | None = None, augment_shift: int = 0):
        """action_delta / obs_delta: frame offsets relative to t, exactly as a LeRobot policy config's
        `action_delta_indices` / `observation_delta_indices` (None = single frame, no time axis)."""
        self.action_delta = np.asarray(action_delta)
        self.obs_delta = None if obs_delta is None else np.asarray(obs_delta)
        self.augment_shift = augment_shift
        states, actions, tasks, ep_ids, imgs = [], [], [], [], {}
        self.camera_keys: list[str] | None = None
        self.metas = []
        offset = 0
        for root in roots:
            ds = load_lerobot(root, episodes)
            self.metas.append(ds.meta)
            cams = list(ds.meta.camera_keys)
            if self.camera_keys is None:
                self.camera_keys = cams
                imgs = {c: [] for c in cams}
            elif cams != self.camera_keys:
                raise ValueError(f"camera keys differ between datasets: {cams} vs {self.camera_keys}")
            self.fps = ds.meta.fps
            for i in range(len(ds)):
                s = ds[i]
                states.append(s["observation.state"].numpy())
                actions.append(s["action"].numpy())
                tasks.append(s["task"])
                ep_ids.append(int(s["episode_index"]) + offset)
                for c in cams:
                    imgs[c].append((s[c].numpy() * 255.0 + 0.5).astype(np.uint8))  # CHW
            offset = max(ep_ids) + 1
        self.state = torch.from_numpy(np.stack(states)).float()
        self.action = torch.from_numpy(np.stack(actions)).float()
        self.images = {c: torch.from_numpy(np.stack(v)) for c, v in imgs.items()}
        self.tasks = tasks
        self.ep = np.asarray(ep_ids)
        # first / last+1 frame index of each sample's episode
        self.ep_start = np.zeros(len(self.ep), dtype=np.int64)
        self.ep_end = np.zeros(len(self.ep), dtype=np.int64)
        for e in np.unique(self.ep):
            idx = np.nonzero(self.ep == e)[0]
            self.ep_start[idx] = idx[0]
            self.ep_end[idx] = idx[-1] + 1
        self.task_names = sorted(set(tasks))

    def __len__(self):
        return len(self.ep)

    def _shift(self, img: torch.Tensor) -> torch.Tensor:
        p = self.augment_shift
        if p <= 0:
            return img
        c, h, w = img.shape
        padded = torch.nn.functional.pad(img[None].float(), (p, p, p, p), mode="replicate")[0]
        dx, dy = np.random.randint(0, 2 * p + 1, 2)
        return padded[:, dy:dy + h, dx:dx + w]

    def _window(self, i: int, delta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        idx = i + delta
        pad = (idx < self.ep_start[i]) | (idx >= self.ep_end[i])
        return np.clip(idx, self.ep_start[i], self.ep_end[i] - 1), pad

    def __getitem__(self, i: int) -> dict:
        aidx, apad = self._window(i, self.action_delta)
        item = {"action": self.action[aidx], "action_is_pad": torch.from_numpy(apad), "task": self.tasks[i]}
        if self.obs_delta is None:
            item["observation.state"] = self.state[i]
            for c in self.camera_keys:
                item[c] = self._shift(self.images[c][i]).float() / 255.0
        else:
            oidx, opad = self._window(i, self.obs_delta)
            item["observation.state"] = self.state[oidx]
            item["observation.state_is_pad"] = torch.from_numpy(opad)
            for c in self.camera_keys:
                item[c] = torch.stack([self._shift(self.images[c][j]) for j in oidx]).float() / 255.0
        return item

    def stats(self) -> dict:
        """Normalisation stats in LeRobot's layout, aggregated across all source datasets."""
        from lerobot.datasets.compute_stats import aggregate_stats

        st = aggregate_stats([m.stats for m in self.metas]) if len(self.metas) > 1 else self.metas[0].stats
        return {k: {kk: torch.as_tensor(np.asarray(vv), dtype=torch.float32) for kk, vv in v.items()}
                for k, v in st.items()}


def collate(batch: list[dict]) -> dict:
    out = {}
    for k in batch[0]:
        if k == "task":
            out[k] = [b[k] for b in batch]
        else:
            out[k] = torch.stack([b[k] for b in batch])
    return out
