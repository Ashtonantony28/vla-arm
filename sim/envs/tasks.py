"""Task definitions: which objects are active, how they are randomised, instructions, success."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from sim.model.build_mjcf import BLOCK_HALF, BOWL_RADIUS, BOWL_WALL_T, BOWL_WALL_H

# Workspace on the table in polar coords around the base (reachable for a top-down grasp).
R_RANGE = (0.17, 0.31)
TH_RANGE = (-0.85, 0.85)

BOWL_INNER = BOWL_RADIUS - 2 * BOWL_WALL_T

# Paraphrases. The first `n_train` of each list are used for training data; the rest are held out
# for language-generalisation eval (PLAN 5 / 6.6).
PARAPHRASES = {
    "pick_place": [
        "put the red block in the bowl",
        "drop the red cube into the bowl",
        "place the red block inside the bowl",
        "move the red cube to the bowl",
        "pick up the red block and put it in the bowl",
    ],
    "stack": [
        "stack the red block on the blue block",
        "put the red cube on top of the blue cube",
        "place the red block onto the blue one",
        "stack red on blue",
        "pick up the red block and set it on the blue block",
    ],
    # {a},{b} are block colours, {A},{B} the bowl colours they go to
    "sort": [
        "put the {a} block in the {A} bowl and the {b} block in the {B} bowl",
        "sort the blocks: {a} into the {A} bowl, {b} into the {B} bowl",
        "move the {a} cube to the {A} bowl and the {b} cube to the {B} bowl",
        "the {a} block goes in the {A} bowl, the {b} block in the {B} bowl",
        "place {a} in {A} and {b} in {B}",
    ],
}
N_TRAIN_PARAPHRASES = 3


@dataclass
class TaskSpec:
    name: str
    blocks: list[str]
    bowls: list[str]
    max_steps: int
    distractor_prob: float = 0.3
    # filled at reset
    assignment: dict[str, str] = field(default_factory=dict)  # block colour -> bowl colour


TASKS = {
    "pick_place": TaskSpec("pick_place", blocks=["red", "blue"], bowls=["green"], max_steps=330),
    "stack": TaskSpec("stack", blocks=["red", "blue"], bowls=[], max_steps=330),
    "sort": TaskSpec("sort", blocks=["red", "blue"], bowls=["green", "yellow"], max_steps=600),
}


def instruction_for(task: str, rng: np.random.Generator, assignment: dict[str, str] | None = None,
                    split: str = "train") -> str:
    opts = PARAPHRASES[task]
    pool = opts[:N_TRAIN_PARAPHRASES] if split == "train" else opts[N_TRAIN_PARAPHRASES:]
    if split == "all":
        pool = opts
    tmpl = pool[int(rng.integers(len(pool)))]
    if task == "sort":
        a, b = "red", "blue"
        if rng.random() < 0.5:  # mention order is random too
            a, b = b, a
        return tmpl.format(a=a, b=b, A=assignment[a], B=assignment[b])
    return tmpl


def sample_layout(task: TaskSpec, rng: np.random.Generator) -> dict[str, tuple[float, float, float]]:
    """Return {object_name: (x, y, yaw)} for every active object, with no overlaps."""
    radius = {"block": 0.03, "bowl": BOWL_RADIUS + 0.012, "cup": 0.045, "target": 0.04}
    names = [f"{c}_bowl" for c in task.bowls] + [f"{c}_block" for c in task.blocks]
    if rng.random() < task.distractor_prob:
        names.append("cup")
    if rng.random() < task.distractor_prob:
        names.append("target")
    for _attempt in range(200):
        placed: dict[str, tuple[float, float, float]] = {}
        ok = True
        for n in names:
            kind = n.split("_")[-1] if "_" in n else n
            for _ in range(100):
                r = rng.uniform(*R_RANGE)
                th = rng.uniform(*TH_RANGE)
                x, y = r * math.cos(th), r * math.sin(th)
                if all(math.hypot(x - px, y - py) > radius[kind] + radius[_kind(pn)]
                       for pn, (px, py, _) in placed.items()):
                    placed[n] = (x, y, float(rng.uniform(-math.pi / 4, math.pi / 4)))
                    break
            else:
                ok = False
                break
        if ok:
            return placed
    raise RuntimeError("could not sample a non-overlapping layout")


def _kind(name: str) -> str:
    return name.split("_")[-1] if "_" in name else name


def assign_sort(rng: np.random.Generator) -> dict[str, str]:
    if rng.random() < 0.5:
        return {"red": "green", "blue": "yellow"}
    return {"red": "yellow", "blue": "green"}


# ---- success predicates (operate on a SceneState snapshot, see tabletop.py) ------------------

def block_in_bowl(block_pos: np.ndarray, bowl_pos: np.ndarray) -> bool:
    d = math.hypot(block_pos[0] - bowl_pos[0], block_pos[1] - bowl_pos[1])
    return d < BOWL_INNER - 0.5 * BLOCK_HALF and block_pos[2] < BOWL_WALL_H


def block_on_block(top: np.ndarray, bottom: np.ndarray) -> bool:
    d = math.hypot(top[0] - bottom[0], top[1] - bottom[1])
    dz = top[2] - bottom[2]
    return d < BLOCK_HALF and abs(dz - 2 * BLOCK_HALF) < 0.006 and bottom[2] < BLOCK_HALF + 0.004
