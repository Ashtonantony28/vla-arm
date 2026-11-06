"""Load the authoritative robot spec (spec/kinematics.yaml). Never edit the spec from here."""
from __future__ import annotations

import functools
import pathlib

import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]
SPEC_PATH = REPO / "spec" / "kinematics.yaml"


@functools.lru_cache(maxsize=1)
def load_spec() -> dict:
    with open(SPEC_PATH) as f:
        return yaml.safe_load(f)


def joint(name: str) -> dict:
    for j in load_spec()["joints"]:
        if j["name"] == name:
            return j
    raise KeyError(name)


def mm(v_m: float) -> float:
    """metres -> millimetres"""
    return v_m * 1000.0
