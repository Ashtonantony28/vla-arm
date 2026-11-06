"""Assembly: forward kinematics from spec/kinematics.yaml, part placement, interference checks."""
from __future__ import annotations

import functools
import itertools
from dataclasses import dataclass

import cadquery as cq
import numpy as np

from spec_loader import load_spec, mm
from . import params as P
from . import parts as PT
from .geom import R, T, place

LINKS = ["base", "shoulder", "upper_arm", "forearm", "wrist", "gripper", "grip_drive", "jaw_a", "jaw_b"]
ARM_JOINTS = [j["name"] for j in load_spec()["joints"]]
# parent link of each link (kinematic tree)
PARENT = {"shoulder": "base", "upper_arm": "shoulder", "forearm": "upper_arm", "wrist": "forearm",
          "gripper": "wrist", "grip_drive": "gripper", "jaw_a": "gripper", "jaw_b": "gripper"}
# which joint moves each link relative to its parent
JOINT_OF = {"shoulder": "base_yaw", "upper_arm": "shoulder_pitch", "forearm": "elbow_pitch",
            "wrist": "wrist_pitch", "gripper": "wrist_roll", "grip_drive": "gripper",
            "jaw_a": "gripper", "jaw_b": "gripper"}


def link_transforms(q: dict[str, float] | None = None) -> dict[str, np.ndarray]:
    """4x4 transforms (mm) of every link frame in the world frame, straight from the spec chain."""
    q = dict(q or {})
    spec = load_spec()
    out = {"base": np.eye(4)}
    m = np.eye(4)
    chain = ["shoulder", "upper_arm", "forearm", "wrist", "gripper"]
    for link, j in zip(chain, spec["joints"]):
        o = [mm(v) for v in j["origin"]]
        m = m @ T(*o) @ R(j["axis"], q.get(j["name"], 0.0))
        out[link] = m
    g = out["gripper"]
    qg = q.get("gripper", 0.0)
    s = P.PINION_R * qg  # rack travel per jaw (mm); opening = 2 s
    out["grip_drive"] = g @ T(0, 0, P.GRIP_SERVO_Z) @ R((1, 0, 0), qg) @ T(0, 0, -P.GRIP_SERVO_Z)
    out["jaw_a"] = g @ T(0, s, 0)
    out["jaw_b"] = g @ T(0, -s, 0)
    return out


def tcp_world(q=None) -> np.ndarray:
    return (link_transforms(q)["gripper"] @ np.array([P.TCP_L, 0, 0, 1]))[:3]


@dataclass
class Item:
    name: str
    link: str
    solid: cq.Shape
    printed: bool


@functools.lru_cache(maxsize=2)
def items(teeth: bool = False) -> tuple[Item, ...]:
    out = []
    for name, (_, link, _) in PT.PRINTED.items():
        out.append(Item(name, link, PT.build_printed(name, teeth), True))
    body, horn = PT.servo_body(), PT.servo_horn()
    # servo cases (fixed to the link that holds them) and horns (fixed to the driven link)
    servos = [
        ("servo_yaw", "base", "shoulder", PT.MOUNT_YAW),
        ("servo_shoulder", "shoulder", "upper_arm", T(0, 0, P.SHOULDER_Z) @ PT.mount_pitch(0.0)),
        ("servo_elbow", "upper_arm", "forearm", PT.mount_pitch(P.UPPER_ARM_L)),
        ("servo_wrist_pitch", "forearm", "wrist", PT.mount_pitch(P.FOREARM_L)),
        # the roll servo CASE rides in the gripper; its horn is bolted to the wrist bracket
        ("servo_roll", "gripper", "wrist", T(P.WRIST_L, 0, 0) @ PT.MOUNT_ROLL),
        ("servo_gripper", "gripper", "grip_drive", PT.MOUNT_GRIP),
    ]
    for name, case_link, horn_link, m in servos:
        # mount matrices are expressed in the case link frame, except the roll servo (wrist frame)
        if name == "servo_roll":
            m_case = PT.MOUNT_ROLL
            m_horn = m  # in wrist frame
        elif name == "servo_shoulder":
            m_case, m_horn = m, PT.mount_pitch(0.0)   # horn in upper-arm frame (axis at origin)
        elif name in ("servo_elbow", "servo_wrist_pitch"):
            m_case, m_horn = m, PT.mount_pitch(0.0)
        elif name == "servo_yaw":
            m_case, m_horn = m, np.eye(4)             # horn face = shoulder (yaw) frame origin
        else:  # gripper servo: horn rotates with grip_drive, whose frame coincides with gripper at q=0
            m_case, m_horn = m, m
        out.append(Item(name + "_case", case_link, place(body, m_case), False))
        out.append(Item(name + "_horn", horn_link, place(horn, m_horn), False))
    out.append(Item("yaw_bearing", "base", PT.yaw_bearing(), False))
    return tuple(out)


def posed(q=None, teeth=False) -> list[tuple[Item, cq.Shape]]:
    tf = link_transforms(q)
    return [(it, place(it.solid, tf[it.link])) for it in items(teeth)]


def _bb_overlap(a: cq.Shape, b: cq.Shape, tol=0.0) -> bool:
    A, B = a.BoundingBox(), b.BoundingBox()
    return not (A.xmax < B.xmin - tol or B.xmax < A.xmin - tol or A.ymax < B.ymin - tol or
                B.ymax < A.ymin - tol or A.zmax < B.zmin - tol or B.zmax < A.zmin - tol)


def intersection_volume(a: cq.Shape, b: cq.Shape) -> float:
    if not _bb_overlap(a, b):
        return 0.0
    return float(a.intersect(b).Volume())


def descendants(link: str) -> set[str]:
    out = {link}
    changed = True
    while changed:
        changed = False
        for c, p in PARENT.items():
            if p in out and c not in out:
                out.add(c); changed = True
    return out


def adjacent(l1: str, l2: str) -> bool:
    return PARENT.get(l1) == l2 or PARENT.get(l2) == l1 or (
        {l1, l2} <= {"jaw_a", "jaw_b", "grip_drive"} and l1 != l2)


def collisions(q=None, links_moving: set[str] | None = None, adjacent_only=True, vol_tol=1.0, teeth=False):
    """Return [(item_a, item_b, volume_mm3)] for pairs with intersection volume > vol_tol.
    If links_moving is given, only pairs with exactly one side in links_moving are checked."""
    pp = posed(q, teeth)
    hits = []
    for (ia, sa), (ib, sb) in itertools.combinations(pp, 2):
        if ia.link == ib.link:
            continue
        if links_moving is not None and ((ia.link in links_moving) == (ib.link in links_moving)):
            continue
        if adjacent_only and not adjacent(ia.link, ib.link):
            continue
        v = intersection_volume(sa, sb)
        if v > vol_tol:
            hits.append((ia.name, ib.name, v))
    return hits
