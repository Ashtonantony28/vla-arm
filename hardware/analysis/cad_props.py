"""Mass properties of each link taken from the generated CAD (printed parts at the effective
density, servo cases as point masses at their case centroid, camera board assumed 15 g)."""
from __future__ import annotations

import functools

import numpy as np


@functools.lru_cache(maxsize=1)
def link_mass_props() -> dict[str, dict]:
    from cad import assembly as A, checks as C, params as P

    lump = {"grip_drive": "gripper", "jaw_a": "gripper", "jaw_b": "gripper"}
    acc: dict[str, list] = {}
    tf = A.link_transforms({})
    for it in A.items():
        link = lump.get(it.link, it.link)
        if it.printed:
            m = C.printed_mass_g(it.name, it.solid)
        elif it.name.endswith("_case"):
            m = P.SERVO.mass_g
        elif it.name == "yaw_bearing":
            m = P.BEARING["mass_g"]
        else:
            continue  # horns: negligible
        # express the centroid in the lumped link's frame (jaws/drive coincide with gripper at q=0)
        c = it.solid.Center()
        p = np.array([c.x, c.y, c.z, 1.0])
        if it.link != link:
            p = np.linalg.inv(tf[link]) @ tf[it.link] @ p
        acc.setdefault(link, []).append((m, p[:3]))
    # camera board: assumed 15 g, centred on the mount face
    zcam = P.GRIP_SERVO_Z + P.SERVO.W / 2 + P.POCKET_CLR + 2.5 + 20
    acc["gripper"].append((15.0, np.array([20.0, 0.0, zcam])))
    out = {}
    for k, lst in acc.items():
        M = sum(m for m, _ in lst)
        com = sum(m * p for m, p in lst) / M
        out[k] = dict(mass_kg=M / 1000, com_m=com / 1000)
    return out
