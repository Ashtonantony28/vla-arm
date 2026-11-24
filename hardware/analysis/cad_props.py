"""Mass properties of each link taken from the generated CAD (printed parts at the effective
density, servo cases as point masses at their case centroid, camera board assumed 15 g, modelled as a part)."""
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
        elif it.name.startswith("idler_bearing"):
            m = P.IDLER_BEARING["mass_g"]
        elif it.name == "camera_board":
            m = 15.0  # assumed USB camera board + M12 lens
        else:
            continue  # horns: negligible
        # express the centroid in the lumped link's frame (jaws/drive coincide with gripper at q=0)
        c = it.solid.Center()
        p = np.array([c.x, c.y, c.z, 1.0])
        if it.link != link:
            p = np.linalg.inv(tf[link]) @ tf[it.link] @ p
        acc.setdefault(link, []).append((m, p[:3]))
    out = {}
    for k, lst in acc.items():
        M = sum(m for m, _ in lst)
        com = sum(m * p for m, p in lst) / M
        out[k] = dict(mass_kg=M / 1000, com_m=com / 1000)
    return out
