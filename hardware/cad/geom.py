"""Tiny solid-modelling helpers on top of CadQuery/OCCT (axis-aligned boxes and cylinders)."""
from __future__ import annotations

import math

import cadquery as cq
import numpy as np
from OCP.gp import gp_Trsf

V = cq.Vector


def box(x0, x1, y0, y1, z0, z1) -> cq.Shape:
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, pnt=V(x0, y0, z0))


def cyl(r, p0, p1) -> cq.Shape:
    """Cylinder of radius r from point p0 to point p1."""
    a, b = np.asarray(p0, float), np.asarray(p1, float)
    d = b - a
    h = float(np.linalg.norm(d))
    return cq.Solid.makeCylinder(r, h, pnt=V(*a), dir=V(*(d / h)))


def ring(r_in, r_out, p0, p1) -> cq.Shape:
    return cyl(r_out, p0, p1).cut(cyl(r_in, p0, p1))


def union(*shapes) -> cq.Shape:
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    return out.clean()


def cut(a, *tools) -> cq.Shape:
    out = a
    for t in tools:
        out = out.cut(t)
    return out.clean()


def single_solid(shape: cq.Shape) -> cq.Solid:
    sols = shape.Solids()
    if len(sols) != 1:
        raise ValueError(f"expected one solid, got {len(sols)}")
    return sols[0]


# ---------------------------------------------------------------- transforms (4x4, mm)
def T(x=0.0, y=0.0, z=0.0) -> np.ndarray:
    m = np.eye(4); m[:3, 3] = (x, y, z); return m


def R(axis, q) -> np.ndarray:
    ax = np.asarray(axis, float); ax = ax / np.linalg.norm(ax)
    x, y, z = ax; c, s = math.cos(q), math.sin(q); C = 1 - c
    m = np.eye(4)
    m[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return m


def to_loc(m: np.ndarray) -> cq.Location:
    t = gp_Trsf()
    t.SetValues(*[float(v) for v in m[:3, :].ravel()])
    return cq.Location(t)


def place(shape: cq.Shape, m: np.ndarray) -> cq.Shape:
    return shape.moved(to_loc(m))
