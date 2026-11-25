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


# ---------------------------------------------------------------- rounded / profiled primitives
def rbox(x0, x1, y0, y1, z0, z1, r: float = 0.0, edges: str = "|Z") -> cq.Shape:
    """Box with the edges parallel to one axis filleted (edges = '|X', '|Y' or '|Z'). Fillets are
    applied to the primitive before any boolean, which keeps OCCT robust."""
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    w = cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
    if r > 0:
        w = w.edges(edges).fillet(r)
    return w.val()


def chamfer_all(shape: cq.Shape, c: float, selector: str | None = None) -> cq.Shape:
    w = cq.Workplane().add(shape)
    w = w.edges(selector) if selector else w.edges()
    try:
        return w.chamfer(c).val()
    except Exception:
        return shape


def fillet_all(shape: cq.Shape, r: float, selector: str | None = None) -> cq.Shape:
    w = cq.Workplane().add(shape)
    w = w.edges(selector) if selector else w.edges()
    try:
        return w.fillet(r).val()
    except Exception:
        return shape


def plate_xz(R: float, x_end: float, h: float, y0: float, y1: float, end_r: float = 3.0) -> cq.Shape:
    """Link side plate normal to y: disc of radius R about the joint axis (origin), straight tangent
    flanks tapering to half-height h at x = x_end, square end with rounded corners. True arcs, so the
    B-rep has no facets."""
    # tangent from circle (0,0,R) to point (x_end, h): for R >= h the flank is the common tangent of
    # the circle and the horizontal line at h; with R > h the flank leaves the circle at angle phi.
    d = x_end
    # line from tangent point on circle to (x_end, +-h): choose the tangent line through (d, h)
    L = math.hypot(d, h)
    phi = math.atan2(h, d) + math.acos(R / L)          # angle of tangent point on circle
    tx, tz = R * math.cos(phi), R * math.sin(phi)
    pts_top = (tx, tz)
    wp = (cq.Workplane("XZ", origin=(0, y1, 0))  # XZ workplane normal is -Y; extrude toward -y
          .moveTo(d, -h + end_r)
          .lineTo(d, h - end_r)
          .threePointArc((d - end_r * (1 - math.cos(math.pi / 4)), h - end_r * (1 - math.sin(math.pi / 4))),
                         (d - end_r, h))
          .lineTo(*pts_top)
          .threePointArc((-R, 0), (tx, -tz))
          .lineTo(d - end_r, -h)
          .threePointArc((d - end_r * (1 - math.cos(math.pi / 4)), -h + end_r * (1 - math.sin(math.pi / 4))),
                         (d, -h + end_r))
          .close()
          .extrude(y1 - y0))
    return wp.val()
