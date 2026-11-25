"""Offscreen renders of the posed assembly (pyrender + EGL), used to eyeball the model.

    PYOPENGL_PLATFORM=egl .venv/bin/python -m cad.render [--out cad/out/renders]
"""
from __future__ import annotations

import argparse
import os
import pathlib

os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

import numpy as np
import pyrender
import trimesh
from PIL import Image

from . import assembly as A

COLORS = {
    "printed": (0.93, 0.47, 0.15, 1.0),
    "tpu": (0.15, 0.15, 0.17, 1.0),
    "servo": (0.12, 0.13, 0.16, 1.0),
    "horn": (0.80, 0.80, 0.82, 1.0),
    "metal": (0.70, 0.72, 0.75, 1.0),
    "pcb": (0.10, 0.45, 0.20, 1.0),
    "spring": (0.75, 0.75, 0.20, 1.0),
}


def color_of(it) -> tuple:
    n = it.name
    if n.startswith("pad"):
        return COLORS["tpu"]
    if it.printed:
        return COLORS["printed"]
    if n == "camera_board":
        return COLORS["pcb"]
    if "screw" in n or "axle" in n:
        return (0.25, 0.25, 0.28, 1.0)
    if n.endswith("_horn"):
        return COLORS["horn"]
    if "case" in n:
        return COLORS["servo"]
    return COLORS["metal"]


def to_trimesh(shape, tol=0.05, ang=0.15) -> trimesh.Trimesh:
    v, f = shape.tessellate(tol, ang)
    return trimesh.Trimesh(np.array([[p.x, p.y, p.z] for p in v]), np.array(f), process=False)


def look_at(eye, target, up=(0, 0, 1)) -> np.ndarray:
    eye, target, up = map(lambda a: np.asarray(a, float), (eye, target, up))
    f = target - eye; f /= np.linalg.norm(f)
    if abs(np.dot(f, up)) > 0.999 * np.linalg.norm(up):   # looking along 'up' (top view): use +y as up
        up = np.array([0.0, 1.0, 0.0])
    s = np.cross(f, up); s /= np.linalg.norm(s)
    u = np.cross(s, f)
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = s, u, -f, eye
    return m


VIEWS = {
    "iso": ((560, -560, 440), (210, 0, 90)),
    "front": ((800, 0, 130), (230, 0, 90)),
    "side": ((230, -680, 100), (230, 0, 90)),
    "top": ((230, 0, 720), (230, 0, 60)),
    "rear_iso": ((-120, 420, 360), (230, 0, 90)),
}


def render(q=None, out="cad/out/renders", prefix="pose_zero", views=None, size=(1200, 900),
           zoom_target=None, zoom_dist=None, highlight=None):
    out = pathlib.Path(out); out.mkdir(parents=True, exist_ok=True)
    scene = pyrender.Scene(bg_color=(1, 1, 1, 1), ambient_light=(0.22, 0.22, 0.22))
    for it, shp in A.posed(q, teeth=True):
        c = color_of(it)
        if highlight and it.name in highlight:
            c = (0.9, 0.1, 0.1, 1.0)
        mat = pyrender.MetallicRoughnessMaterial(baseColorFactor=c, metallicFactor=0.1, roughnessFactor=0.6)
        scene.add(pyrender.Mesh.from_trimesh(to_trimesh(shp), material=mat, smooth=False))
    r = pyrender.OffscreenRenderer(*size)
    paths = []
    for name, (eye, tgt) in (views or VIEWS).items():
        if zoom_target is not None:
            tgt = zoom_target
            d = np.asarray(eye, float) - np.asarray(VIEWS[name][1], float)
            eye = np.asarray(tgt) + d / np.linalg.norm(d) * (zoom_dist or 200)
        pose = look_at(eye, tgt)
        cam = scene.add(pyrender.PerspectiveCamera(yfov=np.radians(35)), pose=pose)
        lights = [scene.add(pyrender.DirectionalLight(intensity=1.8), pose=pose),
                  scene.add(pyrender.DirectionalLight(intensity=1.0), pose=look_at((0, 0, 900), (150, 0, 0), (1, 0, 0)))]
        img, _ = r.render(scene, flags=pyrender.RenderFlags.RGBA)
        p = out / f"{prefix}_{name}.png"
        im = Image.fromarray(img, "RGBA")
        bg = Image.new("RGBA", im.size, (255, 255, 255, 255))
        Image.alpha_composite(bg, im).convert("RGB").save(p)
        paths.append(p)
        scene.remove_node(cam)
        for l in lights:
            scene.remove_node(l)
    r.delete()
    return paths


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="cad/out/renders")
    a = ap.parse_args()
    render(out=a.out)
    render({"shoulder_pitch": -0.6, "elbow_pitch": 1.2, "wrist_pitch": 0.6, "base_yaw": 0.5,
            "wrist_roll": 0.4, "gripper": 1.2}, out=a.out, prefix="pose_work")
    # close-ups at zero pose: gripper + camera, base + shoulder, elbow
    iso = {"iso": VIEWS["iso"], "rear_iso": VIEWS["rear_iso"]}
    render(out=a.out, prefix="closeup_gripper", views=iso, zoom_target=(460, 0, 110), zoom_dist=200)
    render(out=a.out, prefix="closeup_base", views=iso, zoom_target=(0, 0, 60), zoom_dist=260)
    render(out=a.out, prefix="closeup_elbow", views=iso, zoom_target=(200, 0, 100), zoom_dist=180)
