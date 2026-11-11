"""Dump the vertices of every CAD item (printed parts, servo cases/horns, bearing) per link, in that
link's frame (metres), to an .npz.  Run with the hardware venv, from hardware/:

    cd hardware && <hardware venv>/bin/python ../armctl/tools/dump_cad_links.py /tmp/links.npz

The output feeds armctl/tools/fit_capsules.py, which prints the numbers kept in
armctl/armctl/data/collision_model.yaml.
"""
import sys
from collections import defaultdict

import numpy as np

from cad import assembly as A

out = sys.argv[1] if len(sys.argv) > 1 else "links.npz"
pts = defaultdict(list)
for it in A.items(teeth=False):
    verts, tris = it.solid.tessellate(0.2, 0.2)
    V = np.array([[p.x, p.y, p.z] for p in verts])
    v = [V]
    # sample face interiors (barycentric grid, ~2 mm spacing) so large flat faces are covered
    for tri in tris:
        a, b, c = V[list(tri)]
        n = int(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / 2.0) + 1
        uu, ww = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        m = uu + ww <= n
        u, w = uu[m] / n, ww[m] / n
        v.append(a + np.outer(u, b - a) + np.outer(w, c - a))
    # sample edges densely so long straight edges of large flat faces are represented
    for e in it.solid.Edges():
        n = max(2, int(e.Length() / 2.0))
        v.append(np.array([[p.x, p.y, p.z] for p in (e.positionAt(t) for t in np.linspace(0, 1, n))]))
    v = np.vstack(v) / 1000.0
    pts[it.link].append(v)
    lo, hi = np.round(v.min(0) * 1000, 1), np.round(v.max(0) * 1000, 1)
    print(f"{it.name:28s} link={it.link:10s} n={len(v):6d} bbox_mm {lo} .. {hi}")
np.savez(out, **{k: np.vstack(v) for k, v in pts.items()})
