"""
mesh_loader.py

Generic mesh loading for the 3D orientation view. Mesh format is not
hardcoded to the rendering code: `load_mesh(path)` accepts any format
trimesh understands (glTF/.glb, .obj, .stl, .ply, ...) and returns plain
(vertices, faces) numpy arrays that orientation_widget.py feeds straight
into pyqtgraph's GLMeshItem. To swap the craft model, just pass a
different file path -- no rendering-code changes needed.

Mesh convention expected of any file you supply: body frame with the nose
pointing along +X, right side along +Y, belly along +Z (standard aerospace
body axes), roughly centered on the origin. The bundled procedural
fallback (used when no file is supplied, e.g. before you've picked a CC0
asset) already follows this convention.

Suggested free/CC0 sources for a real "small sleek craft" mesh: Sketchfab's
CC0-license category (filter by license), or any glTF/OBJ file you provide
yourself -- avoid IP-encumbered franchise assets.
"""

from __future__ import annotations

import numpy as np


def load_mesh(path: str | None) -> tuple[np.ndarray, np.ndarray]:
    """Returns (vertices Nx3, faces Mx3). Falls back to a procedural craft
    mesh if `path` is None or fails to load."""
    if path:
        try:
            import trimesh

            mesh = trimesh.load(path, force="mesh")
            verts = np.asarray(mesh.vertices, dtype=float)
            faces = np.asarray(mesh.faces, dtype=int)
            verts = _normalize(verts)
            return verts, faces
        except Exception as exc:
            print(f"mesh_loader: failed to load {path!r} ({exc}); using procedural fallback")

    return _procedural_craft()


def _normalize(verts: np.ndarray) -> np.ndarray:
    """Centers on the origin and scales to a ~2-unit nose-to-tail span, so
    arbitrary supplied meshes render at a consistent size next to the NED
    triad regardless of their original units."""
    center = (verts.max(axis=0) + verts.min(axis=0)) / 2.0
    verts = verts - center
    span = np.max(verts.max(axis=0) - verts.min(axis=0))
    if span > 1e-9:
        verts = verts * (2.0 / span)
    return verts


def _procedural_craft() -> tuple[np.ndarray, np.ndarray]:
    """A small low-poly sci-fi/aerospace-silhouette craft, built from
    scratch (no external asset, no IP concerns): a tapered fuselage with an
    elliptical cross-section, a pointed nose cone, swept delta wings and a
    vertical tail fin. Body frame: +X nose, +Y right, +Z belly (down)."""
    verts: list[list[float]] = []
    faces: list[list[int]] = []

    def add_tri(a, b, c):
        faces.append([a, b, c])

    # ---- fuselage: rings of points around stations along X, nose at +X ----
    n_ring = 10
    stations_x = [-1.00, -0.75, -0.35, 0.10, 0.55, 0.85]
    radii_y = [0.10, 0.16, 0.20, 0.18, 0.11, 0.00]  # width (left-right)
    radii_z = [0.08, 0.13, 0.16, 0.14, 0.08, 0.00]  # height (belly-back)

    ring_start_idx = []
    theta = np.linspace(0, 2 * np.pi, n_ring, endpoint=False)
    for x, ry, rz in zip(stations_x, radii_y, radii_z):
        ring_start_idx.append(len(verts))
        if ry == 0.0 and rz == 0.0:
            verts.append([x, 0.0, 0.0])  # nose tip collapses to a point
        else:
            for th in theta:
                verts.append([x, ry * np.cos(th), rz * np.sin(th)])

    # tail cap (flat polygon fan at the first, non-degenerate station)
    tail_ring = ring_start_idx[0]
    tail_center_idx = len(verts)
    verts.append([stations_x[0], 0.0, 0.0])
    for i in range(n_ring):
        add_tri(tail_center_idx, tail_ring + i, tail_ring + (i + 1) % n_ring)

    # lofted quads between consecutive rings (as pairs of triangles)
    for s in range(len(stations_x) - 1):
        r0 = ring_start_idx[s]
        r1 = ring_start_idx[s + 1]
        n1 = radii_y[s + 1] == 0.0 and radii_z[s + 1] == 0.0
        if n1:
            # next station is the nose point: fan to it instead of lofting quads
            tip = r1
            for i in range(n_ring):
                add_tri(tip, r0 + i, r0 + (i + 1) % n_ring)
        else:
            for i in range(n_ring):
                j = (i + 1) % n_ring
                add_tri(r0 + i, r1 + i, r1 + j)
                add_tri(r0 + i, r1 + j, r0 + j)

    # ---- delta wings (thin triangular prisms so they're visible from both sides) ----
    def add_wing(sign_y: float):
        root_front = [0.05, sign_y * 0.03, 0.0]
        root_back = [-0.55, sign_y * 0.03, 0.0]
        tip = [-0.30, sign_y * 1.05, 0.02]
        dz = 0.015
        top = [np.array(p) + [0, 0, -dz] for p in (root_front, root_back, tip)]
        bot = [np.array(p) + [0, 0, dz] for p in (root_front, root_back, tip)]
        base = len(verts)
        for p in top + bot:
            verts.append(list(p))
        t0, t1, t2, b0, b1, b2 = (base + i for i in range(6))
        winding = 1 if sign_y > 0 else -1
        if winding > 0:
            add_tri(t0, t1, t2)
            add_tri(b0, b2, b1)
        else:
            add_tri(t0, t2, t1)
            add_tri(b0, b1, b2)
        # side faces closing the prism
        for (a0, a1, bb0, bb1) in [(t0, t1, b0, b1), (t1, t2, b1, b2), (t2, t0, b2, b0)]:
            add_tri(a0, a1, bb1)
            add_tri(a0, bb1, bb0)

    add_wing(+1.0)
    add_wing(-1.0)

    # ---- vertical tail fin (small triangular prism, belly-side per body-Z-down) ----
    root_front = [-0.35, 0.0, 0.02]
    root_back = [-0.80, 0.0, 0.02]
    tip = [-0.55, 0.0, -0.45]
    dy = 0.012
    left = [np.array(p) + [0, -dy, 0] for p in (root_front, root_back, tip)]
    right = [np.array(p) + [0, dy, 0] for p in (root_front, root_back, tip)]
    base = len(verts)
    for p in left + right:
        verts.append(list(p))
    l0, l1, l2, r0, r1, r2 = (base + i for i in range(6))
    add_tri(l0, l1, l2)
    add_tri(r0, r2, r1)
    for (a0, a1, b0b, b1b) in [(l0, l1, r0, r1), (l1, l2, r1, r2), (l2, l0, r2, r0)]:
        add_tri(a0, a1, b1b)
        add_tri(a0, b1b, b0b)

    return _normalize(np.array(verts, dtype=float)), np.array(faces, dtype=int)
