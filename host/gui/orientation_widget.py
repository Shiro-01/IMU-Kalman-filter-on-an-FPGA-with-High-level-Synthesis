"""
orientation_widget.py

3D orientation view: a rigid-body craft mesh that rotates live to track
roll/pitch/yaw (phi, theta, psi) from either or both state sources, drawn
against a fixed NED reference triad, with separate magnetic-north and
true-north pointers offset by a user-editable declination.

Rendering: pyqtgraph.opengl (Qt/OpenGL widget, same GL context family as
the rest of the app -- no second 3D engine to manage). Mesh geometry comes
from mesh_loader.load_mesh(), completely decoupled from this module: swap
the asset file, nothing here changes.

Coordinate conventions
-----------------------
Body frame (mesh-local):     +X nose, +Y right, +Z belly (down)   [aerospace]
NED (world, sensor-fusion):  +X North, +Y East, +Z Down
Display frame (OpenGL, Z-up): +X East, +Y North, +Z Up

Body -> NED uses the same aerospace ZYX Euler convention the EKF's own
accelerometer model (EKF._ha) is built on:
    R_nb = Rz(psi) @ Ry(theta) @ Rx(phi)
(verified algebraically against EKF._ha's closed form).

NED -> display is a fixed axis permutation/flip (a proper rotation, no
mirroring): east=NED_y, north=NED_x, up=-NED_z.
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PySide6 import QtWidgets

from mesh_loader import load_mesh

NED_TO_ENU = np.array([
    [0.0, 1.0, 0.0],   # display X (East)  = NED Y (East)
    [1.0, 0.0, 0.0],   # display Y (North) = NED X (North)
    [0.0, 0.0, -1.0],  # display Z (Up)    = -NED Z (Down)
])

HW_COLOR = (0.0, 0.83, 1.0, 0.55)     # cyan, translucent
SW_COLOR = (1.0, 0.69, 0.0, 0.55)     # amber, translucent
TRIAD_LEN = 1.8
NORTH_PTR_LEN = 1.5

# Axis-role mapping for reconciling a loaded mesh's own local axes with the
# body-frame convention (see OrientationView.set_mesh_axes). Only nose/belly
# are user-chosen; "right" is always derived, so an invalid (non-orthogonal
# or handedness-flipping) combination can never be constructed.
AXIS_LABELS = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"]
AXIS_VECTORS = {
    "+X": np.array([1.0, 0.0, 0.0]), "-X": np.array([-1.0, 0.0, 0.0]),
    "+Y": np.array([0.0, 1.0, 0.0]), "-Y": np.array([0.0, -1.0, 0.0]),
    "+Z": np.array([0.0, 0.0, 1.0]), "-Z": np.array([0.0, 0.0, -1.0]),
}


def _Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def body_to_ned(phi: float, theta: float, psi: float) -> np.ndarray:
    """Aerospace ZYX Euler convention, matching EKF._ha's model exactly."""
    return _Rz(psi) @ _Ry(theta) @ _Rx(phi)


def _transform_matrix(phi: float, theta: float, psi: float) -> np.ndarray:
    """4x4 local(body) -> display(ENU) transform for a GLGraphicsItem."""
    r = NED_TO_ENU @ body_to_ned(phi, theta, psi)
    m = np.eye(4)
    m[:3, :3] = r
    return m


class OrientationView(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._mode = "Both"
        self._declination_deg = 4.2
        self._mesh_path: str | None = None

        self.gl_view = gl.GLViewWidget()
        self.gl_view.setCameraPosition(distance=6.0, elevation=25, azimuth=45)
        self.gl_view.setBackgroundColor((10, 14, 20))

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.gl_view)

        self._build_static_scene()
        self._build_meshes()
        self.set_attitudes((0, 0, 0), (0, 0, 0))

    # ------------------------------------------------------------------
    def _build_static_scene(self) -> None:
        grid = gl.GLGridItem()
        grid.setSize(x=8, y=8)
        grid.setSpacing(x=0.5, y=0.5)
        grid.setColor((40, 55, 70, 120))
        self.gl_view.addItem(grid)

        def axis_line(direction, color, length):
            pos = np.array([[0, 0, 0], np.array(direction) * length])
            item = gl.GLLinePlotItem(pos=pos, color=color, width=3, antialias=True)
            self.gl_view.addItem(item)
            return item

        # NED triad, drawn in display(ENU) coordinates
        north_enu = NED_TO_ENU @ np.array([1.0, 0.0, 0.0])
        east_enu = NED_TO_ENU @ np.array([0.0, 1.0, 0.0])
        down_enu = NED_TO_ENU @ np.array([0.0, 0.0, 1.0])

        axis_line(north_enu, (0.6, 0.6, 0.6, 1.0), TRIAD_LEN)
        axis_line(east_enu, (0.6, 0.6, 0.6, 1.0), TRIAD_LEN)
        axis_line(down_enu, (0.6, 0.6, 0.6, 1.0), TRIAD_LEN)

        self._label("N", north_enu * TRIAD_LEN, (200, 200, 200, 255))
        self._label("E", east_enu * TRIAD_LEN, (200, 200, 200, 255))
        self._label("D", down_enu * TRIAD_LEN, (200, 200, 200, 255))

        # Magnetic-north pointer (fixed, along NED North) and true-north
        # pointer (fixed, offset by declination east of magnetic north).
        # true_heading = magnetic_heading + declination_east, so the true
        # north direction (in ENU) leans toward +East by declination.
        self.mag_north_item = axis_line(north_enu, (255, 210, 0, 255), NORTH_PTR_LEN)
        self.true_north_item = gl.GLLinePlotItem(
            pos=np.array([[0, 0, 0], [0, NORTH_PTR_LEN, 0]]),
            color=(255, 60, 60, 255), width=3, antialias=True,
        )
        self.gl_view.addItem(self.true_north_item)

        self._label("Mag N", north_enu * NORTH_PTR_LEN, (255, 210, 0, 255))
        self.true_north_label = gl.GLTextItem(
            pos=[0, NORTH_PTR_LEN, 0.05], text="True N", color=(255, 60, 60, 255)
        )
        self.gl_view.addItem(self.true_north_label)

        self._update_true_north()

    def _label(self, text, pos, color) -> gl.GLTextItem:
        item = gl.GLTextItem(pos=list(pos), text=text, color=color)
        self.gl_view.addItem(item)
        return item

    def _build_meshes(self) -> None:
        self._mesh_verts_raw, self._faces = load_mesh(self._mesh_path)
        self._mesh_axis_correction = np.eye(3)  # identity: nose=+X, right=+Y, belly=+Z
        verts = self._mesh_verts_raw

        self.hw_mesh = gl.GLMeshItem(
            vertexes=verts, faces=self._faces, smooth=True, drawEdges=False,
            color=HW_COLOR, shader="shaded", glOptions="translucent",
        )
        self.sw_mesh = gl.GLMeshItem(
            vertexes=verts, faces=self._faces, smooth=True, drawEdges=False,
            color=SW_COLOR, shader="shaded", glOptions="translucent",
        )
        self.gl_view.addItem(self.hw_mesh)
        self.gl_view.addItem(self.sw_mesh)
        self._apply_display_mode()

    # ------------------------------------------------------------------
    def set_mesh(self, path: str | None) -> None:
        self._mesh_path = path
        self._mesh_verts_raw, self._faces = load_mesh(path)
        self._mesh_axis_correction = np.eye(3)  # a new mesh needs its own mapping, not the old one
        self._apply_mesh_orientation()

    def set_mesh_axes(self, nose_axis: str, belly_axis: str) -> None:
        """One-time correction baked into the mesh's vertices at load time,
        to reconcile an arbitrary supplied mesh's own local axes (there's no
        universal convention -- glTF/Sketchfab exports are typically Y-up
        with whatever forward direction the original scene happened to use)
        with the body-frame convention this view expects (+X nose, +Y right,
        +Z belly/down).

        `nose_axis`/`belly_axis` are one of AXIS_LABELS ("+X","-X","+Y","-Y",
        "+Z","-Z"), naming which axis of the *mesh's own* local coordinates
        points toward the nose / toward the belly. They must be mutually
        perpendicular. The third (right) axis is deliberately not a free
        choice -- it's derived as belly x nose, the only vector that keeps
        the result a valid right-handed rotation (never an accidental
        mirror or a non-orthogonal, invalid frame)."""
        nose_vec = AXIS_VECTORS[nose_axis]
        belly_vec = AXIS_VECTORS[belly_axis]
        if abs(np.dot(nose_vec, belly_vec)) > 1e-6:
            raise ValueError(f"nose_axis {nose_axis!r} and belly_axis {belly_axis!r} must be perpendicular")

        right_vec = np.cross(belly_vec, nose_vec)
        self._mesh_axis_correction = np.vstack([nose_vec, right_vec, belly_vec])
        self._apply_mesh_orientation()

    def _apply_mesh_orientation(self) -> None:
        verts = self._mesh_verts_raw @ self._mesh_axis_correction.T
        self.hw_mesh.setMeshData(vertexes=verts, faces=self._faces)
        self.sw_mesh.setMeshData(vertexes=verts, faces=self._faces)

    def set_hw_color(self, rgba: tuple) -> None:
        """rgba: 4-tuple, components in [0,1]."""
        self.hw_mesh.setColor(rgba)

    def set_sw_color(self, rgba: tuple) -> None:
        self.sw_mesh.setColor(rgba)

    def set_display_mode(self, mode: str) -> None:
        """mode: 'Hardware', 'Software', or 'Both'."""
        self._mode = mode
        self._apply_display_mode()

    def _apply_display_mode(self) -> None:
        self.hw_mesh.setVisible(self._mode in ("Hardware", "Both"))
        self.sw_mesh.setVisible(self._mode in ("Software", "Both"))

    def set_declination_deg(self, declination_east: float) -> None:
        self._declination_deg = declination_east
        self._update_true_north()

    def _update_true_north(self) -> None:
        decl = np.radians(self._declination_deg)
        # ENU: North=+Y, East=+X. Leaning `decl` toward East from North.
        direction = np.array([np.sin(decl), np.cos(decl), 0.0]) * NORTH_PTR_LEN
        self.true_north_item.setData(pos=np.array([[0, 0, 0], direction]))
        self.true_north_label.setData(pos=direction + np.array([0, 0, 0.05]))

    def set_attitudes(self, hw_rpy, sw_rpy) -> None:
        """hw_rpy, sw_rpy: (phi, theta, psi) in radians."""
        m_hw = _transform_matrix(*hw_rpy)
        m_sw = _transform_matrix(*sw_rpy)
        self.hw_mesh.setTransform(pg.Transform3D(*m_hw.flatten()))
        self.sw_mesh.setTransform(pg.Transform3D(*m_sw.flatten()))
