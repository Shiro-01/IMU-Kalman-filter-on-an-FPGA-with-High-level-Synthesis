"""
mesh_settings_dialog.py

A separate, non-modal window holding everything related to the 3D craft
mesh: loading a file, fixing up its axis convention, and picking colors/
opacity for the hardware/software overlays. Pulled out of the main
window's toolbar so that toolbar only carries acquisition/view controls;
this stays open (or reopenable) independently while you tweak the mesh.
"""

from __future__ import annotations

import numpy as np
from PySide6 import QtCore, QtGui, QtWidgets

import theme
from orientation_widget import AXIS_LABELS, AXIS_VECTORS, HW_COLOR, SW_COLOR, OrientationView


class MeshSettingsDialog(QtWidgets.QDialog):
    def __init__(self, orientation_view: OrientationView, status_callback, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Mesh Settings")
        self.setMinimumWidth(380)
        # Non-modal: stays open and live-updates the 3D view while the main
        # window remains fully usable (Start/Stop, source switching, etc.).
        self.setModal(False)

        self.orientation_view = orientation_view
        self._status_callback = status_callback

        self._build_ui()

    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)

        load_row = QtWidgets.QHBoxLayout()
        self.mesh_btn = QtWidgets.QPushButton("Load Mesh...")
        self.mesh_btn.clicked.connect(self._on_load_mesh)
        load_row.addWidget(self.mesh_btn)
        load_row.addStretch(1)
        layout.addLayout(load_row)

        layout.addWidget(self._build_axis_group())
        layout.addWidget(self._build_color_group())
        layout.addStretch(1)

    def _build_axis_group(self) -> QtWidgets.QGroupBox:
        # A freshly loaded mesh's own local axes rarely match the body-frame
        # convention this view expects (+X nose, +Y right, +Z belly/down) --
        # there's no way to detect "which way is the nose" from geometry
        # alone. Rather than dialing in a rotation by eye, you directly say
        # which of the mesh's own axes is the nose and which is the belly;
        # "right" is derived automatically (belly x nose) so only valid,
        # orthogonal, correctly-handed combinations are ever reachable -- the
        # Belly dropdown is filtered live to the 4 axes perpendicular to
        # whatever Nose axis is selected.
        group = QtWidgets.QGroupBox("Axis alignment")
        form = QtWidgets.QFormLayout(group)

        self.mesh_nose_combo = self._make_axis_combo()
        self.mesh_nose_combo.addItems(AXIS_LABELS)
        self.mesh_nose_combo.setCurrentText("+X")
        self.mesh_nose_combo.currentTextChanged.connect(self._on_mesh_nose_changed)
        form.addRow("Nose axis:", self.mesh_nose_combo)

        self.mesh_belly_combo = self._make_axis_combo()
        form.addRow("Belly axis:", self.mesh_belly_combo)

        reset_btn = QtWidgets.QPushButton("Reset alignment")
        reset_btn.clicked.connect(self._on_reset_mesh_alignment)
        form.addRow(reset_btn)

        self._populate_belly_options(nose_label="+X", preferred_belly="+Z")
        self.mesh_belly_combo.currentTextChanged.connect(self._on_mesh_axes_changed)
        return group

    def _build_color_group(self) -> QtWidgets.QGroupBox:
        # Each swatch button shows the current color and opens a
        # QColorDialog (forced to Qt's own cross-platform dialog, not the
        # native OS picker, so a hex/"Html" entry field is always available
        # regardless of platform) with an alpha slider; a separate Opacity
        # slider next to it avoids reopening that dialog just to adjust
        # translucency.
        group = QtWidgets.QGroupBox("Colors")
        form = QtWidgets.QFormLayout(group)

        self.hw_color = QtGui.QColor.fromRgbF(*HW_COLOR)
        self.hw_color_btn = self._make_color_swatch(self.hw_color)
        self.hw_color_btn.clicked.connect(lambda: self._on_pick_color("hw"))
        self.hw_opacity_slider, self.hw_opacity_label = self._make_opacity_slider("hw", self.hw_color.alphaF())
        form.addRow("Hardware color:", self._hbox(self.hw_color_btn, self.hw_opacity_slider, self.hw_opacity_label))

        self.sw_color = QtGui.QColor.fromRgbF(*SW_COLOR)
        self.sw_color_btn = self._make_color_swatch(self.sw_color)
        self.sw_color_btn.clicked.connect(lambda: self._on_pick_color("sw"))
        self.sw_opacity_slider, self.sw_opacity_label = self._make_opacity_slider("sw", self.sw_color.alphaF())
        form.addRow("Software color:", self._hbox(self.sw_color_btn, self.sw_opacity_slider, self.sw_opacity_label))
        return group

    @staticmethod
    def _hbox(*widgets) -> QtWidgets.QWidget:
        container = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(container)
        h.setContentsMargins(0, 0, 0, 0)
        for w in widgets:
            h.addWidget(w)
        h.addStretch(1)
        return container

    # ------------------------------------------------------------------
    def _make_axis_combo(self) -> QtWidgets.QComboBox:
        """A QComboBox sized to fit its (short, e.g. '-Z') items comfortably.
        Without an explicit minimum width, a combo box populated only with
        2-character labels sizes itself down to almost nothing, and its
        popup list inherits that same width -- clipping every item to just
        its sign. AdjustToContents keeps it sized to the longest entry as
        items are swapped in (the belly dropdown's options change live)."""
        combo = QtWidgets.QComboBox()
        combo.setMinimumWidth(70)
        combo.setSizeAdjustPolicy(QtWidgets.QComboBox.SizeAdjustPolicy.AdjustToContents)
        return combo

    def _populate_belly_options(self, nose_label: str, preferred_belly: str | None = None) -> None:
        """Restricts the belly dropdown to the 4 axes perpendicular to the
        chosen nose axis, so an invalid (non-orthogonal) pair can never be
        selected in the first place."""
        nose_vec = AXIS_VECTORS[nose_label]
        valid = [lbl for lbl in AXIS_LABELS if abs(np.dot(AXIS_VECTORS[lbl], nose_vec)) < 1e-6]
        self.mesh_belly_combo.blockSignals(True)
        self.mesh_belly_combo.clear()
        self.mesh_belly_combo.addItems(valid)
        if preferred_belly in valid:
            self.mesh_belly_combo.setCurrentText(preferred_belly)
        self.mesh_belly_combo.blockSignals(False)

    def _on_mesh_nose_changed(self, nose_label: str) -> None:
        self._populate_belly_options(nose_label)
        self._on_mesh_axes_changed()

    def _on_mesh_axes_changed(self, *_args) -> None:
        self.orientation_view.set_mesh_axes(
            self.mesh_nose_combo.currentText(), self.mesh_belly_combo.currentText(),
        )

    def _on_reset_mesh_alignment(self) -> None:
        self.mesh_nose_combo.blockSignals(True)
        self.mesh_nose_combo.setCurrentText("+X")
        self.mesh_nose_combo.blockSignals(False)
        self._populate_belly_options(nose_label="+X", preferred_belly="+Z")
        self._on_mesh_axes_changed()

    def _on_load_mesh(self) -> None:
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Load craft mesh", "",
            "3D meshes (*.obj *.stl *.glb *.gltf *.ply);;All files (*)",
        )
        if path:
            self.orientation_view.set_mesh(path)  # resets its axis mapping to identity
            self._on_reset_mesh_alignment()  # keep these controls in sync with that reset
            self._status_callback(f"Loaded mesh: {path} -- set Nose/Belly axis if it looks misaligned")

    # ------------------------------------------------------------------
    def _make_color_swatch(self, color: QtGui.QColor) -> QtWidgets.QPushButton:
        btn = QtWidgets.QPushButton()
        btn.setFixedSize(36, 22)
        self._paint_swatch(btn, color)
        return btn

    def _paint_swatch(self, btn: QtWidgets.QPushButton, color: QtGui.QColor) -> None:
        btn.setStyleSheet(
            f"background-color: rgba({color.red()},{color.green()},{color.blue()},{color.alphaF():.3f});"
            f"border: 1px solid {theme.PANEL_BORDER}; border-radius: 3px;"
        )
        btn.setToolTip(color.name(QtGui.QColor.NameFormat.HexArgb))

    def _on_pick_color(self, which: str) -> None:
        current = self.hw_color if which == "hw" else self.sw_color
        color = QtWidgets.QColorDialog.getColor(
            initial=current,
            parent=self,
            title=f"{'Hardware' if which == 'hw' else 'Software'} mesh color",
            options=(
                QtWidgets.QColorDialog.ColorDialogOption.ShowAlphaChannel
                | QtWidgets.QColorDialog.ColorDialogOption.DontUseNativeDialog
            ),
        )
        if not color.isValid():
            return
        rgba = (color.redF(), color.greenF(), color.blueF(), color.alphaF())
        if which == "hw":
            self.hw_color = color
            self._paint_swatch(self.hw_color_btn, color)
            self.orientation_view.set_hw_color(rgba)
            self._sync_opacity_slider(self.hw_opacity_slider, self.hw_opacity_label, color.alphaF())
        else:
            self.sw_color = color
            self._paint_swatch(self.sw_color_btn, color)
            self.orientation_view.set_sw_color(rgba)
            self._sync_opacity_slider(self.sw_opacity_slider, self.sw_opacity_label, color.alphaF())

    def _make_opacity_slider(self, which: str, initial_alpha: float) -> tuple[QtWidgets.QSlider, QtWidgets.QLabel]:
        slider = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(round(initial_alpha * 100))
        slider.setFixedWidth(90)
        label = QtWidgets.QLabel(f"{round(initial_alpha * 100)}%")
        label.setFixedWidth(34)
        slider.valueChanged.connect(lambda v: self._on_opacity_changed(which, v, label))
        return slider, label

    def _on_opacity_changed(self, which: str, percent: int, label: QtWidgets.QLabel) -> None:
        label.setText(f"{percent}%")
        alpha = percent / 100.0
        color = self.hw_color if which == "hw" else self.sw_color
        color.setAlphaF(alpha)
        rgba = (color.redF(), color.greenF(), color.blueF(), alpha)
        if which == "hw":
            self._paint_swatch(self.hw_color_btn, color)
            self.orientation_view.set_hw_color(rgba)
        else:
            self._paint_swatch(self.sw_color_btn, color)
            self.orientation_view.set_sw_color(rgba)

    def _sync_opacity_slider(self, slider: QtWidgets.QSlider, label: QtWidgets.QLabel, alpha: float) -> None:
        percent = round(alpha * 100)
        slider.blockSignals(True)
        slider.setValue(percent)
        slider.blockSignals(False)
        label.setText(f"{percent}%")
