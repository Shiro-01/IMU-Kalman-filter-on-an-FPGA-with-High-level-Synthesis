""""
 * @file    gui_app.py
 * @brief   Real-time IMU/EKF state visualization dashboard. Replaces
 *          app_mode1.py's console printout with a live PySide6 desktop GUI:
 *          a 3D orientation view, hardware-vs-software state readouts,
 *          and rolling per-component HW/SW state delta plots.
 *
 *          Architecture: a QThread (AcquisitionWorker, acquisition.py) runs
 *          the serial-read -> convert -> prepare -> EKF.step() pipeline at
 *          the full incoming packet rate (up to 1125 Hz), publishing into a
 *          StateBus (state_bus.py). This window's QTimer redraws at a fixed
 *          60 Hz, each tick reading only the latest published state -- the
 *          GUI never gates the filter loop, and a slow frame never stalls it.
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
"""

from __future__ import annotations

import sys

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets

import constants
import theme
from acquisition import AcquisitionWorker
from mesh_settings_dialog import MeshSettingsDialog
from orientation_widget import OrientationView
from state_bus import StateBus

GUI_REFRESH_HZ = 60
GUI_REFRESH_MS = int(1000 / GUI_REFRESH_HZ)

# Number of trailing samples averaged into each rolling RMSE value. A real
# RMSE is an average over *samples*, not over state components -- this is
# what actually makes it a windowed RMSE rather than the flawed single
# -instant "combined norm" this dashboard used before.
RMSE_WINDOW_SAMPLES = 50000


def _rolling_rmse(sq_sum: np.ndarray, window: int) -> np.ndarray:
    """Rolling RMSE from a per-sample sum-of-squared-errors array:
    sqrt(mean(sq_sum) over the trailing `window` samples). Uses an
    expanding window for the first `window` samples so the curve has no
    gap at the start of a run, then a fixed trailing window after that.
    O(N) via a cumulative sum, cheap even for a large buffered history."""
    n = sq_sum.shape[0]
    csum = np.cumsum(sq_sum)
    counts = np.arange(1, n + 1, dtype=float)
    rolling_sum = csum.copy()
    if n > window:
        rolling_sum[window:] = csum[window:] - csum[:-window]
        counts[window:] = window
    return np.sqrt(rolling_sum / counts)


# Equation text shown in each "?" hover tooltip. Kept as plain module-level
# strings (simple HTML for Qt's rich-text tooltip rendering) so the exact
# formula used for each on-screen number/curve is always one hover away.
ANGLE_RMSE_EQ = (
    "<b>Angle RMSE (rolling)</b><br>"
    "RMSE<sub>angle</sub>(t) = &radic;( (1/N) &Sigma;<sub>i=1..N</sub> "
    "(&Delta;&phi;<sub>i</sub>&sup2; + &Delta;&theta;<sub>i</sub>&sup2; + "
    "&Delta;&psi;<sub>i</sub>&sup2;) )<br>"
    f"N = last {RMSE_WINDOW_SAMPLES} samples (rolling window). Roll/pitch/yaw "
    "deltas, wrapped to (&minus;180&deg;, 180&deg;] before differencing, "
    "in degrees."
)
BIAS_RMSE_EQ = (
    "<b>Bias RMSE (rolling)</b><br>"
    "RMSE<sub>bias</sub>(t) = &radic;( (1/N) &Sigma;<sub>i=1..N</sub> "
    "(&Delta;b<sub>x,i</sub>&sup2; + &Delta;b<sub>y,i</sub>&sup2; + "
    "&Delta;b<sub>z,i</sub>&sup2;) )<br>"
    f"N = last {RMSE_WINDOW_SAMPLES} samples (rolling window). Same form as "
    "the angle RMSE, with the three gyro-bias deltas (deg/s) in place of "
    "the three angle deltas."
)
WHOLE_STATE_RMSE_EQ = (
    "<b>Whole-state RMSE (rolling)</b><br>"
    "RMSE<sub>state</sub>(t) = &radic;( (1/N) &Sigma;<sub>i=1..N</sub> "
    "(&Delta;&phi;<sub>i</sub>&sup2; + &Delta;&theta;<sub>i</sub>&sup2; + "
    "&Delta;&psi;<sub>i</sub>&sup2; + &Delta;b<sub>x,i</sub>&sup2; + "
    "&Delta;b<sub>y,i</sub>&sup2; + &Delta;b<sub>z,i</sub>&sup2;) )<br>"
    f"N = last {RMSE_WINDOW_SAMPLES} samples (rolling window), all 6 state "
    "components summed under one square root.<br>"
    "<i>Caveat: this sums squared angle error (deg) and squared bias error "
    "(deg/s) together -- not a single physical unit. A convenience combined "
    "indicator, not a calibrated error metric; the two RMSE lines above are "
    "the unit-correct versions.</i>"
)


class StatePanel(QtWidgets.QGroupBox):
    """One HW or SW readout box: roll/pitch/yaw (deg) + gyro bias (deg/s).

    Deg/s bias display matches app_mode1.py's own console convention (the whole
    state vector, angles and biases alike, printed after a single
    `* 180/pi`) -- kept here for continuity with the validated reference
    driver rather than switching to rad/s.
    """

    FIELDS = ["Roll (φ)", "Pitch (θ)", "Yaw (ψ)", "Bias x", "Bias y", "Bias z"]
    UNITS = ["deg", "deg", "deg", "deg/s", "deg/s", "deg/s"]

    def __init__(self, title: str, accent: str, parent=None):
        super().__init__(title, parent)
        self.setStyleSheet(f"QGroupBox {{ color: {accent}; }}")
        grid = QtWidgets.QGridLayout(self)
        self._value_labels: list[QtWidgets.QLabel] = []
        for row, (name, unit) in enumerate(zip(self.FIELDS, self.UNITS)):
            name_lbl = QtWidgets.QLabel(name)
            name_lbl.setProperty("role", "hint")
            val_lbl = QtWidgets.QLabel("--")
            val_lbl.setProperty("role", "value")
            unit_lbl = QtWidgets.QLabel(unit)
            unit_lbl.setProperty("role", "hint")
            grid.addWidget(name_lbl, row, 0)
            grid.addWidget(val_lbl, row, 1)
            grid.addWidget(unit_lbl, row, 2)
            self._value_labels.append(val_lbl)

    def update_state(self, state_rad: np.ndarray) -> None:
        deg = state_rad * 180.0 / np.pi
        for lbl, v in zip(self._value_labels, deg):
            lbl.setText(f"{v:+.3f}")


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU / EKF Real-Time State Dashboard")
        self.resize(1400, 860)

        self.state_bus = StateBus(history_window_s=30.0)
        self.worker: AcquisitionWorker | None = None
        self.mesh_settings_dialog: MeshSettingsDialog | None = None

        self._build_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(GUI_REFRESH_MS)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start()

    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)

        self.orientation_view = OrientationView()
        self.orientation_view.setMinimumWidth(360)
        root.addWidget(self._build_control_bar())

        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        root.addWidget(splitter, 1)
        splitter.addWidget(self.orientation_view)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        self.hw_panel = StatePanel("Hardware EKF (FPGA)", theme.ACCENT)
        self.sw_panel = StatePanel("Software EKF (host)", theme.ACCENT_WARN)
        right_layout.addWidget(self.hw_panel)
        right_layout.addWidget(self.sw_panel)
        right_layout.addWidget(self._build_divergence_plot(), 1)
        splitter.addWidget(right)

        # setStretchFactor alone only governs how *extra* space is handed
        # out on resize, not the initial split -- the GLViewWidget's small
        # sizeHint next to the panels' larger one otherwise collapses the
        # 3D view to a sliver on first show. Force a concrete initial split.
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([900, 500])

        self.status = self.statusBar()
        self.status.showMessage("Idle")

    def _build_control_bar(self) -> QtWidgets.QWidget:
        bar = QtWidgets.QWidget()
        outer = QtWidgets.QVBoxLayout(bar)
        outer.setContentsMargins(0, 0, 0, 0)

        layout = QtWidgets.QHBoxLayout()
        outer.addLayout(layout)

        layout.addWidget(QtWidgets.QLabel("Source:"))
        self.source_combo = QtWidgets.QComboBox()
        self.source_combo.addItems(["Mock", "Serial"])
        self.source_combo.currentTextChanged.connect(self._on_source_mode_changed)
        layout.addWidget(self.source_combo)

        self.port_edit = QtWidgets.QLineEdit(constants.DEFAULT_PORT)
        self.port_edit.setMinimumWidth(220)
        layout.addWidget(self.port_edit)
        self.baud_edit = QtWidgets.QLineEdit(str(constants.DEFAULT_BAUDRATE))
        self.baud_edit.setMaximumWidth(90)
        layout.addWidget(self.baud_edit)

        self.start_btn = QtWidgets.QPushButton("Start")
        self.start_btn.clicked.connect(self._on_start)
        layout.addWidget(self.start_btn)
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.stop_btn.clicked.connect(self._on_stop)
        self.stop_btn.setEnabled(False)
        layout.addWidget(self.stop_btn)

        layout.addSpacing(20)
        layout.addWidget(QtWidgets.QLabel("3D view:"))
        self.display_mode_combo = QtWidgets.QComboBox()
        self.display_mode_combo.addItems(["Both", "Hardware", "Software"])
        self.display_mode_combo.currentTextChanged.connect(self.orientation_view.set_display_mode)
        layout.addWidget(self.display_mode_combo)

        layout.addWidget(QtWidgets.QLabel("Declination (E):"))
        self.declination_spin = QtWidgets.QDoubleSpinBox()
        self.declination_spin.setRange(-30.0, 30.0)
        self.declination_spin.setSingleStep(0.1)
        self.declination_spin.setSuffix(" deg")
        self.declination_spin.setValue(constants.DEFAULT_DECLINATION_DEG_EAST)
        self.declination_spin.valueChanged.connect(self.orientation_view.set_declination_deg)
        layout.addWidget(self.declination_spin)

        self.mesh_settings_btn = QtWidgets.QPushButton("Mesh Settings...")
        self.mesh_settings_btn.clicked.connect(self._on_open_mesh_settings)
        layout.addWidget(self.mesh_settings_btn)

        layout.addStretch(1)

        self._on_source_mode_changed(self.source_combo.currentText())
        return bar

    def _on_open_mesh_settings(self) -> None:
        if self.mesh_settings_dialog is None:
            self.mesh_settings_dialog = MeshSettingsDialog(
                self.orientation_view, self.status.showMessage, parent=self,
            )
        self.mesh_settings_dialog.show()
        self.mesh_settings_dialog.raise_()
        self.mesh_settings_dialog.activateWindow()

    def _build_divergence_plot(self) -> QtWidgets.QWidget:
        # Two panels, one per unit group: angle deltas (deg) and gyro-bias
        # deltas (deg/s). Kept as separate per-component traces rather than
        # collapsed into one combined number -- summing squared errors
        # across mismatched units (rad vs rad/s) has no physical meaning,
        # and a single scalar would also hide which component actually
        # disagrees. Each panel additionally carries its own rolling RMSE
        # curve (a real, sample-averaged RMSE, not the single-instant
        # combined norm this dashboard used before), and a separate small
        # readout below shows the whole-6-state RMSE as a live number.
        box = QtWidgets.QGroupBox("HW / SW State Delta and RMSE")
        v = QtWidgets.QVBoxLayout(box)

        header = QtWidgets.QHBoxLayout()
        header.addWidget(QtWidgets.QLabel("Window:"))
        self.window_spin = QtWidgets.QSpinBox()
        self.window_spin.setRange(5, 300)
        self.window_spin.setValue(30)
        self.window_spin.setSuffix(" s")
        self.window_spin.valueChanged.connect(self._on_window_changed)
        header.addWidget(self.window_spin)
        header.addStretch(1)
        v.addLayout(header)

        whole_row = QtWidgets.QHBoxLayout()
        whole_row.addWidget(QtWidgets.QLabel("Whole-state RMSE:"))
        self.whole_rmse_label = QtWidgets.QLabel("--")
        self.whole_rmse_label.setProperty("role", "value")
        whole_row.addWidget(self.whole_rmse_label)
        whole_row.addWidget(self._make_help_icon(WHOLE_STATE_RMSE_EQ))
        whole_row.addStretch(1)
        v.addLayout(whole_row)

        pg.setConfigOption("background", theme.PANEL_BG)
        pg.setConfigOption("foreground", theme.TEXT)

        angle_title = QtWidgets.QHBoxLayout()
        angle_title.addWidget(QtWidgets.QLabel("Angle error + RMSE (deg)"))
        angle_title.addWidget(self._make_help_icon(ANGLE_RMSE_EQ))
        angle_title.addStretch(1)
        v.addLayout(angle_title)
        self.angle_plot_widget, self.angle_curves, self.angle_rmse_curve = self._make_error_plot(
            "angle error (deg)", ["Δφ (roll)", "Δθ (pitch)", "Δψ (yaw)"],
        )
        v.addWidget(self.angle_plot_widget)

        bias_title = QtWidgets.QHBoxLayout()
        bias_title.addWidget(QtWidgets.QLabel("Bias error + RMSE (deg/s)"))
        bias_title.addWidget(self._make_help_icon(BIAS_RMSE_EQ))
        bias_title.addStretch(1)
        v.addLayout(bias_title)
        self.bias_plot_widget, self.bias_curves, self.bias_rmse_curve = self._make_error_plot(
            "bias error (deg/s)", ["Δbx", "Δby", "Δbz"],
        )
        v.addWidget(self.bias_plot_widget)
        return box

    def _make_error_plot(
        self, y_label: str, series_names: list[str],
    ) -> tuple[pg.PlotWidget, list, object]:
        colors = [theme.ACCENT, theme.ACCENT_WARN, theme.ACCENT_BAD]
        widget = pg.PlotWidget()
        widget.showGrid(x=True, y=True, alpha=0.25)
        widget.setLabel("bottom", "t", units="s")
        widget.setLabel("left", y_label)
        widget.addLegend(offset=(10, 10))
        curves = []
        for name, color in zip(series_names, colors):
            curve = widget.plot(pen=pg.mkPen(color, width=2), name=name)
            # Bounded rolling window handled by StateBus (time-based pruning
            # at acquisition rate); pyqtgraph's own clip/downsample keeps
            # redraws of the (still potentially tens-of-thousands-of-points)
            # window cheap at 60 Hz without discarding buffered data.
            curve.setDownsampling(auto=True, method="peak")
            curve.setClipToView(True)
            curves.append(curve)

        rmse_curve = widget.plot(
            pen=pg.mkPen(theme.ACCENT_RMSE, width=2, style=QtCore.Qt.PenStyle.DashLine),
            name="RMSE",
        )
        rmse_curve.setDownsampling(auto=True, method="peak")
        rmse_curve.setClipToView(True)
        return widget, curves, rmse_curve

    def _make_help_icon(self, tooltip_html: str) -> QtWidgets.QLabel:
        icon = QtWidgets.QLabel("?")
        icon.setFixedSize(16, 16)
        icon.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        icon.setStyleSheet(
            f"background-color: {theme.PANEL_BORDER}; color: {theme.TEXT}; "
            "border-radius: 8px; font-weight: bold; font-size: 10px;"
        )
        icon.setToolTip(tooltip_html)
        icon.setCursor(QtCore.Qt.CursorShape.WhatsThisCursor)
        return icon

    # ------------------------------------------------------------------
    def _on_source_mode_changed(self, mode: str) -> None:
        is_serial = mode == "Serial"
        self.port_edit.setEnabled(is_serial)
        self.baud_edit.setEnabled(is_serial)

    def _on_start(self) -> None:
        if self.worker is not None:
            return
        self.state_bus.clear()
        mode = "serial" if self.source_combo.currentText() == "Serial" else "mock"
        try:
            baud = int(self.baud_edit.text())
        except ValueError:
            baud = constants.DEFAULT_BAUDRATE

        self.worker = AcquisitionWorker(
            state_bus=self.state_bus,
            source_mode=mode,
            serial_port=self.port_edit.text(),
            baudrate=baud,
        )
        self.worker.status_changed.connect(self.status.showMessage)
        self.worker.error_occurred.connect(self._on_worker_error)
        self.worker.finished.connect(self._on_worker_finished)
        self.worker.start()

        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.source_combo.setEnabled(False)

    def _on_stop(self) -> None:
        if self.worker is None:
            return
        self.worker.stop()
        self.worker.wait(3000)
        self._reset_run_controls()

    def _on_worker_error(self, message: str) -> None:
        self.status.showMessage(f"Error: {message}")
        self._reset_run_controls()

    def _on_worker_finished(self) -> None:
        self._reset_run_controls()

    def _reset_run_controls(self) -> None:
        self.worker = None
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.source_combo.setEnabled(True)

    def _on_window_changed(self, seconds: int) -> None:
        self.state_bus.history_window_s = float(seconds)

    # ------------------------------------------------------------------
    def _on_tick(self) -> None:
        latest, t_hist, delta_hist = self.state_bus.snapshot()
        if latest is None:
            return

        self.hw_panel.update_state(latest.hw)
        self.sw_panel.update_state(latest.sw)
        self.orientation_view.set_attitudes(
            hw_rpy=tuple(latest.hw[:3]), sw_rpy=tuple(latest.sw[:3]),
        )
        if t_hist.size:
            delta_deg = delta_hist * (180.0 / np.pi)  # matches StatePanel's deg/(deg/s) convention
            for i, curve in enumerate(self.angle_curves):
                curve.setData(t_hist, delta_deg[:, i])
            for i, curve in enumerate(self.bias_curves):
                curve.setData(t_hist, delta_deg[:, 3 + i])

            angle_sq_sum = np.sum(delta_deg[:, 0:3] ** 2, axis=1)
            bias_sq_sum = np.sum(delta_deg[:, 3:6] ** 2, axis=1)
            whole_sq_sum = angle_sq_sum + bias_sq_sum

            angle_rmse = _rolling_rmse(angle_sq_sum, RMSE_WINDOW_SAMPLES)
            bias_rmse = _rolling_rmse(bias_sq_sum, RMSE_WINDOW_SAMPLES)
            whole_rmse = _rolling_rmse(whole_sq_sum, RMSE_WINDOW_SAMPLES)

            self.angle_rmse_curve.setData(t_hist, angle_rmse)
            self.bias_rmse_curve.setData(t_hist, bias_rmse)
            self.whole_rmse_label.setText(f"{whole_rmse[-1]:.4f}")

    def closeEvent(self, event) -> None:
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(3000)
        super().closeEvent(event)


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(theme.STYLESHEET)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
