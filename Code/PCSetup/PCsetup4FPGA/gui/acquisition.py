"""
acquisition.py

High-rate acquisition + filtering worker, replacing app2.py's console loop.
Runs entirely in its own QThread so it is never gated by the GUI: it reads
packets, converts/prepares/steps the software EKF, and publishes into a
StateBus as fast as packets arrive (up to 1125 Hz), completely decoupled
from the ~60 Hz GUI redraw timer.

Uses SampleConverter, DataPreparer and EKF exactly as app2.py does --
no changes to those modules. `physical_mode1_as_mode0` is copied verbatim
from app2.py rather than calling SampleConverter's own same-named method,
because that method is defined without `self` in sample_converter.py and
is not safely callable on an instance; app2.py works around it the same way.
"""

from __future__ import annotations

import os
import sys
import threading
import time

import numpy as np
import serial
from PySide6 import QtCore

# The stable backend modules (EKF, DataPreparer, SampleConverter, ...) live
# one directory up, in the existing PCsetup4FPGA/temp/ tree -- not modified,
# just imported. Make sure that directory is importable regardless of what
# the current working directory happens to be when gui_app.py is launched.
_BACKEND_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _BACKEND_DIR not in sys.path:
    sys.path.insert(0, _BACKEND_DIR)

import constants
from data_preparer import DataPreparer
from EKF import EKF
from frame_types import PhysicalSampleMode0, PhysicalSampleMode1
from mock_source import MockFPGASource, mock_read_packet
from sample_converter import SampleConverter
from state_bus import StateBus, StateSnapshot
from UARTFPGA_reader import read_packet


def physical_mode1_as_mode0(s: PhysicalSampleMode1) -> PhysicalSampleMode0:
    """Adapter only -- repackages a MODE_1 sample's raw-sensor fields into
    a PhysicalSampleMode0-shaped object so it can be fed into
    DataPreparer.prepare()/initialize(). Copied from app2.py."""
    return PhysicalSampleMode0(
        t=0, ax=s.ax, ay=s.ay, az=s.az,
        gx=s.gx, gy=s.gy, gz=s.gz,
        temp=float("nan"),
        mx=s.mx, my=s.my, mz=s.mz,
        mag_fresh=s.mag_fresh,
    )


def wrap_angle(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def state_delta(hw: np.ndarray, sw: np.ndarray) -> np.ndarray:
    """Per-component hw - sw for the 6 state components
    [phi,theta,psi,bx,by,bz], with wraparound-safe differencing on the 3
    angle components. Angles (rad) and gyro biases (rad/s) are different
    physical units, so this is deliberately returned as a 6-vector rather
    than collapsed into one combined number -- summing squared errors
    across mismatched units has no physical meaning."""
    d = hw - sw
    d[0:3] = wrap_angle(d[0:3])
    return d


class AcquisitionWorker(QtCore.QThread):
    """Runs the full read -> convert -> prepare -> EKF.step() pipeline in
    its own thread. Publishes into `state_bus` after every packet.

    source_mode: "mock" or "serial".
    """

    status_changed = QtCore.Signal(str)
    error_occurred = QtCore.Signal(str)

    def __init__(
        self,
        state_bus: StateBus,
        source_mode: str = "mock",
        serial_port: str = constants.DEFAULT_PORT,
        baudrate: int = constants.DEFAULT_BAUDRATE,
        parent=None,
    ):
        super().__init__(parent)
        self.state_bus = state_bus
        self.source_mode = source_mode
        self.serial_port = serial_port
        self.baudrate = baudrate
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        try:
            self._run()
        except Exception as exc:  # surface to the GUI instead of dying silently
            self.error_occurred.emit(f"{type(exc).__name__}: {exc}")

    def _run(self) -> None:
        ser, read_fn, hard_iron, soft_iron, accel_bias = self._open_source()
        try:
            sample_converter = SampleConverter(
                constants.ACC_LSB_PER_1_G,
                constants.GYRO_LSB_PER_DPS,
                constants.TIMESTAMP_CLK_FREQ,
                constants.MAG_LSB,
                constants.TEMP_LSB_PER_C,
                constants.TEMP_OFFSET,
            )
            data_preparer = DataPreparer(
                gyro_ARW=constants.GYRO_ARW,
                hard_iron_offset=hard_iron,
                soft_iron_matrix=soft_iron,
                accel_bias_offset=accel_bias,
                mag_field_horizontal_uT=constants.MAG_FIELD_HORIZONTAL_UT,
                mag_field_vertical_uT=constants.MAG_FIELD_VERTICAL_UT,
            )

            self.status_changed.emit("Initializing filter (collecting stationary window)...")
            init_samples: list[PhysicalSampleMode0] = []
            while len(init_samples) < constants.INIT_SAMPLE_COUNT and not self._stop_event.is_set():
                result = read_fn(ser)
                if result is None:
                    continue
                _, raw = result
                phys1: PhysicalSampleMode1 = sample_converter.convert(raw)
                init_samples.append(physical_mode1_as_mode0(phys1))

            if self._stop_event.is_set():
                return

            x0, p0 = data_preparer.initialize(init_samples)
            ekf = EKF(
                x0=x0, p0=p0,
                gyro_ARW=constants.GYRO_ARW,
                gyro_bias_instability=constants.GYRO_BI,
                accel_dev=constants.ACCEL_DEV,
                mag_dev=constants.MAG_DEV,
                mag_field_horizontal_uT=constants.MAG_FIELD_HORIZONTAL_UT,
                mag_field_vertical_uT=constants.MAG_FIELD_VERTICAL_UT,
            )

            self.status_changed.emit(f"Running ({self.source_mode})")
            seq = 0
            t_start = time.monotonic()
            while not self._stop_event.is_set():
                result = read_fn(ser)
                if result is None:
                    continue
                _, raw = result
                phys1 = sample_converter.convert(raw)

                prepared = data_preparer.prepare(physical_mode1_as_mode0(phys1))
                prepared.dt = phys1.dt  # use the real hardware-computed dt

                ekf_result = ekf.step(prepared)

                sw = np.asarray(ekf_result.X, dtype=float)
                hw = np.array([phys1.phi, phys1.theta, phys1.psi, phys1.bx, phys1.by, phys1.bz])
                delta = state_delta(hw.copy(), sw.copy())

                seq += 1
                self.state_bus.publish(StateSnapshot(
                    seq=seq,
                    t=time.monotonic() - t_start,
                    hw=hw, sw=sw,
                    delta=delta,
                    used_mag=ekf_result.used_mag,
                ))
        finally:
            close = getattr(ser, "close", None)
            if close is not None:
                close()

    def _open_source(self):
        if self.source_mode == "serial":
            ser = serial.Serial(
                port=self.serial_port, baudrate=self.baudrate,
                bytesize=constants.BYTESIZE, parity=constants.PARITY,
                stopbits=constants.STOPBITS, timeout=1,
            )
            return (
                ser, read_packet,
                constants.HARD_IRON_OFFSET, constants.SOFT_IRON_MATRIX,
                constants.ACCEL_BIAS_OFFSET,
            )
        elif self.source_mode == "mock":
            ser = MockFPGASource(
                rate_hz=constants.MOCK_SAMPLE_RATE_HZ,
                accel_sens=constants.ACC_LSB_PER_1_G,
                gyro_sens=constants.GYRO_LSB_PER_DPS,
                mag_sens=constants.MAG_LSB,
                mag_field_horizontal_uT=constants.MAG_FIELD_HORIZONTAL_UT,
                mag_field_vertical_uT=constants.MAG_FIELD_VERTICAL_UT,
            )
            return (
                ser, mock_read_packet,
                constants.MOCK_HARD_IRON_OFFSET, constants.MOCK_SOFT_IRON_MATRIX,
                constants.MOCK_ACCEL_BIAS_OFFSET,
            )
        else:
            raise ValueError(f"unknown source_mode {self.source_mode!r}")
