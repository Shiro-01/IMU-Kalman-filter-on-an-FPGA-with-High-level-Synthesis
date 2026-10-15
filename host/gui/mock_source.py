"""
mock_source.py

Simulated FPGA data source, for developing/demoing the GUI without the
physical UART link. Generates synthetic MODE_1 packets: a smoothly moving
true attitude, encoded into raw IMU register counts consistent with
sample_converter.SampleConverter's sign/scale conventions, plus a
simulated "hardware EKF" state that tracks the same truth with a small,
plausible offset -- so the HW/SW divergence plot has something real to show.

`mock_read_packet(source)` mirrors uart_reader.read_packet(ser)'s
signature and return value ((modeWord, RawSampleMode1) or None), so
acquisition.py can use either interchangeably.

Attitude kinematics reuse EKF's own static G(phi,theta)/ha/hm helpers
(ekf.py is not modified) so the simulated raw sensor readings are exactly
consistent with what the filter's measurement/process model expects --
the software EKF should track this mock truth closely.
"""

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass

import numpy as np

_PIPELINE_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "imu_pipeline"
)
if _PIPELINE_DIR not in sys.path:
    sys.path.insert(0, _PIPELINE_DIR)

from ekf import EKF
from frame_types import RawSampleMode1

MODE1_WORD = 0x012E


@dataclass
class _AttitudeProfile:
    """Smooth multi-sine truth trajectory for roll/pitch/yaw, plus analytic
    derivatives (so gyro rates can be computed exactly, not by finite
    differencing noisy samples).

    Motion amplitude is gated by a smoothstep envelope that stays at zero
    until `hold_s`, then ramps to 1 over `ramp_s`: DataPreparer.initialize()
    needs a genuinely stationary window to compute a valid initial gyro-bias
    estimate (it just averages raw gx/gy/gz over the window), so the mock
    must actually hold still through the acquisition worker's init phase
    rather than starting to move at t=0.
    """

    amp: tuple  # (phi_amp, theta_amp, psi_amp) rad
    freq: tuple  # Hz
    phase: tuple  # rad
    offset: tuple  # rad (psi gets a nonzero mean heading)
    hold_s: float = 1.5
    ramp_s: float = 1.5

    def _envelope(self, t: float) -> tuple:
        """Returns (env, denv/dt) of a cubic smoothstep gate."""
        if t <= self.hold_s:
            return 0.0, 0.0
        if t >= self.hold_s + self.ramp_s:
            return 1.0, 0.0
        u = (t - self.hold_s) / self.ramp_s
        env = 3 * u ** 2 - 2 * u ** 3
        denv = (6 * u - 6 * u ** 2) / self.ramp_s
        return env, denv

    def angles(self, t: float) -> np.ndarray:
        env, _ = self._envelope(t)
        w = [2 * np.pi * f for f in self.freq]
        return np.array([
            self.offset[i] + env * self.amp[i] * np.sin(w[i] * t + self.phase[i])
            for i in range(3)
        ])

    def rates(self, t: float) -> np.ndarray:
        env, denv = self._envelope(t)
        w = [2 * np.pi * f for f in self.freq]
        return np.array([
            self.amp[i] * (
                denv * np.sin(w[i] * t + self.phase[i])
                + env * w[i] * np.cos(w[i] * t + self.phase[i])
            )
            for i in range(3)
        ])


class MockFPGASource:
    """Drop-in stand-in for a `serial.Serial` connected to the FPGA."""

    def __init__(
        self,
        rate_hz: float,
        accel_sens: float,
        gyro_sens: float,
        mag_sens: float,
        mag_field_horizontal_uT: float,
        mag_field_vertical_uT: float,
        gravity: float = 9.81,
        seed: int = 0,
    ):
        self.dt = 1.0 / rate_hz
        self.accel_sens = accel_sens
        self.gyro_sens = gyro_sens
        self.mag_sens = mag_sens
        self.mN = mag_field_horizontal_uT
        self.mD = mag_field_vertical_uT
        self.g = gravity

        self._rng = np.random.default_rng(seed)
        self._t = 0.0
        self._next_deadline = time.perf_counter()

        self._profile = _AttitudeProfile(
            amp=(np.radians(15.0), np.radians(10.0), np.radians(60.0)),
            freq=(0.05, 0.037, 0.013),
            phase=(0.0, 1.0, 0.0),
            offset=(0.0, 0.0, np.radians(30.0)),
        )
        self._true_gyro_bias = np.array([0.0021, -0.0013, 0.0009])  # rad/s

        # Fixed, small "FPGA fixed-point vs. our modeled truth" offsets, so
        # the mock hardware state differs from software slightly and by a
        # plausible, bounded amount rather than being identical.
        self._hw_angle_offset_sigma = np.radians(0.15)
        self._hw_bias_offset = np.array([0.00025, -0.00040, 0.00015])

    def close(self) -> None:
        pass

    def read(self) -> RawSampleMode1:
        # Pace to rate_hz using an absolute deadline so small scheduling
        # jitter doesn't accumulate into drift.
        now = time.perf_counter()
        self._next_deadline += self.dt
        sleep_time = self._next_deadline - now
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            self._next_deadline = now  # fell behind -- resync, don't burst

        t = self._t
        self._t += self.dt

        phi, theta, psi = self._profile.angles(t)
        euler_rate = self._profile.rates(t)

        # Invert the EKF's own kinematic model to get body rates: euler_rate
        # = G(phi,theta) @ (gyro - bias)  =>  gyro = G^-1 @ euler_rate + bias.
        G, _ = EKF._G(phi, theta)
        wc = np.linalg.solve(G, euler_rate)
        gyro_true = wc + self._true_gyro_bias

        # EKF._ha/_hm are instance methods (they read self.g/self.mN/self.mD),
        # so their equations are inlined here rather than calling them unbound.
        accel_true = np.array([
            -self.g * np.sin(theta),
            self.g * np.sin(phi) * np.cos(theta),
            self.g * np.cos(phi) * np.cos(theta),
        ])
        mag_true = np.array([
            self.mN * np.cos(theta) * np.cos(psi) - self.mD * np.sin(theta),
            self.mN * (np.sin(phi) * np.sin(theta) * np.cos(psi) - np.cos(phi) * np.sin(psi))
            + self.mD * np.sin(phi) * np.cos(theta),
            self.mN * (np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi))
            + self.mD * np.cos(phi) * np.cos(theta),
        ])

        # small sensor noise, in LSB-ish terms, added before quantizing to counts
        gyro_noisy = gyro_true + self._rng.normal(0, np.radians(0.02), 3)
        accel_noisy = accel_true + self._rng.normal(0, 0.01, 3)
        mag_noisy = mag_true + self._rng.normal(0, 0.05, 3)

        raw_ax = _to_i16(-accel_noisy[0] * self.accel_sens / self.g)
        raw_ay = _to_i16(accel_noisy[1] * self.accel_sens / self.g)
        raw_az = _to_i16(accel_noisy[2] * self.accel_sens / self.g)

        raw_gx = _to_i16(np.degrees(gyro_noisy[0]) * self.gyro_sens)
        raw_gy = _to_i16(-np.degrees(gyro_noisy[1]) * self.gyro_sens)
        raw_gz = _to_i16(-np.degrees(gyro_noisy[2]) * self.gyro_sens)

        raw_mx = _to_i16(mag_noisy[0] / self.mag_sens)
        raw_my = _to_i16(mag_noisy[1] / self.mag_sens)
        raw_mz = _to_i16(mag_noisy[2] / self.mag_sens)

        # Simulated "FPGA's own EKF output" -- tracks truth with a small
        # bounded offset + noise, so it's neither identical to nor wildly
        # different from the software EKF's result.
        hw_angles = np.array([phi, theta, psi]) + self._rng.normal(
            0, self._hw_angle_offset_sigma, 3
        )
        hw_bias = self._true_gyro_bias + self._hw_bias_offset + self._rng.normal(0, 1e-5, 3)

        return RawSampleMode1(
            ax=raw_ax, ay=raw_ay, az=raw_az,
            gx=raw_gx, gy=raw_gy, gz=raw_gz,
            mx=raw_mx, my=raw_my, mz=raw_mz,
            dt=self.dt,
            phi=float(hw_angles[0]), theta=float(hw_angles[1]), psi=float(hw_angles[2]),
            bx=float(hw_bias[0]), by=float(hw_bias[1]), bz=float(hw_bias[2]),
        )


def _to_i16(x: float) -> int:
    return int(np.clip(round(x), -32768, 32767))


def mock_read_packet(source: MockFPGASource):
    """Mirrors uart_reader.read_packet(ser)'s signature: returns
    (modeWord, raw_sample) or None on failure. The mock never fails."""
    return MODE1_WORD, source.read()
