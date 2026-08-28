"""
data_preparer.py

Prepares physical-unit IMU samples for the EKF:
  - converts consecutive timestamps into per-step dt
  - applies magnetometer hard/soft-iron correction
  - flags magnetometer freshness (changed reading, or 10ms elapsed)
  - computes the filter's initial state from a stationary averaging window

Hard/soft-iron constants are supplied at construction, not computed here.
Calibration itself is a separate one-off tool, kept deliberately out of
this class.

@author  Abdelrahman Hewala
@note    Supervisor: Prof. Lutz Leutelt
@date    2026
"""



from dataclasses import dataclass
from typing import List, Optional, Sequence
import numpy as np

from frame_types import PreparedSample, PhysicalSampleMode0


INT_MIN_WINDOW = 100

class DataPreparer:
    """
    Converts a stream of PhysicalSampleMode0 into EKF-ready PreparedSample
    objects, and computes the filter's initial state from an averaging
    window of stationary samples (Step 8 of the EKF design).
    """

    def __init__(
        self,
        gyro_ARW : Sequence[float],
        hard_iron_offset: Sequence[float] = (0.0, 0.0, 0.0),
        soft_iron_matrix: Optional[Sequence[Sequence[float]]] = None,
        accel_bias_offset: Optional[Sequence[float]] = (0.0, 0.0, 0.0),           # Fixed offset of the ACC for each axis (bax, bay, baz)
        mag_field_horizontal_uT: float = 18.54,   # mN, Hamburg WMM 2025-2026
        mag_field_vertical_uT: float = 45.88,     # mD, Hamburg WMM 2025-2026
        gravity: float = 9.81,
        sampling_freq: float = 1125.0             # sampling freq in Hz
    ):
        """
        hard_iron_offset     : 3-vector subtracted from raw mag counts (uT),
                                default zero (no correction) until calibrated.
        soft_iron_matrix      : 3x3 matrix applied after the offset subtract,
                                default identity (no correction) until calibrated.
        accel_bias_offset     : The bais of the 3 accl axis as a list in order ax, ay, az. 
                                the bais value is subtracted(-) to the raed value. so the sign should 
                                be considered while inserting the offset values
        mag_field_horizontal_uT, mag_field_vertical_uT :
                                mN, mD from the WMM at the operating location.
                                Defaults are Hamburg, 2025-2026.
        gravity               : standard gravity, m/s^2.
        """
        self.hard_iron = np.array(hard_iron_offset, dtype=float)
        self.soft_iron = (
            np.eye(3) if soft_iron_matrix is None
            else np.array(soft_iron_matrix, dtype=float)
        )
        self.g = gravity
        self.mN = mag_field_horizontal_uT
        self.mD = mag_field_vertical_uT
        self.accel_bias_offset = accel_bias_offset
        self._last_sample_t: float = None
        self.gyro_ARW = np.array(gyro_ARW)
        self.fs = sampling_freq

    def _correct_mag(self, mx: float, my: float, mz: float) -> np.ndarray:
        raw = np.array([mx, my, mz], dtype=float)
        return self.soft_iron @ (raw - self.hard_iron)

    def initialize(self, samples: List[PhysicalSampleMode0]):
        """
        Computes the initial state vector [phi, theta, psi, bx, by, bz]
        from a window of stationary samplesstructed.

        Inverts the paper's own forward equations directly:
          theta0 = atan2(-ax, sqrt(ay^2 + az^2))
          phi0   = atan2(ay, az)
          psi0   = atan2(sin_psi, cos_psi), from the mag model (eq. 9),
                   using phi0/theta0 above.

        Known limitation: cos(theta0) appears in the psi0 denominator, so
        this is not valid right at theta0 = +-90 degrees, the same gimbal
        lock region where the G matrix itself is singular.
        """
        window = samples[: INT_MIN_WINDOW]
        samples_count = len(window)
        if samples_count < INT_MIN_WINDOW:
            raise ValueError(
                f"initialize() needs at least {INT_MIN_WINDOW} samples, "
                f"got {len(window)}"
            )

        ax = float(np.mean([s.ax for s in window]) - self.accel_bias_offset[0])
        ay = float(np.mean([s.ay for s in window]) - self.accel_bias_offset[1])
        az = float(np.mean([s.az for s in window]) - self.accel_bias_offset[2])

        gx = float(np.mean([s.gx for s in window]))
        gy = float(np.mean([s.gy for s in window]))
        gz = float(np.mean([s.gz for s in window]))

        mx = float(np.mean([s.mx for s in window]))
        my = float(np.mean([s.my for s in window]))
        mz = float(np.mean([s.mz for s in window]))
        mb = self._correct_mag(mx, my, mz)

        theta0 = np.arctan2(-ax, np.sqrt(ay**2 + az**2))
        phi0 = np.arctan2(ay, az)

        sin_psi = (np.sin(phi0) * mb[2] - np.cos(phi0) * mb[1]) / self.mN
        cos_psi = (mb[0] + self.mD * np.sin(theta0)) / (self.mN * np.cos(theta0))
        psi0 = np.arctan2(sin_psi, cos_psi)

        x0 = np.array([phi0, theta0, psi0, gx, gy, gz])
        self._last_sample_t = samples[-1].t

        bias_sigma = self.gyro_ARW * np.sqrt(self.fs/samples_count)
        
        p0 = np.diag([np.radians(1),np.radians(1),np.radians(1), bias_sigma[0], bias_sigma[1], bias_sigma[2]])**2
        return x0, p0

    def prepare(self, sample: PhysicalSampleMode0) -> PreparedSample:
        """
        Converts one live sample into a PreparedSample: computes dt,
        applies mag & Accel offset calibration.
        """
        if self._last_sample_t is None:
            raise RuntimeError(" call initialize() before prepare()")

        dt = sample.t - self._last_sample_t
        self._last_sample_t = sample.t


        mb = self._correct_mag(sample.mx, sample.my, sample.mz)

        return PreparedSample(
            dt=dt,
            gx=sample.gx, gy=sample.gy, gz=sample.gz,
            ax=(sample.ax - self.accel_bias_offset[0]), 
            ay=(sample.ay - self.accel_bias_offset[1]),
            az=(sample.az - self.accel_bias_offset[2]),
            mx=float(mb[0]), my=float(mb[1]), mz=float(mb[2]),
            temp=sample.temp,
            mag_fresh=sample.mag_fresh,
        )