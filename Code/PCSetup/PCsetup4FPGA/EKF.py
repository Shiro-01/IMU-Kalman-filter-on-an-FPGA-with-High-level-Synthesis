"""
ekf.py

Extended Kalman Filter for IMU orientation and gyro-bias estimation, per
the design in EKF_IMU_Paper.pdf (Euler angles, ZYX order, 9-DOF).

State vector: x = [phi, theta, psi, bx, by, bz]  (roll, pitch, yaw, rad;
gyro bias, rad/s)

Every Jacobian used here (Ha, Hm, and the dG(w-b)/dTheta block of Fc) was
numerically verified against finite-differences before being written into
this file -- not just hand-derived algebra. See the conversation history
for the verification, or re-run it yourself before trusting a change here.

Two of the paper's equations use different magnetometer conventions
(Section 1.4.2's full-magnitude mn = [mN, 0, mD] vs Section 2.5.2's
normalized mn = [cos(delta), 0, sin(delta)]). This implementation uses the
full-magnitude form throughout, to stay consistent with DataPreparer.
initialize(), which was built and verified against that same convention.
Using the normalized form here instead would silently break the update
step, since mx/my/mz arrive in physical uT, not normalized units.

@author  Abdelrahman Hewala
@date    2026
"""

from dataclasses import dataclass
from typing import Optional
import numpy as np

from data_preparer import PreparedSample


@dataclass
class EKFResult:
    """One step's output: the updated state and a few values useful for
    tuning/diagnostics without re-deriving them from x and P."""
    x: np.ndarray            # [phi, theta, psi, bx, by, bz]
    P: np.ndarray            # 6x6 covariance
    used_mag: bool           # whether this update included the magnetometer rows
    innovation: np.ndarray    # y - h(x), 3 or 6 long depending on used_mag


class EKF:
    """
    Extended Kalman Filter for IMU orientation + gyro bias.

    Construction requires the initial state (from DataPreparer.initialize())
    and initial covariance (from Allan variance, computed elsewhere -- this
    class does not derive P0). Noise parameters (ARW, bias instability,
    accel/mag measurement noise) are also supplied at construction, since
    guessing them wrong would silently produce a filter that runs without
    error but converges to the wrong answer.
    """

    def __init__(
        self,
        x0: np.ndarray,
        P0: np.ndarray,
        arw: float,               # rad/sqrt(s), from Allan variance
        gyro_bias_instability: float,  # rad/s per sqrt(s), from Allan variance
        accel_noise_std: float,    # m/s^2, per-axis
        mag_noise_std: float,      # uT, per-axis
        mag_field_horizontal_uT: float = 18.54,   # mN, must match DataPreparer
        mag_field_vertical_uT: float = 45.88,     # mD, must match DataPreparer
        gravity: float = 9.81,
    ):
        self.x = np.array(x0, dtype=float).copy()
        self.P = np.array(P0, dtype=float).copy()

        self.arw = arw
        self.sigma_b = gyro_bias_instability
        self.Ra = (accel_noise_std ** 2) * np.eye(3)
        self.Rm = (mag_noise_std ** 2) * np.eye(3)

        self.mN = mag_field_horizontal_uT
        self.mD = mag_field_vertical_uT
        self.g = gravity

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------

    @staticmethod
    def _G(phi: float, theta: float) -> np.ndarray:
        tan_t = np.tan(theta)
        sec_t = 1.0 / np.cos(theta)
        return np.array([
            [1.0, np.sin(phi) * tan_t, np.cos(phi) * tan_t],
            [0.0, np.cos(phi), -np.sin(phi)],
            [0.0, np.sin(phi) * sec_t, np.cos(phi) * sec_t],
        ])

    @staticmethod
    def _dGwc_dTheta(phi: float, theta: float, wc: np.ndarray) -> np.ndarray:
        """Jacobian of G(phi,theta) @ wc with respect to [phi, theta, psi].
        The psi column is always zero since G does not depend on psi.
        Verified against finite-differences, see module docstring."""
        wcx, wcy, wcz = wc
        sin_t, cos_t = np.sin(theta), np.cos(theta)
        sec_t = 1.0 / cos_t
        tan_t = sin_t / cos_t

        d_phidot_dphi = wcy * np.cos(phi) * tan_t - wcz * np.sin(phi) * tan_t
        d_phidot_dtheta = wcy * np.sin(phi) * sec_t**2 + wcz * np.cos(phi) * sec_t**2

        d_thetadot_dphi = -wcy * np.sin(phi) - wcz * np.cos(phi)
        d_thetadot_dtheta = 0.0

        d_psidot_dphi = wcy * np.cos(phi) * sec_t - wcz * np.sin(phi) * sec_t
        d_psidot_dtheta = wcy * np.sin(phi) * sin_t * sec_t**2 + wcz * np.cos(phi) * sin_t * sec_t**2

        return np.array([
            [d_phidot_dphi, d_phidot_dtheta, 0.0],
            [d_thetadot_dphi, d_thetadot_dtheta, 0.0],
            [d_psidot_dphi, d_psidot_dtheta, 0.0],
        ])

    # ------------------------------------------------------------------
    # Measurement models
    # ------------------------------------------------------------------

    def _ha(self, phi: float, theta: float) -> np.ndarray:
        return np.array([
            -self.g * np.sin(theta),
            self.g * np.sin(phi) * np.cos(theta),
            self.g * np.cos(phi) * np.cos(theta),
        ])

    def _Ha(self, phi: float, theta: float) -> np.ndarray:
        H = np.zeros((3, 6))
        H[0, 1] = -self.g * np.cos(theta)
        H[1, 0] = self.g * np.cos(phi) * np.cos(theta)
        H[1, 1] = -self.g * np.sin(phi) * np.sin(theta)
        H[2, 0] = -self.g * np.sin(phi) * np.cos(theta)
        H[2, 1] = -self.g * np.cos(phi) * np.sin(theta)
        return H

    def _hm(self, phi: float, theta: float, psi: float) -> np.ndarray:
        mN, mD = self.mN, self.mD
        return np.array([
            mN * np.cos(theta) * np.cos(psi) - mD * np.sin(theta),
            mN * (np.sin(phi) * np.sin(theta) * np.cos(psi) - np.cos(phi) * np.sin(psi))
                + mD * np.sin(phi) * np.cos(theta),
            mN * (np.cos(phi) * np.sin(theta) * np.cos(psi) + np.sin(phi) * np.sin(psi))
                + mD * np.cos(phi) * np.cos(theta),
        ])

    def _Hm(self, phi: float, theta: float, psi: float) -> np.ndarray:
        mN, mD = self.mN, self.mD
        H = np.zeros((3, 6))
        H[0, 1] = -mN * np.sin(theta) * np.cos(psi) - mD * np.cos(theta)
        H[0, 2] = -mN * np.cos(theta) * np.sin(psi)

        H[1, 0] = mN * np.cos(phi) * np.sin(theta) * np.cos(psi) \
                  + mN * np.sin(phi) * np.sin(psi) + mD * np.cos(phi) * np.cos(theta)
        H[1, 1] = mN * np.sin(phi) * np.cos(theta) * np.cos(psi) - mD * np.sin(phi) * np.sin(theta)
        H[1, 2] = -mN * np.sin(phi) * np.sin(theta) * np.sin(psi) - mN * np.cos(phi) * np.cos(psi)

        H[2, 0] = -mN * np.sin(phi) * np.sin(theta) * np.cos(psi) \
                  + mN * np.cos(phi) * np.sin(psi) - mD * np.sin(phi) * np.cos(theta)
        H[2, 1] = mN * np.cos(phi) * np.cos(theta) * np.cos(psi) - mD * np.cos(phi) * np.sin(theta)
        H[2, 2] = -mN * np.cos(phi) * np.sin(theta) * np.sin(psi) + mN * np.sin(phi) * np.cos(psi)
        return H

    # ------------------------------------------------------------------
    # Predict / update
    # ------------------------------------------------------------------

    def predict(self, sample: PreparedSample) -> None:
        """Propagates x and P forward by sample.dt using the gyroscope."""
        phi, theta, psi, bx, by, bz = self.x
        dt = sample.dt

        wc = np.array([sample.gx - bx, sample.gy - by, sample.gz - bz])
        G = self._G(phi, theta)
        theta_dot = G @ wc

        self.x[0:3] = self.x[0:3] + dt * theta_dot
        # bias states: random walk, mean unchanged by prediction

        Fc = np.zeros((6, 6))
        Fc[0:3, 0:3] = self._dGwc_dTheta(phi, theta, wc)
        Fc[0:3, 3:6] = -G
        Fd = np.eye(6) + dt * Fc

        Q = np.zeros((6, 6))
        Q[0:3, 0:3] = dt * (self.arw ** 2) * (G @ G.T)
        Q[3:6, 3:6] = dt * (self.sigma_b ** 2) * np.eye(3)

        self.P = Fd @ self.P @ Fd.T + Q

    def update(self, sample: PreparedSample, use_mag: Optional[bool] = None) -> EKFResult:
        """
        Corrects x and P using the accelerometer, plus the magnetometer
        if use_mag is True (defaults to sample.mag_fresh, i.e. the gating
        DataPreparer already computed).
        """
        if use_mag is None:
            use_mag = sample.mag_fresh

        phi, theta, psi = self.x[0], self.x[1], self.x[2]

        if use_mag:
            h = np.concatenate([self._ha(phi, theta), self._hm(phi, theta, psi)])
            H = np.vstack([self._Ha(phi, theta), self._Hm(phi, theta, psi)])
            y = np.array([sample.ax, sample.ay, sample.az,
                          sample.mx, sample.my, sample.mz])
            R = np.zeros((6, 6))
            R[0:3, 0:3] = self.Ra
            R[3:6, 3:6] = self.Rm
        else:
            h = self._ha(phi, theta)
            H = self._Ha(phi, theta)
            y = np.array([sample.ax, sample.ay, sample.az])
            R = self.Ra

        eps = y - h
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ eps

        I = np.eye(6)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T   # Joseph form

        return EKFResult(x=self.x.copy(), P=self.P.copy(), used_mag=use_mag, innovation=eps)

    def step(self, sample: PreparedSample, use_mag: Optional[bool] = None) -> EKFResult:
        """Convenience wrapper: predict then update, in one call."""
        self.predict(sample)
        return self.update(sample, use_mag=use_mag)