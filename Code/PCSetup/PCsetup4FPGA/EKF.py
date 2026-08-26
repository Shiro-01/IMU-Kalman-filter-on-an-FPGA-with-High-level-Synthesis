
""""
 * @file    EKF.py

 * @brief   Extended Kalman filter Class For IMU Unit. it takes the intial state vector and its covariance matrix as its input
 *          The filter Design is fully explained in the Paper. 
 *
 *          State vector: x = [phi, theta, psi, bx, by, bz]  (roll, pitch, yaw) in Rad, gyro bias, in rad/s)
 *
 * @author  Abdelrahman Hewala
 * @note    Supervisor: Prof. Lutz Leutelt
 * @date    2026
"""

from dataclasses import dataclass
from typing import Optional, Sequence
import numpy as np

from frame_types import PreparedSample

GIMBAL_LOCK_THRESHOLD_RAD = np.radians(89.0)

@dataclass
class EKFResult:
    """One step's output: the updated state , it Coveriance matrix inovation step."""
    X: np.ndarray            # [phi, theta, psi, bx, by, bz]
    P: np.ndarray            # 6x6 covariance
    used_mag: bool           # whether this update included the magnetometer rows
    innovation: np.ndarray    # y - h(x), 3 or 6 long depending on used_mag

class EKF:
    """
    Extended Kalman Filter for IMU orientation + gyro bias.

    Construction requires the initial state (from DataPreparer.initialize())
    and initial covariance (from Allan variance. this
    class does not derive P0). Noise parameters (Gyro ARW, Gyro bias instability,
    accel/mag measurement noise) are also supplied at construction.
    """

    def __init__(
        self,
        # Initial State
        x0  : np.ndarray,                            # The intial state vector
        p0  : np.ndarray,                            # The intial state vector Covariance Matrix (6x6)

        # Noise
        gyro_ARW : Sequence[float],
        gyro_bias_instability : Sequence[float],     # rad/s per sqrt(s), from Allan variance
        accel_dev : Sequence[float],                 # accel std (sigma) array in  order sigmax, sigmay, sigma z
        mag_dev   : Sequence[float],                 # Mag std (sigma) array in  order sigmax, sigmay, sigma z 
        # Constants
        mag_field_horizontal_uT: float = 18.54,   # mN, Hamburg WMM 2025-2026
        mag_field_vertical_uT: float = 45.88,     # mD, Hamburg WMM 2025-2026
        gravity: float = 9.81,

    ):
        # Initial State
        self.X = np.array(x0, dtype=float).copy()                                  # Must have a copy. without it x hold the address of the passed Xo where ever it is and i'll be writing to it instead writing to X, same applies to evry thing else
        self.P = np.array(p0, dtype=float).copy()
        # Noise
        self.gyro_ARW_squared = np.diag(gyro_ARW)**2                               # arranging the ARW squared in the digonal of 3*3 matrix
        self.gyro_bias_instability_squared = np.diag(gyro_bias_instability)**2     # same as previous for ARW

        self.accel_varriance =  np.diag(accel_dev)**2  
        self.mag_varriance =np.diag(mag_dev)**2    
        # Constants
        self.mN = mag_field_horizontal_uT
        self.mD = mag_field_vertical_uT
        self.g = gravity

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------
    @staticmethod
    def _G(phi: float, theta: float) -> np.ndarray:
        """
        Kinematic mapping G(phi,theta), per the paper's eq. 13.
 
        Near theta = +-90deg (gimbal lock, where 1/cos(theta) blows up),
        theta is clamped to +-GIMBAL_LOCK_THRESHOLD_RAD, preserving its
        sign, and a warning is printed. Returns (G, gimbal_lock_flag) so
        the caller can detect and react to the condition rather than it
        failing silently.
        """
        gimbal_lock = abs(theta) >= GIMBAL_LOCK_THRESHOLD_RAD
        if gimbal_lock:
            print(f"Warning: gimbal lock region (theta={np.degrees(theta):.2f} deg), "
                  f"clamping to {np.degrees(np.sign(theta) * GIMBAL_LOCK_THRESHOLD_RAD):.2f} deg")
            theta_safe = np.sign(theta) * GIMBAL_LOCK_THRESHOLD_RAD
        else:
            theta_safe = theta
 
        tan_theta = np.tan(theta_safe)
        sec_theta = 1.0 / np.cos(theta_safe)
 
        G = np.array([
            [1.0, np.sin(phi) * tan_theta, np.cos(phi) * tan_theta],
            [0.0, np.cos(phi), -np.sin(phi)],
            [0.0, np.sin(phi) * sec_theta, np.cos(phi) * sec_theta],
        ])
        return G, gimbal_lock


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
    # Predict
    # ------------------------------------------------------------------
    def predict(self, sample:PreparedSample) -> None: # no returns
        """Propagates x and P forward by sample.dt using the gyroscope."""
        phi, theta, psi, bx, by, bz = self.X                                     # get the current state
        dt = sample.dt

        # State vector predict
        G, gimbal_lock = self._G(phi, theta)                                                  # step 1: calculate the G matrix
        wc = np.array([(sample.gx - bx), (sample.gy - by), (sample.gz - bz)])    # Step 2: subtract the bias values
        theta_dot = G @ wc                                                       # step 3: calculate the Angular Velocity vector Theat_dot (applying the G transition matrix to the Wc vector to be in Eualer angles domain)
        self.X[0:3] = self.X[0:3] + dt * theta_dot                               # Step 4: Predecting the next state vector

        # updating the Cov Vector P Pk+1 = Fd P FTd  + Q 
        # 1) Creating Fd matrix
        Fc = np.zeros((6, 6))                                                    # Step 5: creating the Jacobiam matrix FC
        Fc[0:3, 0:3] = self._dGwc_dTheta(phi, theta, wc)                         # Step 5.1 filling the Jacobian values
        Fc[0:3, 3:6] = -G                                                        # rest is zero no need to fill
        Fd = np.eye(6) + dt * Fc                                                 # Step 5.2 discrete-time state transition matrix used for the covariance matrix

        # 2) creating the Q matrix
        Q = np.zeros((6, 6))                                                     # Step 6: creating Q matrix
        Q[0:3, 0:3] = dt *  (G @ self.gyro_ARW_squared @ G.T)                    # mulytiplying the  arw  array with the I matrix to get the ARWs in the diagonlas and then applying the linear transformation 
        Q[3:6, 3:6] = dt * self.gyro_bias_instability_squared                    # same for the BI

        # 3) updating the covariance matrix 
        self.P = Fd @ self.P @ Fd.T + Q                                         # step 7: updating the covariance matrix 


    def update(self, sample:PreparedSample) -> EKFResult:
        mag_fresh = sample.mag_fresh
        phi, theta, psi, bx, by, bz = self.X                                        # get the current state

        ha = self._ha(phi, theta)                                                   # step 1 calculate ha vector for invoation calc  (convert the current state into measurement space (acc))
        Ha = self._Ha(phi, theta)                                                   # calc the jacobian of ha

        if mag_fresh :        
            # 1) Prepare the matrixes                                                    # if the mag value is new
            hm = self._hm(phi, theta, psi)                                                    # step 2: Calculate the hm vector for inovation (convert the current state vector into measurement space(mag))
            hx = np.concatenate([ha, hm])                                                           # step 3 build the  hx matrix that hold the stacked repersentation of the current state

            ymeas =np.array([sample.ax, sample.ay, sample.az, sample.mx, sample.my, sample.mz]).T      # step 4: building the measurment vector

            # 2) calc the measurement noise matrix R
            R = np.zeros((6, 6)) 
            R[0:3, 0:3] = self.accel_varriance                                                       # Flling R matrix (measurement noise matrix)
            R[3:6, 3:6] = self.mag_varriance

            Hm = self._Hm(phi, theta, psi)
            H = np.vstack([Ha, Hm])  


        else : # only accel correction
            ymeas =np.array([sample.ax, sample.ay, sample.az]).T  
            hx = ha
            H = Ha
            R = self.accel_varriance

        # 3) calc the inovation 
        inovation_e = ymeas - hx                                                                 # step 5: calc the inovation matrix

        # 4) Inovation Covariance S
        S = H @ self.P @ H.T + R

        # 5) kalman gain calc
        K = self.P @ H.T  @ np.linalg.inv(S)

        # 6) state update
        self.X = self.X + K @ inovation_e

        # 7) Covariance update
        I = np.eye(6)
        IKH = (I - K @ H)
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

        return EKFResult(X=self.X.copy(), P=self.P.copy(), used_mag=mag_fresh, innovation=inovation_e)

    def step(self, sample: PreparedSample) -> EKFResult:
        """Convenience wrapper: predict then update, in one call."""
        self.predict(sample)
        return self.update(sample)