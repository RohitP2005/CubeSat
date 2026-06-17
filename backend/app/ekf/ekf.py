"""
Extended Kalman Filter for spacecraft attitude estimation.

State:   x = [q0, q1, q2, q3, bx, by, bz]
           q  — unit quaternion (scalar-first, inertial-to-body)
           b  — gyroscope bias [rad/s]

Inputs:
  predict:  gyroscope angular velocity [rad/s]
  update:   accelerometer [m/s²] + magnetometer [µT or any consistent unit]

Reference vectors (inertial frame, normalized):
  g_ref  — gravity direction (default: [0, 0, 1], down in NED)
  b_ref  — Earth magnetic field (default: representative LEO vector)
"""

import numpy as np
from .quaternion import normalize, to_dcm, to_euler
from .kinematics import propagate
from .jacobians import process_jacobian, measurement_jacobian

# Default reference vectors (normalized)
_G_REF = np.array([0.0, 0.0, 1.0])
_B_REF = np.array([0.3090, 0.0, 0.9511])  # ~72° inclination from horizontal


class AttitudeEKF:
    """
    EKF-based attitude estimator for a CubeSat.

    Implements the standard five-step EKF cycle:
      1. Predict state (gyro integration)
      2. Predict covariance (process noise propagation)
      3. Compute Kalman gain
      4. Update state (accel + mag correction)
      5. Update covariance (Joseph form for numerical stability)
    """

    def __init__(
        self,
        q0: np.ndarray | None = None,
        sigma_gyro: float = 0.01,
        sigma_gyro_bias: float = 1e-4,
        sigma_accel: float = 0.1,
        sigma_mag: float = 0.05,
        g_ref: np.ndarray = _G_REF,
        b_ref: np.ndarray = _B_REF,
    ):
        q_init = normalize(q0) if q0 is not None else np.array([1.0, 0.0, 0.0, 0.0])
        self._x = np.zeros(7)
        self._x[0:4] = q_init

        # Initial covariance — moderate uncertainty in attitude, small in bias
        self._P = np.diag([0.1, 0.1, 0.1, 0.1, 1e-4, 1e-4, 1e-4])

        # Process noise variance per unit time
        self._sq = sigma_gyro ** 2
        self._sb = sigma_gyro_bias ** 2

        # Measurement noise covariance (6x6)
        sa2, sm2 = sigma_accel ** 2, sigma_mag ** 2
        self._R = np.diag([sa2, sa2, sa2, sm2, sm2, sm2])

        self._g_ref = g_ref / np.linalg.norm(g_ref)
        self._b_ref = b_ref / np.linalg.norm(b_ref)

    # ------------------------------------------------------------------
    # Read-only properties
    # ------------------------------------------------------------------

    @property
    def quaternion(self) -> np.ndarray:
        return self._x[0:4].copy()

    @property
    def bias(self) -> np.ndarray:
        return self._x[4:7].copy()

    @property
    def covariance(self) -> np.ndarray:
        return self._P.copy()

    @property
    def covariance_trace(self) -> float:
        return float(np.trace(self._P))

    @property
    def euler_deg(self) -> tuple[float, float, float]:
        r, p, y = to_euler(self._x[0:4])
        return np.degrees(r), np.degrees(p), np.degrees(y)

    # ------------------------------------------------------------------
    # EKF predict step
    # ------------------------------------------------------------------

    def predict(self, omega_meas: np.ndarray, dt: float) -> None:
        """
        Propagate state and covariance using gyroscope measurement.

        omega_meas: raw gyro reading [rad/s], bias will be subtracted internally
        dt:         time step [s]
        """
        q = self._x[0:4]
        b = self._x[4:7]
        omega = omega_meas - b

        self._x[0:4] = propagate(q, omega, dt)
        # Bias is modeled as random walk — state unchanged, noise added to covariance

        # Process noise (diagonal, scaled by dt)
        Q = np.diag([
            self._sq * dt, self._sq * dt, self._sq * dt, self._sq * dt,
            self._sb * dt, self._sb * dt, self._sb * dt,
        ])

        F = process_jacobian(q, omega, dt)
        self._P = F @ self._P @ F.T + Q

    # ------------------------------------------------------------------
    # EKF update step
    # ------------------------------------------------------------------

    def update(self, accel: np.ndarray, mag: np.ndarray) -> None:
        """
        Correct state using accelerometer and magnetometer measurements.
        Both inputs are normalized internally — units do not matter as long
        as they are consistent.
        """
        q = self._x[0:4]
        R_mat = to_dcm(q)

        a_norm_mag = np.linalg.norm(accel)
        m_norm_mag = np.linalg.norm(mag)

        if a_norm_mag < 1e-6 or m_norm_mag < 1e-6:
            return  # degenerate measurement — skip update

        a_meas = accel / a_norm_mag
        m_meas = mag / m_norm_mag

        # Predicted measurements from current quaternion estimate
        h_acc = R_mat @ self._g_ref
        h_mag = R_mat @ self._b_ref

        # Innovation (6-vector)
        z = np.concatenate([a_meas, m_meas])
        h = np.concatenate([h_acc, h_mag])
        innov = z - h

        # Innovation covariance and Kalman gain
        H = measurement_jacobian(q, self._g_ref, self._b_ref)
        S = H @ self._P @ H.T + self._R
        K = self._P @ H.T @ np.linalg.inv(S)

        # State update
        dx = K @ innov
        self._x += dx
        self._x[0:4] = normalize(self._x[0:4])

        # Covariance update — Joseph form for guaranteed positive semi-definiteness
        I_KH = np.eye(7) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ self._R @ K.T
