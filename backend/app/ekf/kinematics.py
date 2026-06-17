"""
Quaternion kinematics for attitude propagation.

Implements q_dot = 0.5 * Ω(ω) * q  ≡  0.5 * Ξ(q) * ω
where ω is the corrected body-frame angular velocity.
"""

import numpy as np
from .quaternion import normalize


def omega_matrix(w: np.ndarray) -> np.ndarray:
    """
    4x4 skew-symmetric Omega matrix such that Ω(ω)*q = Ξ(q)*ω.
    Used for quaternion kinematic propagation.
    """
    wx, wy, wz = w
    return np.array([
        [ 0,  -wx, -wy, -wz],
        [wx,    0,  wz, -wy],
        [wy,  -wz,   0,  wx],
        [wz,   wy, -wx,   0],
    ])


def xi_matrix(q: np.ndarray) -> np.ndarray:
    """
    4x3 Xi matrix such that Ξ(q)*ω = Ω(ω)*q.
    Used in the process Jacobian to capture how bias affects quaternion propagation.
    """
    q0, q1, q2, q3 = q
    return np.array([
        [-q1, -q2, -q3],
        [ q0, -q3,  q2],
        [ q3,  q0, -q1],
        [-q2,  q1,  q0],
    ])


def propagate(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    """
    First-order Euler integration of quaternion kinematics.

    q:     unit quaternion [q0, q1, q2, q3]
    omega: bias-corrected angular velocity in body frame [rad/s]
    dt:    time step [s]
    """
    q_dot = 0.5 * omega_matrix(omega) @ q
    return normalize(q + q_dot * dt)
