"""
Analytical Jacobians for the EKF predict and update steps.

State vector: x = [q0, q1, q2, q3, bx, by, bz]

Process Jacobian F (7x7):  df/dx evaluated at current x and corrected omega
Measurement Jacobian H (6x7): dh/dx for [accel; mag] measurements
"""

import numpy as np
from .kinematics import omega_matrix, xi_matrix


def process_jacobian(q: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    """
    7x7 state transition Jacobian F = df/dx.

    Derived from:
      q_new = q + 0.5*dt * Ω(ω)*q   ⟹  ∂q_new/∂q = I + 0.5*dt*Ω(ω)
      q_new depends on ω = ω_meas - b  ⟹  ∂q_new/∂b = -0.5*dt*Ξ(q)
      b_new = b (random walk)          ⟹  ∂b_new/∂b = I
    """
    F = np.eye(7)
    F[0:4, 0:4] = np.eye(4) + 0.5 * dt * omega_matrix(omega)
    F[0:4, 4:7] = -0.5 * dt * xi_matrix(q)
    return F


def _dcm_jacobian_wrt_q(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    3x4 Jacobian of R(q)*v with respect to q.

    Derived analytically from:
      (R*v)_i = f(q0, q1, q2, q3, vx, vy, vz)
    using the DCM formula for scalar-first quaternions.
    """
    q0, q1, q2, q3 = q
    vx, vy, vz = v
    return 2.0 * np.array([
        [ q0*vx + q3*vy - q2*vz,   q1*vx + q2*vy + q3*vz,  -q2*vx + q1*vy - q0*vz,  -q3*vx + q0*vy + q1*vz],
        [-q3*vx + q0*vy + q1*vz,   q2*vx - q1*vy + q0*vz,   q1*vx + q2*vy + q3*vz,  -q0*vx - q3*vy + q2*vz],
        [ q2*vx - q1*vy + q0*vz,   q3*vx - q0*vy - q1*vz,   q0*vx + q3*vy - q2*vz,   q1*vx + q2*vy + q3*vz],
    ])


def measurement_jacobian(q: np.ndarray, g_ref: np.ndarray, b_ref: np.ndarray) -> np.ndarray:
    """
    6x7 measurement Jacobian H = dh/dx.

    Measurement vector: z = [ax, ay, az, mx, my, mz]
    Predicted:          h = [R*g_ref; R*b_ref]

    Gyroscope bias has no direct effect on the measurement model, so H[:, 4:7] = 0.
    """
    H = np.zeros((6, 7))
    H[0:3, 0:4] = _dcm_jacobian_wrt_q(q, g_ref)
    H[3:6, 0:4] = _dcm_jacobian_wrt_q(q, b_ref)
    return H
