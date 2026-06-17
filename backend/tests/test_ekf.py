"""
Unit tests for EKF predict/update steps and Jacobian correctness.
Run: pytest tests/test_ekf.py -v
"""

import numpy as np
import pytest
from app.ekf.quaternion import normalize, to_dcm, from_euler
from app.ekf.kinematics import omega_matrix, xi_matrix
from app.ekf.jacobians import process_jacobian, measurement_jacobian, _dcm_jacobian_wrt_q
from app.ekf.ekf import AttitudeEKF


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _rq(seed=0):
    rng = np.random.default_rng(seed)
    q = rng.normal(size=4)
    return q / np.linalg.norm(q)

def _numerical_jacobian(f, x, h=1e-6):
    """Central-difference numerical Jacobian."""
    y0 = np.asarray(f(x))
    m, n = len(y0), len(x)
    J = np.zeros((m, n))
    for i in range(n):
        xp = x.copy(); xp[i] += h
        xm = x.copy(); xm[i] -= h
        J[:, i] = (np.asarray(f(xp)) - np.asarray(f(xm))) / (2 * h)
    return J


# ---------------------------------------------------------------------------
# omega_matrix / xi_matrix consistency
# ---------------------------------------------------------------------------

def test_omega_xi_equivalence():
    """Ω(ω)*q and Ξ(q)*ω must give the same result."""
    q = _rq(0); w = np.array([0.1, -0.2, 0.05])
    via_omega = omega_matrix(w) @ q
    via_xi    = xi_matrix(q) @ w
    assert np.allclose(via_omega, via_xi, atol=1e-12)

def test_xi_transpose_is_left_inverse():
    """For unit q: Ξ(q)ᵀ Ξ(q) = I₃."""
    q = _rq(1)
    Xi = xi_matrix(q)
    assert np.allclose(Xi.T @ Xi, np.eye(3), atol=1e-12)


# ---------------------------------------------------------------------------
# process_jacobian — numerical verification
# ---------------------------------------------------------------------------

def test_process_jacobian_numerical():
    q0 = _rq(2)
    b0 = np.array([0.001, -0.002, 0.0015])
    x0 = np.concatenate([q0, b0])
    w_meas = np.array([0.05, -0.03, 0.02])
    dt = 0.01

    def f(x):
        # Unnormalized propagation — matches the analytic Jacobian which is derived
        # without the unit-norm constraint.  Normalizing inside a finite-difference
        # function adds a projection (I - qqᵀ) that the analytic F does not include.
        q = x[0:4]
        b = x[4:7]
        omega = w_meas - b
        q_new = q + 0.5 * omega_matrix(w_meas - b) @ q * dt
        return np.concatenate([q_new, b.copy()])

    F_analytic  = process_jacobian(q0, w_meas - b0, dt)
    F_numerical = _numerical_jacobian(f, x0)

    assert np.allclose(F_analytic, F_numerical, atol=1e-6), (
        f"Max deviation: {np.max(np.abs(F_analytic - F_numerical)):.2e}"
    )


# ---------------------------------------------------------------------------
# measurement_jacobian — numerical verification
# ---------------------------------------------------------------------------

def test_measurement_jacobian_numerical():
    q0 = _rq(3)
    b0 = np.zeros(3)
    x0 = np.concatenate([q0, b0])

    g_ref = np.array([0.0, 0.0, 1.0])
    b_ref = normalize(np.array([0.3090, 0.0, 0.9511]))

    def h(x):
        # Unconstrained q — see note in test_process_jacobian_numerical.
        # The analytic Jacobian treats q as an unconstrained 4-vector; normalizing
        # inside the finite-difference function would add a projection and cause mismatch.
        q = x[0:4]
        R = to_dcm(q)
        return np.concatenate([R @ g_ref, R @ b_ref])

    H_analytic  = measurement_jacobian(q0, g_ref, b_ref)
    H_numerical = _numerical_jacobian(h, x0)

    assert np.allclose(H_analytic, H_numerical, atol=1e-6), (
        f"Max deviation: {np.max(np.abs(H_analytic - H_numerical)):.2e}"
    )


# ---------------------------------------------------------------------------
# DCM Jacobian — spot-check against gravity reference
# ---------------------------------------------------------------------------

def test_dcm_jacobian_gravity_reference():
    """Verify the gravity-specific sub-case analytically."""
    q = _rq(5)
    g_ref = np.array([0.0, 0.0, 1.0])
    q0, q1, q2, q3 = q
    # Expected 3x4 Jacobian for R*e_3
    expected = 2.0 * np.array([
        [-q2,  q3, -q0,  q1],
        [ q1,  q0,  q3,  q2],
        [ q0, -q1, -q2,  q3],
    ])
    computed = _dcm_jacobian_wrt_q(q, g_ref)
    assert np.allclose(computed, expected, atol=1e-12)


# ---------------------------------------------------------------------------
# AttitudeEKF — predict step
# ---------------------------------------------------------------------------

def test_predict_quaternion_unit_norm():
    ekf = AttitudeEKF()
    omega = np.array([0.05, -0.02, 0.01])
    ekf.predict(omega, dt=0.01)
    assert abs(np.linalg.norm(ekf.quaternion) - 1.0) < 1e-10

def test_predict_covariance_grows_without_update():
    """Covariance trace should increase when only predict is called (process noise)."""
    ekf = AttitudeEKF()
    trace_before = ekf.covariance_trace
    for _ in range(20):
        ekf.predict(np.array([0.01, 0.005, -0.008]), dt=0.01)
    assert ekf.covariance_trace > trace_before

def test_predict_zero_omega_leaves_quaternion_unchanged():
    q_init = normalize(np.array([0.9, 0.1, 0.2, 0.3]))
    ekf = AttitudeEKF(q0=q_init)
    # With zero corrected omega (omega_meas == bias), quaternion should not change
    # Set initial bias to zero and feed zero omega
    ekf._x[4:7] = np.zeros(3)
    ekf.predict(np.zeros(3), dt=0.01)
    assert np.allclose(ekf.quaternion, q_init, atol=1e-8)

def test_predict_covariance_positive_definite():
    ekf = AttitudeEKF()
    for _ in range(10):
        ekf.predict(np.array([0.02, -0.01, 0.005]), dt=0.01)
    eigenvalues = np.linalg.eigvalsh(ekf.covariance)
    assert np.all(eigenvalues > 0)


# ---------------------------------------------------------------------------
# AttitudeEKF — update step
# ---------------------------------------------------------------------------

def test_update_reduces_covariance_trace():
    """A measurement update should reduce the covariance trace (information gain)."""
    ekf = AttitudeEKF()
    ekf.predict(np.array([0.01, 0.0, 0.0]), dt=0.01)
    trace_before = ekf.covariance_trace

    q = ekf.quaternion
    R = to_dcm(q)
    accel = R @ np.array([0.0, 0.0, 1.0])
    mag   = R @ normalize(np.array([0.3090, 0.0, 0.9511]))
    ekf.update(accel, mag)

    assert ekf.covariance_trace < trace_before

def test_update_preserves_quaternion_norm():
    ekf = AttitudeEKF()
    ekf.predict(np.array([0.05, -0.03, 0.02]), dt=0.01)
    ekf.update(
        np.array([0.01, 0.02, 0.99]),
        np.array([0.31, 0.02, 0.95]),
    )
    assert abs(np.linalg.norm(ekf.quaternion) - 1.0) < 1e-10

def test_update_covariance_positive_definite():
    ekf = AttitudeEKF()
    ekf.predict(np.array([0.02, -0.01, 0.005]), dt=0.01)
    ekf.update(
        np.array([0.0, 0.0, 1.0]),
        np.array([0.3090, 0.0, 0.9511]),
    )
    eigenvalues = np.linalg.eigvalsh(ekf.covariance)
    assert np.all(eigenvalues > -1e-12)

def test_update_degenerate_accel_is_skipped():
    """Near-zero accelerometer reading should not crash the filter."""
    ekf = AttitudeEKF()
    ekf.predict(np.zeros(3), dt=0.01)
    trace_before = ekf.covariance_trace
    ekf.update(np.array([0.0, 0.0, 0.0]), np.array([0.3090, 0.0, 0.9511]))
    assert ekf.covariance_trace == trace_before  # update was skipped


# ---------------------------------------------------------------------------
# End-to-end convergence test
# ---------------------------------------------------------------------------

def test_ekf_converges_to_known_attitude():
    """
    Filter initialized near true attitude should converge within 10 s (1000 steps at 100 Hz).
    Using perfect (noiseless) measurements.
    """
    from app.ekf.quaternion import error_angle

    q_true = from_euler(0.3, -0.2, 1.1)
    omega_true = np.array([0.01, -0.005, 0.008])

    # Slightly wrong initial attitude
    q_init = q_true + np.array([0.03, -0.01, 0.02, -0.01])
    q_init = normalize(q_init)
    ekf = AttitudeEKF(q0=q_init, sigma_gyro=0.001, sigma_accel=0.01, sigma_mag=0.01)

    g_ref = np.array([0.0, 0.0, 1.0])
    b_ref = normalize(np.array([0.3090, 0.0, 0.9511]))
    dt = 0.01

    for _ in range(1000):
        R = to_dcm(q_true)
        accel_meas = R @ g_ref
        mag_meas   = R @ b_ref
        ekf.predict(omega_true, dt)
        ekf.update(accel_meas, mag_meas)

    final_error_deg = np.degrees(error_angle(ekf.quaternion, q_true))
    assert final_error_deg < 2.0, f"Did not converge: error = {final_error_deg:.3f}°"
