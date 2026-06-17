"""
Unit tests for quaternion math utilities.
Run: pytest tests/test_quaternion.py -v
"""

import numpy as np
import pytest
from app.ekf.quaternion import (
    normalize, multiply, conjugate, to_dcm, from_dcm, to_euler, from_euler, error_angle
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _random_unit_q(seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    q = rng.normal(size=4)
    return q / np.linalg.norm(q)

IDENTITY_Q = np.array([1.0, 0.0, 0.0, 0.0])


# ---------------------------------------------------------------------------
# normalize
# ---------------------------------------------------------------------------

def test_normalize_unit_vector():
    q = np.array([2.0, 0.0, 0.0, 0.0])
    assert np.allclose(normalize(q), [1.0, 0.0, 0.0, 0.0])

def test_normalize_preserves_direction():
    q = np.array([1.0, 2.0, 3.0, 4.0])
    qn = normalize(q)
    assert abs(np.linalg.norm(qn) - 1.0) < 1e-12

def test_normalize_already_unit():
    q = _random_unit_q(seed=1)
    assert np.allclose(normalize(q), q)

def test_normalize_zero_raises():
    with pytest.raises(ValueError):
        normalize(np.zeros(4))


# ---------------------------------------------------------------------------
# multiply
# ---------------------------------------------------------------------------

def test_multiply_identity_right():
    q = _random_unit_q(seed=2)
    assert np.allclose(multiply(q, IDENTITY_Q), q, atol=1e-12)

def test_multiply_identity_left():
    q = _random_unit_q(seed=3)
    assert np.allclose(multiply(IDENTITY_Q, q), q, atol=1e-12)

def test_multiply_self_conjugate_is_identity():
    q = _random_unit_q(seed=4)
    result = multiply(q, conjugate(q))
    # q ⊗ q* = identity (up to sign, but scalar part should be 1)
    assert abs(abs(result[0]) - 1.0) < 1e-12
    assert np.allclose(result[1:], 0.0, atol=1e-12)

def test_multiply_associativity():
    a = _random_unit_q(0); b = _random_unit_q(1); c = _random_unit_q(2)
    assert np.allclose(multiply(multiply(a, b), c), multiply(a, multiply(b, c)), atol=1e-12)

def test_multiply_norm_preserved():
    p = _random_unit_q(5); q = _random_unit_q(6)
    assert abs(np.linalg.norm(multiply(p, q)) - 1.0) < 1e-12


# ---------------------------------------------------------------------------
# conjugate
# ---------------------------------------------------------------------------

def test_conjugate_scalar_unchanged():
    q = _random_unit_q(7)
    assert conjugate(q)[0] == q[0]

def test_conjugate_vector_negated():
    q = _random_unit_q(8)
    assert np.allclose(conjugate(q)[1:], -q[1:])

def test_conjugate_twice_is_identity():
    q = _random_unit_q(9)
    assert np.allclose(conjugate(conjugate(q)), q)


# ---------------------------------------------------------------------------
# to_dcm
# ---------------------------------------------------------------------------

def test_dcm_identity_quaternion():
    R = to_dcm(IDENTITY_Q)
    assert np.allclose(R, np.eye(3), atol=1e-12)

def test_dcm_is_orthogonal():
    q = _random_unit_q(10)
    R = to_dcm(q)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-10)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-10)

def test_dcm_determinant_is_one():
    q = _random_unit_q(11)
    R = to_dcm(q)
    assert abs(np.linalg.det(R) - 1.0) < 1e-10

def test_dcm_pure_yaw_90():
    # 90° CCW yaw: body x-axis = inertial y-axis.
    # DCM maps inertial→body, so inertial [1,0,0] appears as body [0,-1,0].
    q = normalize(np.array([np.cos(np.pi/4), 0.0, 0.0, np.sin(np.pi/4)]))
    R = to_dcm(q)
    v_body = R @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(v_body, [0.0, -1.0, 0.0], atol=1e-10)


# ---------------------------------------------------------------------------
# from_dcm
# ---------------------------------------------------------------------------

def test_from_dcm_identity():
    """from_dcm(I₃) must give the identity quaternion."""
    q = from_dcm(np.eye(3))
    assert np.allclose(q, IDENTITY_Q, atol=1e-10) or np.allclose(q, -IDENTITY_Q, atol=1e-10)

def test_from_dcm_roundtrip():
    """from_dcm(to_dcm(q)) must recover q (up to double-cover sign)."""
    for seed in range(8):
        q = _random_unit_q(seed + 50)
        R = to_dcm(q)
        q2 = from_dcm(R)
        same = np.allclose(q, q2, atol=1e-10) or np.allclose(q, -q2, atol=1e-10)
        assert same, f"Roundtrip failed for seed={seed}: {q} vs {q2}"

def test_from_dcm_to_dcm_roundtrip():
    """to_dcm(from_dcm(R)) must recover R exactly."""
    for seed in range(5):
        q = _random_unit_q(seed + 60)
        R = to_dcm(q)
        q2 = from_dcm(R)
        R2 = to_dcm(q2)
        assert np.allclose(R, R2, atol=1e-10), f"DCM roundtrip failed for seed={seed}"

def test_from_dcm_pure_yaw_90():
    """90° CCW yaw DCM must recover a yaw-90° quaternion."""
    # DCM for 90° CCW yaw: [[0,1,0],[-1,0,0],[0,0,1]]
    R = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    q = from_dcm(R)
    # Verify by re-converting to DCM
    assert np.allclose(to_dcm(q), R, atol=1e-10)

def test_from_dcm_all_four_branches():
    """Exercise each Shepperd branch by constructing DCMs where each qᵢ is the largest."""
    # q0 largest: identity-like
    q_q0 = normalize(np.array([0.9, 0.1, 0.1, 0.1]))
    assert np.allclose(from_dcm(to_dcm(q_q0)), q_q0, atol=1e-9) or \
           np.allclose(from_dcm(to_dcm(q_q0)), -q_q0, atol=1e-9)
    # q1 largest
    q_q1 = normalize(np.array([0.1, 0.9, 0.1, 0.1]))
    assert np.allclose(from_dcm(to_dcm(q_q1)), q_q1, atol=1e-9) or \
           np.allclose(from_dcm(to_dcm(q_q1)), -q_q1, atol=1e-9)
    # q2 largest
    q_q2 = normalize(np.array([0.1, 0.1, 0.9, 0.1]))
    assert np.allclose(from_dcm(to_dcm(q_q2)), q_q2, atol=1e-9) or \
           np.allclose(from_dcm(to_dcm(q_q2)), -q_q2, atol=1e-9)
    # q3 largest
    q_q3 = normalize(np.array([0.1, 0.1, 0.1, 0.9]))
    assert np.allclose(from_dcm(to_dcm(q_q3)), q_q3, atol=1e-9) or \
           np.allclose(from_dcm(to_dcm(q_q3)), -q_q3, atol=1e-9)


# ---------------------------------------------------------------------------
# Euler angles
# ---------------------------------------------------------------------------

def test_euler_roundtrip():
    for seed in range(5):
        q = _random_unit_q(seed + 20)
        r, p, y = to_euler(q)
        q2 = from_euler(r, p, y)
        # q and -q represent the same rotation (double cover); accept either sign
        same = np.allclose(q, q2, atol=1e-10) or np.allclose(q, -q2, atol=1e-10)
        assert same, f"Roundtrip failed for seed={seed}: {q} vs {q2}"

def test_euler_identity_is_zero():
    r, p, y = to_euler(IDENTITY_Q)
    assert np.allclose([r, p, y], 0.0, atol=1e-12)

def test_euler_pitch_90():
    # 90° pitch
    q = from_euler(0.0, np.pi/2, 0.0)
    _, pitch, _ = to_euler(q)
    assert abs(pitch - np.pi/2) < 1e-10


# ---------------------------------------------------------------------------
# error_angle
# ---------------------------------------------------------------------------

def test_error_angle_same_quaternion_is_zero():
    q = _random_unit_q(30)
    assert abs(error_angle(q, q)) < 1e-10

def test_error_angle_negated_quaternion_is_zero():
    q = _random_unit_q(31)
    assert abs(error_angle(q, -q)) < 1e-10

def test_error_angle_known_rotation():
    # Two quaternions separated by exactly 90°
    q1 = normalize(np.array([1.0, 0.0, 0.0, 0.0]))
    q2 = normalize(np.array([np.cos(np.pi/4), 0.0, 0.0, np.sin(np.pi/4)]))
    angle = error_angle(q1, q2)
    assert abs(angle - np.pi/2) < 1e-10

def test_error_angle_symmetry():
    q1 = _random_unit_q(32); q2 = _random_unit_q(33)
    assert abs(error_angle(q1, q2) - error_angle(q2, q1)) < 1e-10
