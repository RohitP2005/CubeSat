"""
Quaternion math utilities.

Convention: scalar-first, [q0, q1, q2, q3] where q0 is the scalar part.
The DCM R(q) maps vectors from inertial frame to body frame: v_body = R(q) * v_inertial.
"""

import numpy as np


def normalize(q: np.ndarray) -> np.ndarray:
    """Return unit quaternion."""
    n = np.linalg.norm(q)
    if n < 1e-10:
        raise ValueError(f"Quaternion norm {n} too small to normalize")
    return q / n


def multiply(p: np.ndarray, q: np.ndarray) -> np.ndarray:
    """Hamilton product p ⊗ q (scalar-first convention)."""
    p0, p1, p2, p3 = p
    q0, q1, q2, q3 = q
    return np.array([
        p0*q0 - p1*q1 - p2*q2 - p3*q3,
        p0*q1 + p1*q0 + p2*q3 - p3*q2,
        p0*q2 - p1*q3 + p2*q0 + p3*q1,
        p0*q3 + p1*q2 - p2*q1 + p3*q0,
    ])


def conjugate(q: np.ndarray) -> np.ndarray:
    """Return q* = [q0, -q1, -q2, -q3]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def to_dcm(q: np.ndarray) -> np.ndarray:
    """Convert unit quaternion to 3x3 DCM (inertial-to-body)."""
    q0, q1, q2, q3 = q
    return np.array([
        [q0**2+q1**2-q2**2-q3**2,  2*(q1*q2+q0*q3),           2*(q1*q3-q0*q2)       ],
        [2*(q1*q2-q0*q3),           q0**2-q1**2+q2**2-q3**2,   2*(q2*q3+q0*q1)       ],
        [2*(q1*q3+q0*q2),           2*(q2*q3-q0*q1),           q0**2-q1**2-q2**2+q3**2],
    ])


def to_euler(q: np.ndarray) -> tuple[float, float, float]:
    """
    Convert unit quaternion to ZYX Euler angles in radians.
    Returns (roll, pitch, yaw).
    """
    q0, q1, q2, q3 = q
    roll  = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
    sinp  = 2*(q0*q2 - q3*q1)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    yaw   = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))
    return roll, pitch, yaw


def from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build unit quaternion from ZYX Euler angles (roll, pitch, yaw) in radians."""
    cr, sr = np.cos(roll / 2),  np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2),   np.sin(yaw / 2)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])


def from_dcm(R: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix (DCM) to a unit quaternion.

    Uses Shepperd's method for numerical stability — branches on the largest
    diagonal element to avoid division by near-zero denominators.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s  = 0.5 / np.sqrt(trace + 1.0)
        q0 = 0.25 / s
        q1 = (R[1, 2] - R[2, 1]) * s
        q2 = (R[2, 0] - R[0, 2]) * s
        q3 = (R[0, 1] - R[1, 0]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s  = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q0 = (R[1, 2] - R[2, 1]) / s
        q1 = 0.25 * s
        q2 = (R[0, 1] + R[1, 0]) / s
        q3 = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s  = 2.0 * np.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2])
        q0 = (R[2, 0] - R[0, 2]) / s
        q1 = (R[0, 1] + R[1, 0]) / s
        q2 = 0.25 * s
        q3 = (R[1, 2] + R[2, 1]) / s
    else:
        s  = 2.0 * np.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2])
        q0 = (R[0, 1] - R[1, 0]) / s
        q1 = (R[0, 2] + R[2, 0]) / s
        q2 = (R[1, 2] + R[2, 1]) / s
        q3 = 0.25 * s
    return normalize(np.array([q0, q1, q2, q3]))


def error_angle(q1: np.ndarray, q2: np.ndarray) -> float:
    """
    Angular separation between two orientations in radians.
    Handles the quaternion double-cover (q and -q represent the same rotation).
    """
    q_err = normalize(multiply(conjugate(normalize(q1)), normalize(q2)))
    if q_err[0] < 0:
        q_err = -q_err
    return 2.0 * np.arccos(np.clip(q_err[0], -1.0, 1.0))
