"""
TRIAD Attitude Determination Algorithm (BE-03).

TRIAD (Tri-Axial Attitude Determination) is a deterministic, single-shot method
for computing the attitude DCM from two body-frame vector observations and their
corresponding inertial reference vectors.

Algorithm:
  Given body observations b1 (accel) and b2 (mag) and inertial references
  r1 (gravity) and r2 (magnetic field), construct two orthonormal triad bases:

    Observation triad:            Reference triad:
      t1 = normalize(b1)            s1 = normalize(r1)
      t2 = normalize(b1 × b2)       s2 = normalize(r1 × r2)
      t3 = t1 × t2                  s3 = s1 × s2

  The DCM (inertial → body) that satisfies R @ si ≈ ti is:

      R = [t1|t2|t3] @ [s1|s2|s3]ᵀ

  The first reference vector is matched exactly; the second approximately.

Limitations:
  - No noise filtering; each estimate is independent (no memory between calls)
  - Accuracy degrades under sensor noise; use as a baseline vs EKF
  - Returns None when measurement vectors are nearly parallel (singularity)
"""

import numpy as np
from ..ekf.quaternion import normalize, from_dcm, to_euler

# Default inertial reference vectors (normalised)
G_REF: np.ndarray = np.array([0.0, 0.0, 1.0])           # gravity, pointing down (NED)
B_REF: np.ndarray = normalize(np.array([0.3090, 0.0, 0.9511]))  # representative LEO mag field


class TriadEstimator:
    """
    TRIAD attitude estimator.

    Accepts physical-unit inputs (m/s², µT) or normalised floats — the result
    is identical because both vectors are normalised internally.
    """

    def __init__(
        self,
        g_ref: np.ndarray = G_REF,
        b_ref: np.ndarray = B_REF,
        singularity_threshold: float = 0.1,
    ):
        """
        g_ref:                  normalised gravity reference in inertial frame
        b_ref:                  normalised magnetic field reference in inertial frame
        singularity_threshold:  minimum |b1 × b2| below which None is returned
        """
        self._g_ref    = g_ref / np.linalg.norm(g_ref)
        self._b_ref    = b_ref / np.linalg.norm(b_ref)
        self._threshold = singularity_threshold

        # Pre-compute the reference triad (constant between calls)
        ref_cross      = np.cross(self._g_ref, self._b_ref)
        self._s1       = self._g_ref
        self._s2       = ref_cross / np.linalg.norm(ref_cross)
        self._s3       = np.cross(self._s1, self._s2)
        self._M_ref_T  = np.column_stack([self._s1, self._s2, self._s3]).T

    def set_mag_reference(self, b_ref_normalized: np.ndarray) -> None:
        """
        Update the magnetic-field reference vector and recompute the reference triad.
        Called each tick in live mode to use the real IGRF B-field direction at the
        satellite's current orbital position.
        """
        self._b_ref   = b_ref_normalized / np.linalg.norm(b_ref_normalized)
        ref_cross     = np.cross(self._g_ref, self._b_ref)
        norm_rc       = np.linalg.norm(ref_cross)
        if norm_rc < 1e-6:
            return  # new reference is degenerate — keep old triad
        self._s1      = self._g_ref
        self._s2      = ref_cross / norm_rc
        self._s3      = np.cross(self._s1, self._s2)
        self._M_ref_T = np.column_stack([self._s1, self._s2, self._s3]).T

    def estimate(self, accel: np.ndarray, mag: np.ndarray) -> dict | None:
        """
        Estimate attitude from accelerometer and magnetometer readings.

        Both inputs are normalised internally — physical units (m/s², µT) and
        dimensionless normalised vectors produce identical results.

        Returns None if:
          - either input vector is near-zero
          - |normalize(accel) × normalize(mag)| < singularity_threshold
            (vectors too close to parallel for a stable triad)

        On success returns a dict with:
          algorithm  — "TRIAD"
          quaternion — np.ndarray [q0, q1, q2, q3]
          roll       — float, degrees (ZYX convention)
          pitch      — float, degrees
          yaw        — float, degrees
        """
        a_norm = np.linalg.norm(accel)
        m_norm = np.linalg.norm(mag)

        if a_norm < 1e-6 or m_norm < 1e-6:
            return None

        b1 = accel / a_norm   # primary body observation  (gravity direction in body)
        b2 = mag   / m_norm   # secondary body observation (mag field direction in body)

        # Singularity guard: vectors must not be nearly parallel
        cross      = np.cross(b1, b2)
        cross_norm = np.linalg.norm(cross)
        if cross_norm < self._threshold:
            return None

        # Build observation triad
        t1 = b1
        t2 = cross / cross_norm   # = normalize(b1 × b2)
        t3 = np.cross(t1, t2)

        # DCM (inertial → body): R = M_obs @ M_ref^T
        M_obs = np.column_stack([t1, t2, t3])
        R     = M_obs @ self._M_ref_T

        q          = from_dcm(R)
        roll, pitch, yaw = to_euler(q)

        return {
            "algorithm": "TRIAD",
            "quaternion": q,
            "roll":  float(np.degrees(roll)),
            "pitch": float(np.degrees(pitch)),
            "yaw":   float(np.degrees(yaw)),
        }
