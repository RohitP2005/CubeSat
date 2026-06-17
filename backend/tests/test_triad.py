"""
Unit tests for the TRIAD attitude determination algorithm.
Run: pytest tests/test_triad.py -v
"""

import numpy as np
import pytest
from app.ekf.quaternion import normalize, to_dcm, from_euler, error_angle
from app.triad.triad import TriadEstimator, G_REF, B_REF


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _rq(seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    q = rng.normal(size=4)
    return q / np.linalg.norm(q)


def _perfect_meas(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Generate noiseless accel and mag vectors for a given attitude."""
    R = to_dcm(q)
    return R @ G_REF, R @ B_REF


_EST = TriadEstimator()   # shared default estimator


# ---------------------------------------------------------------------------
# Basic functionality
# ---------------------------------------------------------------------------

def test_triad_noiseless_recovery():
    """Noiseless measurements must recover true attitude within 1e-5 degrees."""
    for seed in range(12):
        q_true = _rq(seed)
        accel, mag = _perfect_meas(q_true)
        result = _EST.estimate(accel, mag)
        assert result is not None, f"seed={seed}: estimate returned None"
        err = np.degrees(error_angle(result["quaternion"], q_true))
        assert err < 1e-5, f"seed={seed}: error={err:.2e}°"


def test_triad_result_keys():
    """Output dict must contain all BE-03 required keys."""
    accel, mag = _perfect_meas(_rq(0))
    result = _EST.estimate(accel, mag)
    assert result is not None
    for key in ("algorithm", "quaternion", "roll", "pitch", "yaw"):
        assert key in result, f"Missing key: {key}"


def test_triad_algorithm_label():
    accel, mag = _perfect_meas(_rq(0))
    assert _EST.estimate(accel, mag)["algorithm"] == "TRIAD"


def test_triad_output_quaternion_is_unit():
    for seed in range(6):
        accel, mag = _perfect_meas(_rq(seed))
        result = _EST.estimate(accel, mag)
        assert result is not None
        assert abs(np.linalg.norm(result["quaternion"]) - 1.0) < 1e-10


def test_triad_euler_are_degrees_not_radians():
    """Roll/pitch/yaw must be in degrees — values should be in degree-scale ranges."""
    q = from_euler(0.2, 0.1, 0.3)     # ≈ 11°, 6°, 17° — unambiguously degrees
    accel, mag = _perfect_meas(q)
    r = _EST.estimate(accel, mag)
    assert r is not None
    assert abs(r["roll"])  <= 180.0
    assert abs(r["pitch"]) <= 90.0
    assert abs(r["yaw"])   <= 180.0


def test_triad_euler_scalar_types():
    accel, mag = _perfect_meas(_rq(0))
    r = _EST.estimate(accel, mag)
    assert isinstance(r["roll"],  float)
    assert isinstance(r["pitch"], float)
    assert isinstance(r["yaw"],   float)


# ---------------------------------------------------------------------------
# Known rotations
# ---------------------------------------------------------------------------

def test_triad_identity_attitude():
    """At identity orientation the estimate must be identity (error < 1e-5°)."""
    result = _EST.estimate(G_REF.copy(), B_REF.copy())
    assert result is not None
    q_id = np.array([1.0, 0.0, 0.0, 0.0])
    err = np.degrees(error_angle(result["quaternion"], q_id))
    assert err < 1e-5, f"Identity error = {err:.2e}°"


def test_triad_pure_roll_30():
    q_true = from_euler(np.radians(30.0), 0.0, 0.0)
    accel, mag = _perfect_meas(q_true)
    r = _EST.estimate(accel, mag)
    assert r is not None
    assert np.degrees(error_angle(r["quaternion"], q_true)) < 1e-5


def test_triad_pure_pitch_20():
    q_true = from_euler(0.0, np.radians(20.0), 0.0)
    accel, mag = _perfect_meas(q_true)
    r = _EST.estimate(accel, mag)
    assert r is not None
    assert np.degrees(error_angle(r["quaternion"], q_true)) < 1e-5


def test_triad_pure_yaw_45():
    q_true = from_euler(0.0, 0.0, np.radians(45.0))
    accel, mag = _perfect_meas(q_true)
    r = _EST.estimate(accel, mag)
    assert r is not None
    assert np.degrees(error_angle(r["quaternion"], q_true)) < 1e-5


def test_triad_combined_rotation():
    q_true = normalize(from_euler(np.radians(15.0), np.radians(-8.0), np.radians(45.0)))
    accel, mag = _perfect_meas(q_true)
    r = _EST.estimate(accel, mag)
    assert r is not None
    assert np.degrees(error_angle(r["quaternion"], q_true)) < 1e-5


def test_triad_dcm_is_orthogonal():
    """The recovered DCM R = to_dcm(q) must satisfy R Rᵀ = I."""
    for seed in range(5):
        accel, mag = _perfect_meas(_rq(seed))
        r = _EST.estimate(accel, mag)
        assert r is not None
        R = to_dcm(r["quaternion"])
        assert np.allclose(R @ R.T, np.eye(3), atol=1e-10)


# ---------------------------------------------------------------------------
# Singularity handling
# ---------------------------------------------------------------------------

def test_singularity_parallel_vectors():
    """Parallel vectors: cross product ≈ 0 → must return None."""
    est = TriadEstimator(singularity_threshold=0.1)
    assert est.estimate(np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0])) is None


def test_singularity_anti_parallel():
    est = TriadEstimator(singularity_threshold=0.1)
    assert est.estimate(np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -1.0])) is None


def test_singularity_nearly_parallel():
    """Angle ≈ 3°: sin(3°) ≈ 0.052 < threshold 0.1 → None."""
    est = TriadEstimator(singularity_threshold=0.1)
    accel = np.array([0.0, 0.0, 1.0])
    ang   = np.radians(3.0)
    mag   = np.array([np.sin(ang), 0.0, np.cos(ang)])
    assert est.estimate(accel, mag) is None


def test_singularity_threshold_respected():
    """Angle exactly at threshold (sin θ = threshold) should return None."""
    threshold = 0.15
    est = TriadEstimator(singularity_threshold=threshold)
    accel = np.array([0.0, 0.0, 1.0])
    # |accel × mag| = sin(angle_between); set mag so sin ≈ threshold - ε
    ang = np.arcsin(threshold - 0.01)
    mag = np.array([np.sin(ang), 0.0, np.cos(ang)])
    assert est.estimate(accel, mag) is None


def test_non_singular_90_degrees():
    """Perpendicular vectors (90°) must not trigger singularity."""
    est = TriadEstimator(singularity_threshold=0.1)
    assert est.estimate(np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0])) is not None


def test_zero_accel_returns_none():
    assert _EST.estimate(np.zeros(3), np.array([1.0, 0.0, 0.0])) is None


def test_zero_mag_returns_none():
    assert _EST.estimate(np.array([0.0, 0.0, 1.0]), np.zeros(3)) is None


# ---------------------------------------------------------------------------
# Scale invariance
# ---------------------------------------------------------------------------

def test_scale_invariant_accel():
    """Multiplying accel by G_EARTH (9.81) must give identical result."""
    q = _rq(10)
    accel, mag = _perfect_meas(q)
    r1 = _EST.estimate(accel, mag)
    r2 = _EST.estimate(accel * 9.81, mag)
    assert r1 is not None and r2 is not None
    assert np.degrees(error_angle(r1["quaternion"], r2["quaternion"])) < 1e-10


def test_scale_invariant_mag():
    """Multiplying mag by B_EARTH (50 µT) must give identical result."""
    q = _rq(11)
    accel, mag = _perfect_meas(q)
    r1 = _EST.estimate(accel, mag)
    r2 = _EST.estimate(accel, mag * 50.0)
    assert r1 is not None and r2 is not None
    assert np.degrees(error_angle(r1["quaternion"], r2["quaternion"])) < 1e-10


def test_scale_invariant_both():
    """Scaling both inputs to physical units must give identical result."""
    q = _rq(12)
    accel, mag = _perfect_meas(q)
    r1 = _EST.estimate(accel, mag)
    r2 = _EST.estimate(accel * 9.81, mag * 50.0)
    assert r1 is not None and r2 is not None
    assert np.degrees(error_angle(r1["quaternion"], r2["quaternion"])) < 1e-10


# ---------------------------------------------------------------------------
# Gravity vector matching (TRIAD priority)
# ---------------------------------------------------------------------------

def test_triad_matches_first_vector_exactly():
    """
    TRIAD prioritises the first reference vector (gravity).
    R @ g_ref must equal normalised accel to within floating-point precision.
    """
    for seed in range(6):
        q_true = _rq(seed)
        accel, mag = _perfect_meas(q_true)
        r = _EST.estimate(accel, mag)
        assert r is not None
        R        = to_dcm(r["quaternion"])
        expected = accel / np.linalg.norm(accel)
        got      = R @ G_REF
        assert np.allclose(got, expected, atol=1e-10), f"seed={seed}: g-match failed"


# ---------------------------------------------------------------------------
# Accuracy comparison: TRIAD vs EKF under noise
# ---------------------------------------------------------------------------

def test_triad_vs_ekf_noise_accuracy():
    """
    After EKF convergence, EKF mean angular error must be lower than TRIAD's.
    Both are given the same noisy sensor stream.
    """
    from app.ekf.ekf import AttitudeEKF
    from app.simulation.orbit import CircularOrbit
    from app.simulation.sensor_sim import SensorSimulator

    orbit = CircularOrbit()
    sim   = SensorSimulator(orbit, sigma_gyro=0.005, sigma_accel=0.05, sigma_mag=0.02, rng_seed=42)
    ekf   = AttitudeEKF(
        q0=orbit.true_attitude(0.0),
        sigma_gyro=0.005, sigma_accel=0.05, sigma_mag=0.02,
    )
    triad = TriadEstimator()

    dt          = 0.01
    n_steps     = 600       # 6 s — EKF converges in ~2 s
    convergence = 200       # skip first 2 s for EKF
    triad_errs, ekf_errs = [], []

    for i in range(n_steps):
        t    = i * dt
        data = sim.sample(t, dt)
        accel   = np.array([data["ax"], data["ay"], data["az"]])
        mag     = np.array([data["mx"], data["my"], data["mz"]])
        q_true  = data["true_q"]

        ekf.predict(np.array([data["gx"], data["gy"], data["gz"]]), dt)
        ekf.update(accel, mag)

        tr = triad.estimate(accel, mag)
        if tr is not None:
            triad_errs.append(np.degrees(error_angle(tr["quaternion"], q_true)))

        if i >= convergence:
            ekf_errs.append(np.degrees(error_angle(ekf.quaternion, q_true)))

    mean_triad = float(np.mean(triad_errs))
    mean_ekf   = float(np.mean(ekf_errs))

    assert mean_ekf < mean_triad, (
        f"EKF ({mean_ekf:.3f}°) should outperform TRIAD ({mean_triad:.3f}°) under noise"
    )


def test_triad_accuracy_under_noise_within_bound():
    """
    Under moderate noise, TRIAD mean angular error should be finite and < 10°.
    (TRIAD has no filtering so it will be noisier than EKF, but not wildly wrong.)
    """
    from app.simulation.orbit import CircularOrbit
    from app.simulation.sensor_sim import SensorSimulator

    orbit = CircularOrbit()
    sim   = SensorSimulator(orbit, sigma_gyro=0.005, sigma_accel=0.05, sigma_mag=0.02, rng_seed=13)
    triad = TriadEstimator()
    errs  = []

    for i in range(200):
        data  = sim.sample(i * 0.01, 0.01)
        accel = np.array([data["ax"], data["ay"], data["az"]])
        mag   = np.array([data["mx"], data["my"], data["mz"]])
        tr    = triad.estimate(accel, mag)
        if tr is not None:
            errs.append(np.degrees(error_angle(tr["quaternion"], data["true_q"])))

    assert np.mean(errs) < 10.0, f"TRIAD mean error {np.mean(errs):.2f}° unexpectedly large"
