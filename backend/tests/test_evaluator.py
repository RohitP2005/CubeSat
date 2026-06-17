"""
Unit tests for PerformanceEvaluator.

Covers:
  - Per-cycle recording and latest-value access
  - Known-answer RMSE computation (must match NumPy reference within 1e-6)
  - Convergence window: early steps excluded from aggregate metrics
  - Buffer cap: deque never exceeds maxlen
  - TRIAD singularity (None) handled without affecting TRIAD buffer
  - clear() fully resets state
  - Edge cases: empty buffer, single sample, all pre-convergence
"""

import numpy as np
import pytest

from app.evaluation.evaluator import PerformanceEvaluator, _wrap_deg
from app.ekf.quaternion import normalize, from_euler, error_angle, to_euler


# ── Helpers ───────────────────────────────────────────────────────────────────

def _q_identity() -> np.ndarray:
    return np.array([1.0, 0.0, 0.0, 0.0])

def _q_roll(deg: float) -> np.ndarray:
    return normalize(from_euler(np.radians(deg), 0.0, 0.0))

def _q_yaw(deg: float) -> np.ndarray:
    return normalize(from_euler(0.0, 0.0, np.radians(deg)))

def _make_evaluator(**kw) -> PerformanceEvaluator:
    return PerformanceEvaluator(**kw)


# ── _wrap_deg helper ──────────────────────────────────────────────────────────

def test_wrap_deg_no_wrap_needed():
    assert abs(_wrap_deg(45.0) - 45.0)  < 1e-10
    assert abs(_wrap_deg(-90.0) - (-90.0)) < 1e-10

def test_wrap_deg_wraps_at_180():
    assert abs(_wrap_deg(190.0) - (-170.0)) < 1e-10

def test_wrap_deg_wraps_negative():
    assert abs(_wrap_deg(-190.0) - 170.0) < 1e-10

def test_wrap_deg_exactly_180():
    # 180° maps to -180°  ((-180, 180] convention)
    result = _wrap_deg(180.0)
    assert abs(abs(result) - 180.0) < 1e-10


# ── Record and latest access ──────────────────────────────────────────────────

def test_record_increments_step():
    ev = _make_evaluator()
    assert ev.step_count == 0
    ev.record(_q_identity(), None, _q_identity())
    assert ev.step_count == 1
    ev.record(_q_identity(), None, _q_identity())
    assert ev.step_count == 2

def test_angular_error_zero_for_identical():
    ev = _make_evaluator(convergence_steps=0)
    q = normalize(from_euler(0.3, -0.2, 1.1))
    ev.record(q, None, q)
    assert abs(ev.angular_error_ekf()) < 1e-10

def test_angular_error_ekf_known_rotation():
    """10° roll rotation should produce ~10° angular error."""
    ev = _make_evaluator(convergence_steps=0)
    q_true = _q_identity()
    q_ekf  = _q_roll(10.0)
    ev.record(q_ekf, None, q_true)
    assert abs(ev.angular_error_ekf() - 10.0) < 0.01

def test_angular_error_triad_recorded():
    ev = _make_evaluator(convergence_steps=0)
    q_true  = _q_identity()
    q_triad = _q_roll(5.0)
    ev.record(_q_identity(), q_triad, q_true)
    assert abs(ev.angular_error_triad() - 5.0) < 0.01

def test_angular_error_triad_none_not_recorded():
    """When TRIAD returns None (singularity), the TRIAD buffer must stay empty."""
    ev = _make_evaluator()
    ev.record(_q_identity(), None, _q_identity())
    assert ev.angular_error_triad() is None
    assert ev.triad_buffer_size == 0

def test_latest_ekf_has_all_keys():
    ev = _make_evaluator()
    ev.record(_q_roll(5.0), None, _q_identity())
    latest = ev.latest_ekf()
    assert latest is not None
    for k in ("angular_error_deg", "roll_error_deg", "pitch_error_deg", "yaw_error_deg"):
        assert k in latest

def test_latest_triad_has_all_keys():
    ev = _make_evaluator()
    ev.record(_q_identity(), _q_roll(3.0), _q_identity())
    latest = ev.latest_triad()
    assert latest is not None
    for k in ("angular_error_deg", "roll_error_deg", "pitch_error_deg", "yaw_error_deg"):
        assert k in latest

def test_latest_ekf_none_before_record():
    ev = _make_evaluator()
    assert ev.latest_ekf()   is None
    assert ev.angular_error_ekf() is None

def test_latest_triad_none_before_record():
    ev = _make_evaluator()
    assert ev.latest_triad() is None
    assert ev.angular_error_triad() is None


# ── Per-axis error correctness ────────────────────────────────────────────────

def test_roll_error_correct():
    """30° roll error: roll_error_deg should be ~30°, pitch/yaw near 0."""
    ev = _make_evaluator(convergence_steps=0)
    ev.record(_q_roll(30.0), None, _q_identity())
    latest = ev.latest_ekf()
    assert abs(latest["roll_error_deg"]  - 30.0) < 0.1
    assert abs(latest["pitch_error_deg"])         < 0.1
    assert abs(latest["yaw_error_deg"])           < 0.1

def test_yaw_error_wrapping():
    """179° vs -179° yaw: error should be ~2°, not 358°."""
    ev = _make_evaluator(convergence_steps=0)
    q_true = _q_yaw(179.0)
    q_est  = _q_yaw(-179.0)
    ev.record(q_est, None, q_true)
    latest = ev.latest_ekf()
    assert latest["yaw_error_deg"] < 5.0   # wrapped correctly

def test_pitch_error_no_wrapping_needed():
    """20° pitch difference should give ~20° pitch error."""
    ev = _make_evaluator(convergence_steps=0)
    ev.record(from_euler(0, np.radians(20), 0), None, _q_identity())
    latest = ev.latest_ekf()
    assert abs(latest["pitch_error_deg"] - 20.0) < 0.1


# ── Known-answer RMSE ─────────────────────────────────────────────────────────

def test_rmse_angular_matches_numpy_reference():
    """RMSE computed by evaluator must match NumPy reference within 1e-6."""
    ev  = _make_evaluator(convergence_steps=0)
    rng = np.random.default_rng(0)
    q_true  = _q_identity()
    errors  = []

    for _ in range(200):
        angle = rng.uniform(0.5, 8.0)   # degrees
        q_est = _q_roll(angle)
        ev.record(q_est, None, q_true)
        errors.append(np.degrees(error_angle(q_est, q_true)))

    expected = float(np.sqrt(np.mean(np.square(errors))))
    actual   = ev.summary()["ekf"]["rmse_angular"]
    assert abs(actual - expected) < 1e-6, f"Expected {expected:.8f}, got {actual:.8f}"

def test_rmse_roll_matches_numpy_reference():
    """Per-axis RMSE must match manual NumPy computation within 1e-6."""
    ev  = _make_evaluator(convergence_steps=0)
    rng = np.random.default_rng(1)
    q_true  = _q_identity()
    roll_errs = []

    for _ in range(200):
        angle = rng.uniform(0.0, 10.0)
        q_est = _q_roll(angle)
        ev.record(q_est, None, q_true)
        r_e, _, _ = to_euler(q_est)
        r_t, _, _ = to_euler(q_true)
        roll_errs.append(abs(_wrap_deg(np.degrees(r_e) - np.degrees(r_t))))

    expected = float(np.sqrt(np.mean(np.square(roll_errs))))
    actual   = ev.summary()["ekf"]["rmse_roll"]
    assert abs(actual - expected) < 1e-6

def test_constant_error_rmse():
    """N identical errors of X° → RMSE = X°."""
    ev  = _make_evaluator(convergence_steps=0)
    q_true = _q_identity()
    q_est  = _q_roll(5.0)
    for _ in range(100):
        ev.record(q_est, None, q_true)
    s = ev.summary()["ekf"]
    assert abs(s["rmse_angular"]  - 5.0) < 0.01
    assert abs(s["mean_error_deg"] - 5.0) < 0.01


# ── Convergence window ────────────────────────────────────────────────────────

def test_convergence_window_excludes_early_steps():
    """
    Pre-convergence: 10° errors (steps 0–99).
    Post-convergence: 1° errors (steps 100–199).
    With convergence_steps=100, aggregate must reflect only the 1° data.
    """
    ev = _make_evaluator(convergence_steps=100)
    q_true = _q_identity()

    q_bad  = _q_roll(10.0)
    q_good = _q_roll(1.0)

    for _ in range(100):
        ev.record(q_bad,  None, q_true)
    for _ in range(100):
        ev.record(q_good, None, q_true)

    s = ev.summary()["ekf"]
    assert s["sample_count"] == 100          # only post-convergence
    assert abs(s["rmse_angular"] - 1.0) < 0.01

def test_convergence_window_fallback_when_insufficient_data():
    """
    When all recorded steps are pre-convergence, fallback to all available.
    """
    ev = _make_evaluator(convergence_steps=1000)
    q_true = _q_identity()
    for _ in range(10):
        ev.record(_q_roll(5.0), None, q_true)

    s = ev.summary()["ekf"]
    assert s["sample_count"] == 10   # fallback: use all


# ── Buffer cap ────────────────────────────────────────────────────────────────

def test_buffer_does_not_grow_beyond_maxlen():
    maxlen = 50
    ev = _make_evaluator(maxlen=maxlen, convergence_steps=0)
    q_true = _q_identity()
    for _ in range(200):
        ev.record(_q_identity(), _q_roll(2.0), q_true)
    assert ev.ekf_buffer_size   <= maxlen
    assert ev.triad_buffer_size <= maxlen

def test_buffer_wraps_correctly():
    """After wrap, aggregate is computed over the most recent maxlen samples."""
    maxlen = 100
    ev = _make_evaluator(maxlen=maxlen, convergence_steps=0)
    q_true = _q_identity()

    # First 100 steps: 10° error
    for _ in range(100):
        ev.record(_q_roll(10.0), None, q_true)
    # Next 100 steps: 1° error — overwrites the 10° data in the buffer
    for _ in range(100):
        ev.record(_q_roll(1.0), None, q_true)

    s = ev.summary()["ekf"]
    assert abs(s["rmse_angular"] - 1.0) < 0.05


# ── Summary structure ─────────────────────────────────────────────────────────

def test_summary_keys_present_before_record():
    ev = _make_evaluator()
    s  = ev.summary()
    assert "step_count"        in s
    assert "convergence_steps" in s
    assert "ekf"               in s
    assert "triad"             in s
    assert s["step_count"] == 0

def test_summary_ekf_keys():
    ev = _make_evaluator(convergence_steps=0)
    ev.record(_q_roll(3.0), None, _q_identity())
    ekf = ev.summary()["ekf"]
    for k in ("sample_count", "mean_error_deg", "rmse_angular",
              "rmse_roll", "rmse_pitch", "rmse_yaw"):
        assert k in ekf, f"Missing key: {k}"

def test_summary_triad_keys():
    ev = _make_evaluator(convergence_steps=0)
    ev.record(_q_identity(), _q_roll(3.0), _q_identity())
    triad = ev.summary()["triad"]
    for k in ("sample_count", "mean_error_deg", "rmse_angular",
              "rmse_roll", "rmse_pitch", "rmse_yaw"):
        assert k in triad, f"Missing key: {k}"

def test_summary_ekf_empty_returns_none_values():
    ev = _make_evaluator()
    ekf = ev.summary()["ekf"]
    assert ekf["sample_count"]   == 0
    assert ekf["mean_error_deg"] is None
    assert ekf["rmse_angular"]   is None


# ── clear() ───────────────────────────────────────────────────────────────────

def test_clear_resets_step_count():
    ev = _make_evaluator()
    for _ in range(50):
        ev.record(_q_identity(), None, _q_identity())
    ev.clear()
    assert ev.step_count == 0

def test_clear_resets_buffers():
    ev = _make_evaluator()
    ev.record(_q_roll(5.0), _q_roll(3.0), _q_identity())
    ev.clear()
    assert ev.ekf_buffer_size   == 0
    assert ev.triad_buffer_size == 0
    assert ev.latest_ekf()   is None
    assert ev.latest_triad() is None

def test_clear_then_record_works():
    ev = _make_evaluator(convergence_steps=0)
    for _ in range(10):
        ev.record(_q_roll(10.0), None, _q_identity())
    ev.clear()
    ev.record(_q_roll(2.0), None, _q_identity())
    assert abs(ev.angular_error_ekf() - 2.0) < 0.01
    assert ev.step_count == 1


# ── Integration: evaluator driven by simulation data ─────────────────────────

def test_evaluator_with_orbit_simulation():
    """
    Run a short simulation through the orbit + sensor + EKF stack and verify
    that the evaluator's RMSE matches a manual reference computation.
    """
    from app.simulation.orbit import CircularOrbit
    from app.simulation.sensor_sim import SensorSimulator
    from app.ekf.ekf import AttitudeEKF
    from app.triad.triad import TriadEstimator

    orbit = CircularOrbit()
    sim   = SensorSimulator(orbit, rng_seed=0)
    ekf   = AttitudeEKF(sigma_gyro=0.005, sigma_accel=0.05, sigma_mag=0.02)
    triad = TriadEstimator()
    ev    = PerformanceEvaluator(convergence_steps=0)

    manual_ekf_errs   = []
    manual_triad_errs = []

    dt = 0.01
    for i in range(100):
        t    = i * dt
        data = sim.sample(t, dt)
        accel   = np.array([data["ax"], data["ay"], data["az"]])
        mag     = np.array([data["mx"], data["my"], data["mz"]])
        q_true  = data["true_q"]

        ekf.predict(np.array([data["gx"], data["gy"], data["gz"]]), dt)
        ekf.update(accel, mag)
        tr = triad.estimate(accel, mag)

        q_tr = tr["quaternion"] if tr else None
        ev.record(ekf.quaternion, q_tr, q_true)

        manual_ekf_errs.append(np.degrees(error_angle(ekf.quaternion, q_true)))
        if q_tr is not None:
            manual_triad_errs.append(np.degrees(error_angle(q_tr, q_true)))

    # Evaluator RMSE must match manual computation within 1e-6
    summary = ev.summary()
    expected_ekf_rmse = float(np.sqrt(np.mean(np.square(manual_ekf_errs))))
    assert abs(summary["ekf"]["rmse_angular"] - expected_ekf_rmse) < 1e-6
