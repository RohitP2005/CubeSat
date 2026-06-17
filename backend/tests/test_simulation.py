"""
Unit tests for the simulation modules: CircularOrbit and SensorSimulator.
Run: pytest tests/test_simulation.py -v
"""

import numpy as np
import pytest
from app.simulation.orbit import CircularOrbit, orbital_period
from app.simulation.sensor_sim import SensorSimulator, G_EARTH, B_EARTH


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _default_orbit() -> CircularOrbit:
    return CircularOrbit(altitude_km=500, tumble_rate_deg_s=0.1)

def _default_sim(orbit: CircularOrbit | None = None) -> SensorSimulator:
    if orbit is None:
        orbit = _default_orbit()
    return SensorSimulator(orbit, rng_seed=0)


# ---------------------------------------------------------------------------
# orbital_period
# ---------------------------------------------------------------------------

def test_orbital_period_iss_altitude():
    """ISS at ~400 km should have period close to 92 minutes."""
    T = orbital_period(400)
    assert 5400 < T < 5600   # 90–93 min in seconds

def test_orbital_period_increases_with_altitude():
    assert orbital_period(400) < orbital_period(500) < orbital_period(800)


# ---------------------------------------------------------------------------
# CircularOrbit.true_attitude
# ---------------------------------------------------------------------------

def test_orbit_attitude_is_unit_quaternion():
    orbit = _default_orbit()
    for t in [0.0, 1.0, 10.0, 100.0, 500.0]:
        q = orbit.true_attitude(t)
        assert abs(np.linalg.norm(q) - 1.0) < 1e-10, f"norm != 1 at t={t}"

def test_orbit_attitude_changes_over_time():
    orbit = _default_orbit()
    q0 = orbit.true_attitude(0.0)
    q1 = orbit.true_attitude(10.0)
    assert not np.allclose(q0, q1)   # attitude must change

def test_orbit_attitude_is_deterministic():
    orbit = _default_orbit()
    assert np.allclose(orbit.true_attitude(5.0), orbit.true_attitude(5.0))


# ---------------------------------------------------------------------------
# CircularOrbit.true_angular_velocity
# ---------------------------------------------------------------------------

def test_orbit_angular_velocity_magnitude():
    """For a slow tumble orbit the angular rate should be small (< 0.01 rad/s typical)."""
    orbit = _default_orbit()
    for t in [0.0, 30.0, 100.0]:
        omega = orbit.true_angular_velocity(t)
        assert np.linalg.norm(omega) < 0.02, f"|ω| = {np.linalg.norm(omega):.4f} at t={t}"

def test_orbit_angular_velocity_integrates_consistently():
    """
    Integrating ω over a short interval must approximately match the
    attitude change given by the kinematic equation.
    """
    from app.ekf.kinematics import propagate
    orbit = _default_orbit()
    dt = 0.001
    t  = 5.0

    q_true_now  = orbit.true_attitude(t)
    q_true_next = orbit.true_attitude(t + dt)
    omega       = orbit.true_angular_velocity(t)

    q_integrated = propagate(q_true_now, omega, dt)

    from app.ekf.quaternion import error_angle
    err = np.degrees(error_angle(q_integrated, q_true_next))
    assert err < 0.01, f"Integration error too large: {err:.4f} deg"


# ---------------------------------------------------------------------------
# CircularOrbit.state
# ---------------------------------------------------------------------------

def test_orbit_state_has_required_keys():
    orbit  = _default_orbit()
    result = orbit.state(0.0)
    assert "true_quaternion"       in result
    assert "true_angular_velocity" in result

def test_orbit_state_quaternion_matches_true_attitude():
    orbit = _default_orbit()
    t = 7.3
    assert np.allclose(orbit.state(t)["true_quaternion"], orbit.true_attitude(t))

def test_orbit_state_omega_matches_true_angular_velocity():
    orbit = _default_orbit()
    t = 7.3
    assert np.allclose(orbit.state(t)["true_angular_velocity"], orbit.true_angular_velocity(t))


# ---------------------------------------------------------------------------
# CircularOrbit.magnetic_field_inertial
# ---------------------------------------------------------------------------

def test_mag_field_is_unit_vector():
    orbit = _default_orbit()
    b = orbit.magnetic_field_inertial()
    assert abs(np.linalg.norm(b) - 1.0) < 1e-12


# ---------------------------------------------------------------------------
# SensorSimulator output format (BE-02)
# ---------------------------------------------------------------------------

EXPECTED_KEYS = {"timestamp", "gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz",
                 "true_omega", "true_q"}

def test_sensor_output_has_all_keys():
    sim  = _default_sim()
    data = sim.sample(0.0, 0.01)
    assert EXPECTED_KEYS.issubset(data.keys()), f"Missing keys: {EXPECTED_KEYS - data.keys()}"

def test_sensor_scalar_fields_are_floats():
    sim  = _default_sim()
    data = sim.sample(0.0, 0.01)
    for key in ("gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz", "timestamp"):
        assert isinstance(data[key], float), f"{key} should be float, got {type(data[key])}"

def test_sensor_true_q_is_unit_quaternion():
    sim  = _default_sim()
    data = sim.sample(5.0, 0.01)
    assert abs(np.linalg.norm(data["true_q"]) - 1.0) < 1e-10


# ---------------------------------------------------------------------------
# Physical unit checks
# ---------------------------------------------------------------------------

def test_sensor_accel_magnitude_near_g():
    """Accelerometer magnitude should be close to G_EARTH (9.81 m/s²) on average."""
    orbit = _default_orbit()
    sim   = SensorSimulator(orbit, sigma_accel=0.001, rng_seed=1)  # tiny noise
    mags  = []
    for i in range(100):
        data = sim.sample(i * 0.01, 0.01)
        mags.append(np.linalg.norm([data["ax"], data["ay"], data["az"]]))
    mean_mag = np.mean(mags)
    assert abs(mean_mag - G_EARTH) < 0.1, f"Mean |accel| = {mean_mag:.3f}, expected ≈ {G_EARTH}"

def test_sensor_mag_magnitude_near_b_earth():
    """Magnetometer magnitude should be close to B_EARTH (50 µT) on average."""
    orbit = _default_orbit()
    sim   = SensorSimulator(orbit, sigma_mag=0.001, rng_seed=2)    # tiny noise
    mags  = []
    for i in range(100):
        data = sim.sample(i * 0.01, 0.01)
        mags.append(np.linalg.norm([data["mx"], data["my"], data["mz"]]))
    mean_mag = np.mean(mags)
    assert abs(mean_mag - B_EARTH) < 1.0, f"Mean |mag| = {mean_mag:.3f}, expected ≈ {B_EARTH}"

def test_sensor_gyro_units_are_rad_per_s():
    """Gyro output should be in rad/s — typical values well below 1 rad/s for slow tumble."""
    sim  = _default_sim()
    data = sim.sample(0.0, 0.01)
    gyro_mag = np.linalg.norm([data["gx"], data["gy"], data["gz"]])
    assert gyro_mag < 0.1, f"|gyro| = {gyro_mag:.4f} — unexpectedly large (wrong units?)"


# ---------------------------------------------------------------------------
# Noise and bias behaviour
# ---------------------------------------------------------------------------

def test_sensor_bias_drifts_over_time():
    """True gyroscope bias must change over a long run (random walk)."""
    orbit  = _default_orbit()
    sim    = SensorSimulator(orbit, bias_instability=1e-3, rng_seed=3)
    bias_0 = sim.true_bias.copy()
    for i in range(1000):
        sim.sample(i * 0.01, 0.01)
    bias_1 = sim.true_bias
    assert not np.allclose(bias_0, bias_1), "Bias did not drift — bias_instability may be zero"

def test_sensor_zero_noise_gives_perfect_measurements():
    """With sigma=0 and no bias, gyro must equal true angular velocity exactly."""
    orbit = _default_orbit()
    sim   = SensorSimulator(
        orbit,
        sigma_gyro=0.0,
        sigma_accel=0.0,
        sigma_mag=0.0,
        initial_bias=np.zeros(3),
        bias_instability=0.0,
        rng_seed=0,
    )
    t = 5.0
    data  = sim.sample(t, 0.01)
    omega_true = orbit.true_angular_velocity(t)
    gyro_meas  = np.array([data["gx"], data["gy"], data["gz"]])
    assert np.allclose(gyro_meas, omega_true, atol=1e-10)

def test_sensor_reproducible_with_same_seed():
    """Two simulators with the same seed must produce identical samples."""
    orbit = _default_orbit()
    sim_a = SensorSimulator(orbit, rng_seed=99)
    sim_b = SensorSimulator(orbit, rng_seed=99)
    for t in [0.0, 0.01, 0.02]:
        a = sim_a.sample(t, 0.01)
        b = sim_b.sample(t, 0.01)
        for key in ("gx", "gy", "gz", "ax", "ay", "az", "mx", "my", "mz"):
            assert a[key] == b[key], f"key={key} differs at t={t}"
