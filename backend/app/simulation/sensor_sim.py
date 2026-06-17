"""
Sensor simulator for IMU and magnetometer measurements (BE-02).

Produces physically scaled sensor outputs:
  - Gyroscope:     rad/s  (Gaussian noise + random-walk bias drift)
  - Accelerometer: m/s²   (scaled by G_EARTH = 9.81; EKF normalises internally)
  - Magnetometer:  µT     (scaled by B_EARTH = 50.0; EKF normalises internally)

Output format matches the BE-02 API packet spec and the /ws/telemetry payload.

All noise is reproducible via rng_seed for test determinism.
"""

import numpy as np
from ..ekf.quaternion import to_dcm

# Physical scaling constants
G_EARTH: float = 9.81   # m/s²  — standard gravity magnitude
B_EARTH: float = 50.0   # µT    — representative LEO total field strength


class SensorSimulator:
    """
    Generates one BE-02 sensor packet per call to sample(t, dt).

    sigma_accel and sigma_mag are kept in normalised units (same scale as the
    EKF measurement noise R matrix).  Physical output is obtained by multiplying
    the normalised signal by G_EARTH / B_EARTH respectively.

    Return dict keys:
      timestamp               — simulation time [s]
      gx, gy, gz              — gyroscope [rad/s]
      ax, ay, az              — accelerometer [m/s²]
      mx, my, mz              — magnetometer [µT]
      true_omega              — ground-truth angular velocity np.ndarray [rad/s]
      true_q                  — ground-truth quaternion np.ndarray
    """

    def __init__(
        self,
        orbit,
        sigma_gyro: float = 0.005,
        sigma_accel: float = 0.05,    # normalised; physical ≈ 0.49 m/s²
        sigma_mag: float = 0.02,      # normalised; physical ≈ 1.0 µT
        initial_bias: np.ndarray | None = None,
        bias_instability: float = 1e-5,
        rng_seed: int = 42,
    ):
        self._orbit            = orbit
        self._sigma_gyro       = sigma_gyro
        self._sigma_accel      = sigma_accel
        self._sigma_mag        = sigma_mag
        self._bias_instability = bias_instability
        self._rng              = np.random.default_rng(rng_seed)

        self._bias = (
            initial_bias.copy()
            if initial_bias is not None
            else np.array([0.002, -0.001, 0.0015])
        )

    @property
    def true_bias(self) -> np.ndarray:
        """Current true gyroscope bias [rad/s]."""
        return self._bias.copy()

    def sample(self, t: float, dt: float) -> dict:
        """
        Generate one noisy sensor packet at time t.

        The EKF normalises accel and mag internally, so physical scaling
        (G_EARTH, B_EARTH) does not affect estimation accuracy — it only
        ensures the API payload contains realistic physical values.
        """
        q_true     = self._orbit.true_attitude(t)
        omega_true = self._orbit.true_angular_velocity(t)
        R          = to_dcm(q_true)   # inertial → body

        # Gyroscope: true rate + drifting bias + white noise [rad/s]
        bias_step   = self._rng.normal(0.0, self._bias_instability * np.sqrt(dt), 3)
        self._bias += bias_step
        gyro = omega_true + self._bias + self._rng.normal(0.0, self._sigma_gyro, 3)

        # Accelerometer: gravity rotated to body frame, scaled to m/s² + noise
        g_inertial   = np.array([0.0, 0.0, 1.0])          # normalised gravity (down)
        accel_norm   = R @ g_inertial + self._rng.normal(0.0, self._sigma_accel, 3)
        accel        = accel_norm * G_EARTH                 # [m/s²]

        # Magnetometer: reference field rotated to body frame, scaled to µT + noise
        b_inertial   = self._orbit.magnetic_field_inertial()
        mag_norm     = R @ b_inertial + self._rng.normal(0.0, self._sigma_mag, 3)
        mag          = mag_norm * B_EARTH                   # [µT]

        return {
            "timestamp":   t,
            "gx": float(gyro[0]),  "gy": float(gyro[1]),  "gz": float(gyro[2]),
            "ax": float(accel[0]), "ay": float(accel[1]), "az": float(accel[2]),
            "mx": float(mag[0]),   "my": float(mag[1]),   "mz": float(mag[2]),
            "true_omega":  omega_true,
            "true_q":      q_true,
        }
