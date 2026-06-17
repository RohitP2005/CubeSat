#!/usr/bin/env python3
"""
EKF validation script — Phase 1.

Runs a 60-second closed-loop simulation and reports attitude estimation
accuracy against known ground truth. Saves a four-panel plot.

Usage (from backend/ directory):
    python -m scripts.validate_ekf
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from app.ekf.ekf import AttitudeEKF
from app.ekf.quaternion import to_euler, error_angle, normalize
from app.simulation.orbit import CircularOrbit
from app.simulation.sensor_sim import SensorSimulator


# ---------------------------------------------------------------------------
# Simulation
# ---------------------------------------------------------------------------

def run_simulation(duration: float = 60.0, dt: float = 0.01) -> dict:
    orbit = CircularOrbit(altitude_km=500, tumble_rate_deg_s=0.1)
    sim   = SensorSimulator(
        orbit,
        sigma_gyro=0.005,
        sigma_accel=0.05,
        sigma_mag=0.02,
        bias_instability=1e-5,
        rng_seed=42,
    )

    # Initialize EKF with a small attitude error (~3°) to test convergence
    q_true_0 = orbit.true_attitude(0.0)
    q_init   = q_true_0 + np.array([0.02, -0.01, 0.01, -0.005])
    q_init   = normalize(q_init)

    ekf = AttitudeEKF(
        q0=q_init,
        sigma_gyro=0.005,
        sigma_gyro_bias=1e-4,
        sigma_accel=0.05,
        sigma_mag=0.02,
    )

    n_steps = int(duration / dt)
    times      = np.empty(n_steps)
    roll_est   = np.empty(n_steps); pitch_est   = np.empty(n_steps); yaw_est   = np.empty(n_steps)
    roll_true  = np.empty(n_steps); pitch_true  = np.empty(n_steps); yaw_true  = np.empty(n_steps)
    errors_deg = np.empty(n_steps)
    cov_trace  = np.empty(n_steps)

    t = 0.0
    for i in range(n_steps):
        data = sim.sample(t, dt)

        gyro_meas  = np.array([data["gx"], data["gy"], data["gz"]])
        accel_meas = np.array([data["ax"], data["ay"], data["az"]])
        mag_meas   = np.array([data["mx"], data["my"], data["mz"]])

        ekf.predict(gyro_meas, dt)
        ekf.update(accel_meas, mag_meas)

        r_e, p_e, y_e = (np.degrees(v) for v in to_euler(ekf.quaternion))
        r_t, p_t, y_t = (np.degrees(v) for v in to_euler(data["true_q"]))

        times[i]      = t
        roll_est[i]   = r_e;  pitch_est[i]   = p_e;  yaw_est[i]   = y_e
        roll_true[i]  = r_t;  pitch_true[i]  = p_t;  yaw_true[i]  = y_t
        errors_deg[i] = np.degrees(error_angle(ekf.quaternion, data["true_q"]))
        cov_trace[i]  = ekf.covariance_trace

        t += dt

    return dict(
        times=times,
        roll_est=roll_est,   pitch_est=pitch_est,   yaw_est=yaw_est,
        roll_true=roll_true, pitch_true=pitch_true, yaw_true=yaw_true,
        errors_deg=errors_deg,
        cov_trace=cov_trace,
    )


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def _wrap(diff: np.ndarray) -> np.ndarray:
    return (diff + 180.0) % 360.0 - 180.0

def compute_metrics(data: dict, skip_s: float = 5.0, dt: float = 0.01) -> dict:
    skip = int(skip_s / dt)
    rmse_roll  = float(np.sqrt(np.mean(_wrap(data["roll_est"][skip:]  - data["roll_true"][skip:])  ** 2)))
    rmse_pitch = float(np.sqrt(np.mean(_wrap(data["pitch_est"][skip:] - data["pitch_true"][skip:]) ** 2)))
    rmse_yaw   = float(np.sqrt(np.mean(_wrap(data["yaw_est"][skip:]   - data["yaw_true"][skip:])   ** 2)))
    mean_err   = float(np.mean(data["errors_deg"][skip:]))
    max_err    = float(np.max(data["errors_deg"][skip:]))
    return dict(
        rmse_roll=rmse_roll, rmse_pitch=rmse_pitch, rmse_yaw=rmse_yaw,
        mean_err=mean_err, max_err=max_err,
    )


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_results(data: dict, out_path: Path) -> None:
    t = data["times"]
    fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
    fig.suptitle("CubeSat EKF Attitude Estimation — Phase 1 Validation", fontsize=13)

    angle_cfg = [
        ("roll_est",  "roll_true",  "Roll (deg)"),
        ("pitch_est", "pitch_true", "Pitch (deg)"),
        ("yaw_est",   "yaw_true",   "Yaw (deg)"),
    ]
    for ax, (key_e, key_t, label) in zip(axes[:3], angle_cfg):
        ax.plot(t, data[key_t], "k-",  lw=1.5, label="Ground Truth", alpha=0.8)
        ax.plot(t, data[key_e], "r--", lw=1.0, label="EKF Estimate")
        ax.set_ylabel(label)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(True, alpha=0.3)

    axes[3].plot(t, data["errors_deg"], "b-", lw=0.8, label="Angular Error (deg)")
    axes[3].axhline(2.0, color="r", ls="--", lw=1.2, label="2° accuracy target")
    axes[3].set_ylabel("Error (deg)")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="upper right", fontsize=8)
    axes[3].grid(True, alpha=0.3)
    axes[3].set_ylim(bottom=0)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"  Plot saved → {out_path}")
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    print("=" * 50)
    print("  CubeSat EKF — Phase 1 Validation")
    print("=" * 50)
    print("Running 60-second simulation at 100 Hz...")

    data    = run_simulation(duration=60.0, dt=0.01)
    metrics = compute_metrics(data, skip_s=5.0, dt=0.01)

    print("\nResults (after 5 s convergence window):")
    print(f"  RMSE Roll  : {metrics['rmse_roll']:.3f} deg")
    print(f"  RMSE Pitch : {metrics['rmse_pitch']:.3f} deg")
    print(f"  RMSE Yaw   : {metrics['rmse_yaw']:.3f} deg")
    print(f"  Mean Error : {metrics['mean_err']:.3f} deg")
    print(f"  Max Error  : {metrics['max_err']:.3f} deg")
    print(f"  Target     : < 2.0 deg mean")

    status = "PASS" if metrics["mean_err"] < 2.0 else "FAIL"
    print(f"\n  Status: {status}")

    out = Path(__file__).parent / "ekf_validation.png"
    plot_results(data, out)
    print("Done.")


if __name__ == "__main__":
    main()
