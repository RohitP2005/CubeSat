# Backend Specification — CubeSat Attitude Estimation Simulator

## Project Name

CubeSat Attitude Determination Service

## Domain

Aerospace GNC / State Estimation / Algorithm Comparison

---

## Technology Stack

| Component | Technology |
|---|---|
| Language | Python 3.11+ |
| API Framework | FastAPI |
| Numerical | NumPy, SciPy |
| Filtering | Custom EKF (Phase 1 delivered), FilterPy optional |
| WebSocket | FastAPI WebSocket / Starlette |
| Containerization | Docker |

**No external database or cache is required.** All simulation state is held in-memory. The simulator is stateless across restarts by design.

---

## Objective

Generate synthetic CubeSat rotational motion, produce realistic sensor measurements, estimate attitude using both TRIAD and EKF, evaluate algorithm performance, and stream all outputs to the React dashboard via WebSocket.

---

## Functional Requirements

### BE-01 — Spacecraft Dynamics Simulator

Generate deterministic ground-truth attitude trajectory.

Inputs:

```json
{
  "angular_velocity": { "x": 0.01, "y": 0.02, "z": 0.03 },
  "duration_s": 60.0,
  "dt": 0.01
}
```

Outputs per step:

```json
{
  "true_quaternion": [q0, q1, q2, q3],
  "true_angular_velocity": [wx, wy, wz]
}
```

The default trajectory is a circular orbit attitude profile (nadir-pointing with slow sinusoidal tumble) provided by `simulation/orbit.py` (delivered in Phase 1).

---

### BE-02 — Sensor Simulator

Generate noisy IMU and magnetometer measurements from the ground-truth trajectory.

#### Gyroscope

```
ω_measured = ω_true + bias + Gaussian(0, σ_g)
bias drifts as random walk: σ_b per √s
```

#### Accelerometer

```
a_measured = R(q_true) · g_ref + Gaussian(0, σ_a)
```

#### Magnetometer

```
m_measured = R(q_true) · b_ref + Gaussian(0, σ_m)
```

Output packet:

```json
{
  "timestamp": 0.0,
  "gx": 0.01, "gy": 0.02, "gz": 0.03,
  "ax": 0.1,  "ay": 0.2,  "az": 9.8,
  "mx": 25.4, "my": -12.1, "mz": 42.3
}
```

Default noise parameters:

| Sensor | Parameter | Default |
|---|---|---|
| Gyroscope | σ_g | 0.005 rad/s |
| Gyroscope bias | σ_b | 1e-5 rad/s/√Hz |
| Accelerometer | σ_a | 0.05 (normalized) |
| Magnetometer | σ_m | 0.02 (normalized) |

---

### BE-03 — TRIAD Algorithm

Deterministic single-shot attitude determination using two reference vector pairs.

**Input vectors**:
- Body measurement 1: normalized accelerometer reading
- Inertial reference 1: gravity reference vector `g_ref = [0, 0, 1]`
- Body measurement 2: normalized magnetometer reading
- Inertial reference 2: normalized magnetic field reference `b_ref`

**Method**: Construct two triad basis sets (one from observations, one from references) and form the DCM as the product.

**Output**:

```json
{
  "algorithm": "TRIAD",
  "roll": 1.2,
  "pitch": -0.8,
  "yaw": 23.4,
  "quaternion": [q0, q1, q2, q3]
}
```

**Limitations**: No noise filtering; performance degrades when vectors are nearly parallel (|sin θ| < 0.1).

---

### BE-04 — Extended Kalman Filter (EKF)

Recursive attitude estimator. Implemented and tested in Phase 1.

**State vector** (7-state, includes gyroscope bias for improved accuracy):

```
x = [q0, q1, q2, q3, bx, by, bz]
```

**Inputs per cycle**:
- Predict: gyroscope angular velocity `[ωx, ωy, ωz]`
- Update: accelerometer `[ax, ay, az]` + magnetometer `[mx, my, mz]`

**Outputs**:

```json
{
  "algorithm": "EKF",
  "quaternion": [q0, q1, q2, q3],
  "roll": 0.0,
  "pitch": 0.0,
  "yaw": 0.0,
  "covariance_trace": 0.004,
  "bias_estimate": [bx, by, bz]
}
```

Accuracy target: mean angular error < 2° after convergence (validated in Phase 1).

---

### BE-05 — Performance Evaluation

Compare all three attitude representations against ground truth each cycle.

**Metrics per cycle**:

| Metric | Description |
|---|---|
| `angular_error_deg` | `2 * arccos(|q_est · q_true|)` converted to degrees |
| `roll_error_deg` | `|roll_est - roll_true|` (wrapped to ±180°) |
| `pitch_error_deg` | `|pitch_est - pitch_true|` |
| `yaw_error_deg` | `|yaw_est - yaw_true|` (wrapped to ±180°) |

**Aggregate metrics** (computed on demand over full run):

| Metric | Description |
|---|---|
| `rmse_roll` | Root mean square roll error (degrees) |
| `rmse_pitch` | Root mean square pitch error (degrees) |
| `rmse_yaw` | Root mean square yaw error (degrees) |
| `rmse_angular` | Root mean square total angular error |

---

### BE-06 — WebSocket Stream

Broadcast one combined frame per simulation cycle (default 100 Hz) to all connected clients.

**Attitude stream** `/ws/attitude`:

```json
{
  "timestamp": 1.23,
  "ground_truth": {
    "quaternion": [q0, q1, q2, q3],
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0
  },
  "triad": {
    "quaternion": [q0, q1, q2, q3],
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "angular_error_deg": 0.0
  },
  "ekf": {
    "quaternion": [q0, q1, q2, q3],
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
    "angular_error_deg": 0.0,
    "covariance_trace": 0.0
  }
}
```

**Telemetry stream** `/ws/telemetry`:

```json
{
  "timestamp": 1.23,
  "gx": 0.0, "gy": 0.0, "gz": 0.0,
  "ax": 0.0, "ay": 0.0, "az": 0.0,
  "mx": 0.0, "my": 0.0, "mz": 0.0
}
```

---

### BE-07 — REST API

```
GET   /simulation/status          — current run state and step count
GET   /attitude/current           — latest attitude frame (all three algorithms)
GET   /performance/summary        — aggregate RMSE and error stats for current run
POST  /simulation/start           — begin simulation loop
POST  /simulation/stop            — pause simulation loop
POST  /simulation/reset           — reset to t=0, reinitialize EKF
POST  /simulation/configure       — update noise params, dt, angular velocity profile
```

---

## Internal Service Architecture

```
SimulationEngine  (owns the loop)
  │
  ├── DynamicsSimulator     → true_q, true_omega
  ├── SensorSimulator       → gyro, accel, mag
  ├── TriadEstimator        → triad_q, triad_euler
  ├── AttitudeEKF           → ekf_q, ekf_euler, cov
  └── PerformanceEvaluator  → per-cycle errors
          │
          ▼
    WebSocketManager        → broadcasts combined frame
          │
          ▼
    FastAPI REST routes      → expose current state + aggregates
```

---

## Non-Functional Requirements

| Requirement | Target |
|---|---|
| Simulation loop rate | 100 Hz (configurable 10–200 Hz) |
| EKF attitude accuracy | < 2° mean angular error (post-convergence) |
| WebSocket broadcast lag | < 50 ms per cycle |
| EKF filter stability | Stable over 10-minute continuous run |
| TRIAD singularity handling | Graceful fallback when vectors nearly parallel |
