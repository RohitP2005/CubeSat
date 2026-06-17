# CubeSat ADCS — Technical Concepts Reference

A complete reference for the mathematical, algorithmic, and architectural concepts used in this project. Organized from fundamentals to implementation details.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Attitude Representation](#2-attitude-representation)
   - 2.1 [Quaternions](#21-quaternions)
   - 2.2 [Direction Cosine Matrix (DCM)](#22-direction-cosine-matrix-dcm)
   - 2.3 [Euler Angles (ZYX Convention)](#23-euler-angles-zyx-convention)
   - 2.4 [Coordinate Frames](#24-coordinate-frames)
3. [Attitude Determination Algorithms](#3-attitude-determination-algorithms)
   - 3.1 [TRIAD Algorithm](#31-triad-algorithm)
   - 3.2 [Extended Kalman Filter (EKF)](#32-extended-kalman-filter-ekf)
4. [Physics & Kinematics](#4-physics--kinematics)
   - 4.1 [Quaternion Kinematics](#41-quaternion-kinematics)
   - 4.2 [Orbital Mechanics](#42-orbital-mechanics)
   - 4.3 [Earth's Magnetic Field](#43-earths-magnetic-field)
5. [Sensor Simulation](#5-sensor-simulation)
6. [Performance Evaluation](#6-performance-evaluation)
7. [Backend Architecture](#7-backend-architecture)
8. [Frontend Architecture](#8-frontend-architecture)
   - 8.1 [Rendering Stack](#81-rendering-stack)
   - 8.2 [State Management](#82-state-management)
   - 8.3 [Data Flow](#83-data-flow)
   - 8.4 [3D Visualization](#84-3d-visualization)
   - 8.5 [Charts](#85-charts)
9. [Real-Time Communication](#9-real-time-communication)
10. [Three Operating Modes](#10-three-operating-modes)
11. [Key Design Decisions](#11-key-design-decisions)

---

## 1. System Overview

This project is a **CubeSat Attitude Determination and Control System (ADCS) simulator**. It implements two attitude estimation algorithms — TRIAD and EKF — running against a synthetic satellite trajectory, and provides a real-time web dashboard to visualize and compare their performance.

```
┌──────────────────────────────────────────────────────────┐
│                        Backend (Python)                   │
│                                                           │
│  CircularOrbit ──► SensorSim ──► TRIAD ──► EKF           │
│       (ground truth)  (noisy sensors)   (estimation)      │
│                                                           │
│  FastAPI REST + WebSocket broadcast @ 50–100 Hz           │
└──────────────────────────┬───────────────────────────────┘
                           │ WebSocket / HTTP
┌──────────────────────────▼───────────────────────────────┐
│                     Frontend (React)                      │
│                                                           │
│  Zustand stores ◄── WS feeds / demo generator            │
│                                                           │
│  3D viewer (R3F)  │  Plotly charts  │  Metrics tiles      │
└──────────────────────────────────────────────────────────┘
```

**Physical scale:** A 1U CubeSat is a 10 cm × 10 cm × 10 cm satellite. In practice this project simulates its attitude (orientation) only — no translation, no propulsion, no power budgets.

---

## 2. Attitude Representation

Attitude is the orientation of a rigid body relative to a reference frame. Three representations are used throughout the codebase, each with its own advantages.

### 2.1 Quaternions

A quaternion `q = [q₀, q₁, q₂, q₃]` (scalar-first convention) represents a rotation by angle `θ` around unit axis `û = [uₓ, uᵧ, uᵤ]`:

```
q₀ = cos(θ/2)
q₁ = uₓ sin(θ/2)
q₂ = uᵧ sin(θ/2)
q₃ = uᵤ sin(θ/2)
```

**Why quaternions are preferred over Euler angles:**
- No gimbal lock (Euler angles become singular at pitch = ±90°)
- Smooth interpolation (SLERP)
- Efficient composition: `q_total = q₁ ⊗ q₂` (Hamilton product)
- Compact (4 numbers vs 9 for DCM)

**Constraint:** `|q| = 1` always. Every update step must re-normalize.

**Double-cover:** `q` and `-q` represent the same rotation. The EKF always flips to the "positive hemisphere" (`q₀ ≥ 0`) to avoid sign discontinuities.

**Key operations used in the codebase:**

*Normalization:*
```
q_norm = q / √(q₀² + q₁² + q₂² + q₃²)
```

*Hamilton product (composes two rotations):*
```
(p ⊗ q)₀ = p₀q₀ − p₁q₁ − p₂q₂ − p₃q₃
(p ⊗ q)₁ = p₀q₁ + p₁q₀ + p₂q₃ − p₃q₂
(p ⊗ q)₂ = p₀q₂ − p₁q₃ + p₂q₀ + p₃q₁
(p ⊗ q)₃ = p₀q₃ + p₁q₂ − p₂q₁ + p₃q₀
```

*Conjugate (inverse for unit quaternions):*
```
q* = [q₀, −q₁, −q₂, −q₃]
```

*Error quaternion between two attitudes:*
```
q_err = q_truth* ⊗ q_estimated
angular_error = 2 arccos(clip(|q_err₀|, −1, 1))    [radians]
```

### 2.2 Direction Cosine Matrix (DCM)

The DCM `R` is a 3×3 orthogonal matrix (`RRᵀ = I`, `det(R) = 1`) that rotates inertial vectors into the body frame:

```
v_body = R · v_inertial
```

Constructed from the quaternion `q = [q₀, q₁, q₂, q₃]`:

```
R = | q₀²+q₁²−q₂²−q₃²    2(q₁q₂−q₀q₃)      2(q₁q₃+q₀q₂)  |
    | 2(q₁q₂+q₀q₃)        q₀²−q₁²+q₂²−q₃²   2(q₂q₃−q₀q₁)  |
    | 2(q₁q₃−q₀q₂)        2(q₂q₃+q₀q₁)      q₀²−q₁²−q₂²+q₃² |
```

The DCM is used extensively in the EKF measurement model to predict what the sensors should read given the current quaternion estimate.

**Inverse DCM → Quaternion (Shepperd method):** Branches on the sign of `trace(R)` to avoid dividing by values near zero — a numerical stability requirement.

### 2.3 Euler Angles (ZYX Convention)

Three sequential rotations: first Yaw (Z), then Pitch (Y), then Roll (X). This is the aerospace standard.

Extracted from a quaternion:

```
roll  = atan2( 2(q₀q₁ + q₂q₃),  1 − 2(q₁² + q₂²) )
pitch = arcsin( clip( 2(q₀q₂ − q₃q₁), −1, 1 ) )
yaw   = atan2( 2(q₀q₃ + q₁q₂),  1 − 2(q₂² + q₃²) )
```

The `clip` on pitch prevents `arcsin` domain errors from floating-point noise.

**Limitations:** Euler angles are used only for display (charts, readouts). All internal math uses quaternions to avoid gimbal lock.

### 2.4 Coordinate Frames

| Frame | Symbol | Definition |
|-------|--------|------------|
| Inertial (ECI) | I | Earth-Centered Inertial — fixed to distant stars |
| Body | B | Fixed to the CubeSat — X/Y/Z = satellite axes |
| LVLH | L | Local Vertical Local Horizontal — Z points nadir |

**Reference vectors (inertial frame, normalized):**
```
g_ref = [0, 0, 1]              ← gravity (downward / nadir)
b_ref = [0.309, 0, 0.951]      ← Earth's magnetic field at ~500 km LEO
                                  (72° inclination, simplified dipole)
```

These are what the sensors *should* read (ideally, without noise) when the satellite is at a known attitude.

---

## 3. Attitude Determination Algorithms

### 3.1 TRIAD Algorithm

TRIAD (Two-Reference-vector-based Inertial Attitude Determination) is the simplest deterministic attitude estimator. It requires exactly two non-parallel vector observations.

**Inputs:**
- `a_body` — accelerometer reading (body-frame gravity direction)
- `m_body` — magnetometer reading (body-frame B-field direction)

**Algorithm (step by step):**

```
Step 1 — Normalize measurements
  b₁ = a_body / |a_body|    (primary, more reliable)
  b₂ = m_body / |m_body|    (secondary)

Step 2 — Build observation triad (orthonormal basis from body measurements)
  t₁ = b₁
  t₂ = normalize(b₁ × b₂)
  t₃ = t₁ × t₂
  M_obs = [t₁ | t₂ | t₃]     (3×3 matrix, columns are triad vectors)

Step 3 — Build reference triad (same construction from known inertial vectors)
  s₁ = g_ref
  s₂ = normalize(g_ref × b_ref)
  s₃ = s₁ × s₂
  M_ref = [s₁ | s₂ | s₃]

Step 4 — Compute rotation matrix
  R = M_obs · M_refᵀ          (maps inertial → body)

Step 5 — Convert
  q = dcm_to_quaternion(R)
```

**Why it works:** If `b₁ = R · s₁` and `b₂ ≈ R · s₂`, then constructing the same orthonormal frame from both sides and relating them yields R directly.

**Properties:**
- No iterations, no memory — instantaneous
- The primary vector is matched *exactly*; the secondary only approximately
- Returns `None` when `|b₁ × b₂| < 0.1` (vectors too parallel → singular geometry)
- In live mode: the magnetic reference `b_ref` is updated each tick with the real IGRF field at the satellite's current position

**Limitation:** TRIAD is sensitive to sensor noise (each measurement is used raw) and has no state history. High-frequency noise maps directly to attitude noise. This is why the EKF consistently outperforms it.

### 3.2 Extended Kalman Filter (EKF)

The EKF is a recursive state estimator. It maintains a *belief* about the current state (mean + covariance) and updates it with each sensor measurement. It is the industry-standard algorithm for attitude estimation.

#### State Vector (7 components)

```
x = [q₀, q₁, q₂, q₃, bₓ, bᵧ, bᵤ]ᵀ

  q ∈ ℝ⁴   — unit quaternion (attitude)
  b ∈ ℝ³   — gyroscope bias [rad/s], slowly-drifting systematic error
```

Estimating bias alongside attitude is why the EKF needs 7 states rather than 4. The gyro is the primary *propagation* sensor; its accumulated bias is the main long-term error source. The accelerometer and magnetometer *correct* the drift.

#### Uncertainty (Covariance Matrix)

```
P ∈ ℝ⁷ˣ⁷   — state error covariance
```

`P[i,i]` is the variance of state `i`. Off-diagonal elements capture correlations. The EKF starts with a high P (high uncertainty) and P shrinks as measurements accumulate — this is called *filter convergence*.

#### Predict Step

Between measurements, the state evolves using only the gyroscope:

```
1. Remove estimated bias:
   ω_corrected = ω_measured − b

2. Propagate quaternion (Euler integration):
   q_new = normalize(q + 0.5 · dt · Ω(ω_corrected) · q)

   where Ω(ω) is the 4×4 omega matrix:
       [ 0    −ωₓ  −ωᵧ  −ωᵤ ]
       [ ωₓ    0    ωᵤ  −ωᵧ ]
       [ ωᵧ  −ωᵤ   0    ωₓ ]
       [ ωᵤ   ωᵧ  −ωₓ   0  ]

3. Bias is modeled as random walk (no prediction, stays at b):
   b_new = b

4. Propagate covariance:
   F = process_jacobian(q, ω_corrected, dt)    ← linearization
   P_new = F · P · Fᵀ + Q                      ← Q = process noise
```

The **process Jacobian F** (7×7) captures how small perturbations in the current state map forward in time:

```
F = [ I₄ + 0.5·dt·Ω(ω)     −0.5·dt·Ξ(q) ]
    [ 0₃ₓ₄                   I₃           ]

Upper-left  (4×4): quaternion state transition (linear in ω)
Upper-right (4×3): how bias error affects quaternion evolution
Lower-left  (3×4): zeros (bias not directly driven by attitude)
Lower-right (3×3): identity (bias is random-walk, unchanged)
```

where `Ξ(q)` is the 4×3 kinematic coupling matrix.

#### Update Step

The accelerometer and magnetometer provide corrections:

```
1. Predict measurements from current state:
   h_accel = R(q) · g_ref          ← predicted accel direction in body frame
   h_mag   = R(q) · b_ref          ← predicted mag direction in body frame

2. Stack into predicted measurement vector h:
   h = [h_accel; h_mag]   ∈ ℝ⁶

3. Normalize actual measurements:
   z = [a_body/|a_body|; m_body/|m_body|]   ∈ ℝ⁶

4. Innovation (difference between measured and predicted):
   y = z − h

5. Measurement Jacobian H (6×7):
   H = [ ∂h_accel/∂q    0₃ₓ₃ ]    ← 3×7 rows for accel
       [ ∂h_mag/∂q      0₃ₓ₃ ]    ← 3×7 rows for mag
   (Bias has no direct measurement effect → right 3 columns = 0)

6. Innovation covariance:
   S = H · P · Hᵀ + R_noise     ← R_noise: measurement noise covariance

7. Kalman gain:
   K = P · Hᵀ · S⁻¹             ← 7×6 matrix

8. State update:
   x_new = x + K · y
   q_new = normalize(x_new[0:4])  ← re-enforce unit quaternion

9. Covariance update (Joseph form for numerical stability):
   P_new = (I − K·H) · P · (I − K·H)ᵀ + K · R_noise · Kᵀ
```

#### Why the Joseph Form?

The standard covariance update `P = (I − KH)P` can produce slightly non-symmetric or non-positive-definite matrices due to floating-point rounding. The Joseph form `(I − KH)P(I − KH)ᵀ + KRKᵀ` is algebraically equivalent but numerically guaranteed to stay positive semi-definite.

#### Gyro Bias Estimation

The bias states `b` are never directly measured — they are inferred from the *residual drift* in the quaternion over time. When the quaternion slowly drifts in a direction inconsistent with the magnetometer/accelerometer, the Kalman gain assigns some of that discrepancy to the bias estimate. Over ~100 steps (1 second at 100 Hz), the filter converges to a good bias estimate.

---

## 4. Physics & Kinematics

### 4.1 Quaternion Kinematics

How attitude changes over time given angular velocity `ω = [ωₓ, ωᵧ, ωᵤ]` [rad/s]:

```
q̇ = 0.5 · Ω(ω) · q
```

This is the fundamental quaternion kinematic equation. It relates the time derivative of the quaternion to the current quaternion and the body angular velocity.

Integrated with first-order Euler:

```
q(t + dt) = normalize(q(t) + 0.5 · dt · Ω(ω) · q(t))
```

**Dual form** (used in the Jacobian):

```
q̇ = 0.5 · Ξ(q) · ω

where Ξ(q) ∈ ℝ⁴ˣ³ is:
  Ξ(q) = [ −q₁  −q₂  −q₃ ]
          [  q₀  −q₃   q₂ ]
          [  q₃   q₀  −q₁ ]
          [ −q₂   q₁   q₀ ]
```

This form makes the relationship between angular velocity and quaternion derivative linear — useful for deriving the Jacobians.

**Recovering angular velocity from quaternions:**

When ground truth quaternions are known at discrete times, angular velocity is recovered via central differences:

```
ω ≈ 2 · Ξ(q)ᵀ · (q(t+δ) − q(t−δ)) / (2δ)
```

### 4.2 Orbital Mechanics

The simulation uses a simplified **circular Keplerian orbit** at 500 km altitude.

**Orbital period:**

```
T = 2π √(r³ / μ)

where r = R_Earth + altitude = 6371 + 500 = 6871 km
      μ = 3.986 × 10¹⁴ m³/s²  (Earth's gravitational parameter)
      T ≈ 5676 s ≈ 94.6 minutes
```

**Mean motion:**

```
n = 2π / T   [rad/s]
```

**Ground truth attitude trajectory** (nadir-pointing base + sinusoidal tumble):

```
yaw(t)   = n · t                            (continuous nadir-pointing rotation)
roll(t)  = 0.15 · sin(ω_roll · t)           (sinusoidal wobble)
pitch(t) = 0.10 · sin(ω_pitch · t + 0.5)   (offset phase)
```

`ω_roll` and `ω_pitch` are scaled versions of `n` multiplied by `tumble_rate_deg_s / 0.1` to allow configurable dynamics.

**Live mode** replaces the synthetic orbit with real TLE (Two-Line Element) propagation via the `sgp4` library, giving the satellite's actual position in ECI coordinates.

### 4.3 Earth's Magnetic Field

**Simplified dipole model** (used in simulation mode):

```
b_inertial = [0.309, 0, 0.951]     (normalized, ~72° inclination LEO)
```

**IGRF dipole approximation** (used in live mode for the current satellite position):

```
B = (B₀ · R_E³ / r³) · (3(m̂·r̂)r̂ − m̂)

where B₀ = 3.12 × 10⁻⁵ T   (surface field strength)
      R_E = 6.371 × 10⁶ m    (Earth radius)
      r   = distance from Earth center
      m̂   = magnetic dipole axis (11.5° tilt, longitude −72°)
      r̂   = unit position vector of satellite
```

This gives ~5% accuracy for low Earth orbit — sufficient for sensor simulation purposes.

---

## 5. Sensor Simulation

The satellite's IMU is simulated with realistic noise and bias models.

### Noise Models

**Gyroscope:**
```
ω_measured = ω_true + b(t) + n_gyro

n_gyro ~ N(0, σ_gyro²)           (white noise, default σ = 0.005 rad/s)
b(t) = random walk:
  b(t+dt) = b(t) + n_drift,  n_drift ~ N(0, σ_drift² · dt)
  σ_drift = 1e−5 rad/s/√s    (bias instability)
```

Gyros accumulate error over time due to bias drift — this is what makes the EKF's bias estimation crucial for long-duration accuracy.

**Accelerometer:**
```
a_measured = normalize( R · g_ref_inertial ) · G_EARTH + n_accel

n_accel ~ N(0, σ_accel²)         (σ = 0.05, normalized units)
Physical: a_physical = a_normalized × 9.81 m/s²
```

The accelerometer measures specific force (gravity in body frame). In free fall, it reads the gravity direction rotated into body coordinates.

**Magnetometer:**
```
m_measured = normalize( R · b_field_inertial ) · B_EARTH + n_mag

n_mag ~ N(0, σ_mag²)             (σ = 0.02, normalized units)
Physical: m_physical = m_normalized × 50.0 µT   (typical LEO field strength)
```

### Reproducibility

All random number generation uses a seeded NumPy RNG (`rng_seed = 42` default). The same seed always produces the same trajectory — essential for reproducible testing and debugging.

---

## 6. Performance Evaluation

The evaluator runs alongside the simulation, tracking estimation error over time.

### Angular Error

For each time step:

```
error_EKF   = angular_error(q_ekf,   q_truth)   [degrees]
error_TRIAD = angular_error(q_triad, q_truth)   [degrees, or NaN if singularity]
```

### Convergence Window

The first 100 steps (~1 second at 100 Hz) are excluded from statistics. During this window the EKF covariance is still collapsing from its initial high uncertainty. Metrics computed from the transient would be artificially pessimistic.

### Metrics

```
RMSE_angular = √( mean(error²) )   over post-convergence window

RMSE_roll    = √( mean(roll_error²) )
RMSE_pitch   = √( mean(pitch_error²) )
RMSE_yaw     = √( mean(yaw_error²) )

improvement_ratio = mean_error_TRIAD / mean_error_EKF
```

A typical result shows the EKF achieving ~0.5–1° RMSE while TRIAD shows ~2–4°, giving an improvement ratio of ~3–6×.

### Covariance Trace

```
trace(P) = P[0,0] + P[1,1] + ... + P[6,6]
```

Plotting `trace(P)` over time shows filter convergence: starts high (uncertain), decays exponentially as measurements are incorporated, eventually plateaus at a steady-state level determined by sensor noise.

---

## 7. Backend Architecture

### Simulation Loop

The core loop runs as an `asyncio` task at 100 Hz (target):

```
while running:
  t_start = now()

  1. orbit.sample(t)          → q_truth, ω_truth, mag_inertial
  2. sensor_sim.sample(...)   → a_body, m_body, ω_gyro (with noise)
  3. triad.estimate(a, m)     → q_triad (or None if singular)
  4. ekf.predict(ω_gyro, dt)  → propagates state + covariance
  5. ekf.update(a, m)         → corrects state with measurements
  6. evaluator.record(...)    → accumulates error statistics
  7. build_attitude_frame()   → packages everything for broadcast
  8. ws_manager.broadcast()   → sends JSON to all connected clients

  sleep(max(0, dt − (now() − t_start)))   ← timing regulation
```

### WebSocket Channels

Two independent channels allow clients to subscribe selectively:

| Channel | Rate | Payload |
|---------|------|---------|
| `/ws/attitude` | 50 Hz | Ground truth + TRIAD + EKF quaternions, Euler angles, angular errors |
| `/ws/telemetry` | 50 Hz | Raw sensor readings (gyro, accel, mag) + timestamp |

### REST API

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/health` | Liveness check |
| GET | `/simulation/status` | Run state, elapsed time, config |
| POST | `/simulation/start` | Begin simulation loop |
| POST | `/simulation/stop` | Halt loop (state preserved) |
| POST | `/simulation/reset` | Reset to t=0 |
| POST | `/simulation/configure` | Change dt, noise, altitude, etc. |
| GET | `/attitude/current` | Latest attitude frame (snapshot) |
| GET | `/performance/summary` | RMSE + improvement ratio |

### Data Conventions

- **Timestamps:** seconds from simulation start (`t`)
- **Quaternion order:** scalar-first `[q₀, q₁, q₂, q₃]` everywhere
- **Angles:** radians internally, degrees only in display/API payloads
- **Run state strings:** uppercase (`"RUNNING"`, `"STOPPED"`, `"RESET"`)

---

## 8. Frontend Architecture

### 8.1 Rendering Stack

| Layer | Technology | Role |
|-------|-----------|------|
| Meta-framework | TanStack Start | SSR, routing, server functions |
| Router | TanStack Router | File-based routing, type-safe links |
| UI | React 19 | Component tree |
| 3D | React Three Fiber + drei | WebGL via Three.js with React bindings |
| Charts | Plotly.js (react-plotly.js) | Interactive 2D plots |
| Styling | Tailwind CSS 4 | Utility-first CSS, OKLCH color tokens |
| UI components | Radix UI (via shadcn) | Accessible primitives |

**TanStack Start and SSR:** The app uses streaming server-side rendering (`renderToReadableStream`). The server renders the HTML shell; React hydrates on the client. This requires that any browser-only library (Plotly, Three.js) be imported only on the client side.

**Why Plotly cannot be statically imported:** `plotly.js` accesses the global `self` (a browser alias for `window`) at module-evaluation time. Importing it in Node.js throws `ReferenceError: self is not defined` before any React component renders. The `PlotlyChart` component works around this with a `useEffect`-deferred import — `useEffect` never runs on the server.

**Why Three.js is excluded from Vite's dep optimization:** On Windows, Vite writes pre-bundled packages to a cache directory. Windows Defender can lock these files mid-write, leaving partial/missing bundle files. Three.js packages are excluded (`optimizeDeps.exclude`) so Vite never writes bundle files for them — they are served as raw ESM directly from `node_modules/`.

### 8.2 State Management

Five Zustand stores hold all client-side state:

```
attitudeStore     — latest attitude frame (single object, replaced each tick)
telemetryStore    — ring buffer of sensor readings (max 6000 samples = 60s)
comparisonStore   — ring buffer of Euler + quaternion history for chart traces
performanceStore  — RMSE summary + covariance trace
simulationStore   — run state, elapsed time, WS connection state, app mode
```

**Why Zustand over Redux:** Zustand stores are plain JavaScript with minimal boilerplate. Each store is a single `create()` call. Components subscribe to exactly the slices they need (`useStore(s => s.field)`) — no provider tree required.

**Ring buffer pattern:** `telemetryStore` and `comparisonStore` use arrays with a max length. When `push()` is called and the buffer is full, the oldest entry is removed with `shift()`. This bounds memory use regardless of how long the simulation runs.

### 8.3 Data Flow

**Demo mode** (no backend required):

```
useDemoFeed (50 Hz, requestAnimationFrame)
  │
  ├─ Generates synthetic q_truth, q_ekf, q_triad
  ├─ Adds realistic noise (EKF: σ=0.01, TRIAD: σ=0.05, 2% singularities)
  ├─ Computes angular errors
  │
  ├─► attitudeStore.set(frame)
  ├─► telemetryStore.push(sensorFrame)
  ├─► comparisonStore.push(historyFrame)
  └─► performanceStore.setSummary(stats)  [updated every 1s]
```

**Live mode** (backend connected):

```
useWebSocket → /ws/attitude  ──► attitudeStore + comparisonStore
useWebSocket → /ws/telemetry ──► telemetryStore

useSimulationStatus (poll every 2s) ──► simulationStore.runState, elapsed

performanceApi.summary() [on-demand] ──► performanceStore.summary
```

**Component subscription pattern:**

```typescript
// Component only re-renders when `latest` changes — not the whole store
const frame = useAttitudeStore((s) => s.latest);
```

Selector functions prevent unnecessary re-renders. Components subscribe to the minimum state they actually use.

### 8.4 3D Visualization

The 3D viewer uses **React Three Fiber (R3F)**, which maps React's component model onto Three.js objects. The canvas auto-resizes, handles device pixel ratio (capped at 2× for performance), and supports mouse-drag orbit controls.

**CubeSat geometry:**

```
Box geometry: 1×1×1 unit (represents 10cm cube)

Face materials:
  ±Z faces → dark navy (#1e3a5f)   — body panels
  ±X faces → slightly lighter navy  — body panels
  ±Y faces → lighter gray-blue     — solar panel faces

Axis indicators (solid variant only):
  +X → red cylinder + cone    [roll axis]
  +Y → green cylinder + cone  [pitch axis]
  +Z → blue cylinder + cone   [yaw axis]
```

**Quaternion interpolation:** The mesh does not jump instantly to the new attitude. Each frame it slerps (spherical linear interpolation) between current and target by factor 0.35:

```typescript
currentQ.slerp(targetQ, 0.35)
```

Slerp follows the shortest arc on the unit quaternion sphere — visually smooth, no gimbal artifacts.

**Quaternion convention difference:** The EKF uses scalar-first `[q₀, q₁, q₂, q₃]`. Three.js uses scalar-last `(x, y, z, w)`. The conversion is:

```typescript
new THREE.Quaternion(q[1], q[2], q[3], q[0])   // [q₀,q₁,q₂,q₃] → (x,y,z,w)
```

**Ghost overlay:** When "show ground-truth ghost" is enabled, a second semi-transparent CubeSat mesh (opacity 0.18) is rendered at the ground truth attitude. Comparing ghost vs solid gives intuitive visual feedback on estimation error.

### 8.5 Charts

All charts go through the `PlotlyChart` wrapper which:
- Defers `react-plotly.js` import to client-side (SSR-safe)
- Applies a consistent dark theme (transparent background, slate text, muted gridlines)
- Enables `useResizeHandler` so charts respond to container resizing
- Hides the Plotly mode bar for a cleaner look

**Chart types and their purpose:**

| Chart | Route | Shows |
|-------|-------|-------|
| `TelemetryChart` | /telemetry | Gyro (rad/s), Accel (m/s²), Mag (µT) — 3 axes per sensor |
| `EulerOverlayChart` | /comparison | Roll/pitch/yaw for Truth (white), TRIAD (amber), EKF (blue) |
| `QuaternionChart` | /comparison | q₀–q₃ components for Truth vs EKF |
| `ErrorChart` | /comparison | Angular error over time + 2° target line |
| `CovarianceChart` | /performance | `trace(P)` on log scale — shows filter convergence |
| `RmseBarChart` | /performance | Grouped bars: TRIAD vs EKF RMSE per axis |

**Windowed display:** Charts show a rolling time window (10/30/60 seconds selectable). The buffer stores up to 6000 samples (~60 seconds at 100 Hz). Slicing is done in the component:

```typescript
const now = buffer.at(-1)?.t ?? 0;
const visible = buffer.filter((f) => f.t >= now - windowSec);
```

**TRIAD gaps:** TRIAD returns `null` when the singularity guard fires (gravity and magnetic vectors too parallel). Plotly renders this as a gap in the line (`connectgaps: false`), which is more honest than interpolating across invalid estimates.

---

## 9. Real-Time Communication

### WebSocket Protocol

Messages are newline-delimited JSON over a persistent TCP connection. The browser's native `WebSocket` API is used (no socket.io or similar).

**Attitude frame payload:**

```json
{
  "t": 12.34,
  "ground_truth": {
    "quaternion": [q0, q1, q2, q3],
    "euler": { "roll": 0.1, "pitch": -0.05, "yaw": 2.3 }
  },
  "triad": {
    "quaternion": [q0, q1, q2, q3],
    "euler": { "roll": 0.11, "pitch": -0.04, "yaw": 2.28 },
    "angular_error_deg": 1.4
  },
  "ekf": {
    "quaternion": [q0, q1, q2, q3],
    "euler": { "roll": 0.101, "pitch": -0.049, "yaw": 2.298 },
    "angular_error_deg": 0.6
  },
  "orbital": null
}
```

`triad` is `null` when a singularity was detected that tick. `orbital` is non-null only in live mode.

### Reconnection Strategy

`useWebSocket` implements exponential backoff:

```
attempt 1: wait 1s
attempt 2: wait 2s
attempt 3: wait 4s
attempt 4: wait 8s
attempt 5: wait 16s
```

After 5 failures the hook stops retrying. The user can trigger a manual reconnect by toggling the demo/live switch or refreshing.

---

## 10. Three Operating Modes

| Mode | Data Source | Backend Required? |
|------|-------------|-------------------|
| **Demo** | Browser-generated synthetic data | No |
| **Simulation** | Backend simulation engine via WebSocket | Yes |
| **Live** | Backend + real TLE + IGRF B-field | Yes + internet (CelesTrak) |

**Demo mode** is the default. It generates plausible attitude trajectories entirely in the browser using `requestAnimationFrame` at 50 Hz. All four stores are populated identically to the live/simulation path — the UI is fully functional with no backend.

**Simulation mode** connects to the FastAPI backend and streams data from the Python simulation loop. The backend runs at 100 Hz but broadcasts at 50 Hz to reduce bandwidth. Configuration (noise levels, altitude, tumble rate) can be changed via the `/simulation/configure` endpoint.

**Live mode** (simulation mode variant): the backend fetches the current TLE for a real satellite (default: ISS, NORAD 25544) from CelesTrak, propagates it with `sgp4`, and computes the IGRF magnetic field at each tick. The satellite's orientation is still simulated (no real attitude telemetry exists) but its orbital position and magnetic environment are real.

---

## 11. Key Design Decisions

### Quaternion scalar-first convention

The codebase consistently uses `[q₀, q₁, q₂, q₃]` (scalar first) rather than the alternative `[q₁, q₂, q₃, q₀]` (Hamilton/scalar-last). This matches MATLAB Aerospace Toolbox and most spacecraft GNC textbooks. The only exception is Three.js (scalar-last), handled by explicit conversion at the boundary.

### Normalizing sensor inputs in the EKF measurement model

Rather than feeding raw accelerometer/magnetometer magnitudes into the EKF, measurements are normalized before computing the innovation `y = z − h`. This decouples the attitude estimate from sensor scale calibration errors and simplifies the measurement noise model (uniform unit-sphere noise rather than physical-unit noise).

### TRIAD as primary reference, not EKF initialization

TRIAD runs every tick independently of the EKF. It is not used to initialize the EKF (the EKF starts from a fixed initial quaternion `[1, 0, 0, 0]`). This allows the dashboard to show TRIAD vs EKF comparison from the very first tick, and lets TRIAD serve as an independent error reference throughout the run.

### Demo feed replicates the full backend data contract

`useDemoFeed.ts` generates data in exactly the same shape as the WebSocket messages from the real backend. All chart components and stores are unaware of whether data came from the demo generator or the WebSocket. This means the UI is always fully testable without running Python.

### SSR-compatible component boundaries

TanStack Start renders on the server. Browser-only libraries (Plotly, Three.js) would throw in Node.js. The boundary is enforced at the component level:
- `PlotlyChart`: `useEffect` defers `import("react-plotly.js")` to post-mount (client only)
- `AttitudeScene`: imported from `index.tsx`, which uses `codeSplitGroupings: []` to keep Three.js out of SSR code paths on the server

### Covariance update via Joseph form

The standard update `P = (I − KH)P` is cheaper but numerically fragile. The Joseph form `(I − KH)P(I − KH)ᵀ + KRKᵀ` is used instead. For a 7×7 matrix this is negligible cost, and it guarantees `P` stays positive semi-definite across thousands of update cycles without manual clamping.
