# Phase 1 — Mathematical Derivation

## EKF Attitude Estimation for CubeSat

---

## 1. Conventions and Frames

### Quaternion Convention

Scalar-first: `q = [q0, q1, q2, q3]` where `q0` is the scalar part.

The unit quaternion encodes a **passive rotation** — it describes the orientation of the body frame relative to the inertial frame. A vector expressed in inertial coordinates is transformed to body coordinates by:

```
v_body = R(q) * v_inertial
```

The quaternion double cover means `q` and `-q` represent the same physical rotation.

### Reference Frames

| Frame | Symbol | Description |
|---|---|---|
| Inertial | I | Earth-Centered Inertial (ECI) or North-East-Down (NED) |
| Body | B | Fixed to the CubeSat structure |

### Gravity Reference

In NED, gravity points down (positive z). The normalized reference vector used by the filter:

```
g_ref = [0, 0, 1]
```

### Magnetic Field Reference

A simplified dipole reference representative of a 500 km LEO equatorial orbit:

```
b_ref = normalize([0.3090, 0.0, 0.9511])
```

---

## 2. Direction Cosine Matrix

Given unit quaternion `q = [q0, q1, q2, q3]`, the DCM `R(q)` (inertial → body) is:

```
R = [ q0²+q1²-q2²-q3²,   2(q1q2+q0q3),         2(q1q3-q0q2)         ]
    [ 2(q1q2-q0q3),       q0²-q1²+q2²-q3²,       2(q2q3+q0q1)         ]
    [ 2(q1q3+q0q2),       2(q2q3-q0q1),           q0²-q1²-q2²+q3²     ]
```

Properties: orthogonal (`R Rᵀ = I`), determinant = +1, invertible by transposition.

---

## 3. Quaternion Kinematics

The continuous-time equation for quaternion rate given body-frame angular velocity `ω`:

```
q̇ = 0.5 * q ⊗ [0, ω]ᵀ  =  0.5 * Ξ(q) * ω  =  0.5 * Ω(ω) * q
```

### Xi Matrix Ξ(q) — 4×3

```
Ξ(q) = [ -q1  -q2  -q3 ]
         [  q0  -q3   q2 ]
         [  q3   q0  -q1 ]
         [ -q2   q1   q0 ]
```

**Key property**: For unit quaternions, `Ξᵀ Ξ = I₃`. This is used to recover `ω` from `q̇` via `ω = 2 Ξ(q)ᵀ q̇`.

### Omega Matrix Ω(ω) — 4×4

```
Ω(ω) = [  0   -ωx  -ωy  -ωz ]
         [ ωx    0   ωz  -ωy ]
         [ ωy  -ωz    0   ωx ]
         [ ωz   ωy  -ωx    0 ]
```

The identity `Ω(ω) * q = Ξ(q) * ω` holds for all unit `q` and all `ω`.

---

## 4. State Vector

```
x = [ q0, q1, q2, q3,  bx, by, bz ]
      ←— quaternion —→  ←— bias —→
```

- `q`: unit quaternion representing spacecraft orientation
- `b`: gyroscope bias in body frame [rad/s]

---

## 5. Process Model

### True Angular Velocity

The gyroscope measures `ω_meas = ω_true + b + noise`. The bias-corrected estimate:

```
ω_corrected = ω_meas - b
```

### State Propagation

Using first-order Euler integration (valid for small `dt`):

```
q(k+1) = normalize(q(k) + 0.5 * dt * Ω(ω_corrected) * q(k))
b(k+1) = b(k)   [random walk, noise added via Q]
```

### Process Jacobian F — 7×7

Linearizing the process model `f(x, u)` about current state:

```
F = ∂f/∂x =

[ I₄ + 0.5·dt·Ω(ω_corr)  |  -0.5·dt·Ξ(q) ]   ← 4 rows (quaternion)
[ ─────────────────────────────────────────── ]
[         0₃ₓ₄            |      I₃         ]   ← 3 rows (bias)
```

**Derivation of the off-diagonal block**:

Since `ω_corrected = ω_meas - b`, we have `∂ω_corrected/∂b = -I₃`. Therefore:

```
∂q(k+1)/∂b = 0.5·dt · ∂(Ω(ω)*q)/∂ω · (-I) = -0.5·dt · Ξ(q)
```

The last equality uses the identity `∂(Ω(ω)*q)/∂ω = Ξ(q)`.

### Process Noise Covariance Q — 7×7

Diagonal approximation scaled by `dt`:

```
Q = diag(σ_ω²·dt, σ_ω²·dt, σ_ω²·dt, σ_ω²·dt,  σ_b²·dt, σ_b²·dt, σ_b²·dt)
```

---

## 6. Measurement Model

### Measurement Vector

```
z = [ ax, ay, az,  mx, my, mz ]ᵀ   (normalized)
```

### Predicted Measurements

Both are computed by rotating the inertial reference vector into the body frame:

```
h_acc(q) = R(q) * g_ref     (expected gravity direction in body)
h_mag(q) = R(q) * b_ref     (expected magnetic field direction in body)
```

### Measurement Jacobian H — 6×7

```
H = ∂h/∂x =

[ ∂(R·g_ref)/∂q  |  0₃ₓ₃ ]   ← accelerometer rows
[ ∂(R·b_ref)/∂q  |  0₃ₓ₃ ]   ← magnetometer rows
```

The bias columns are zero because the measurement model does not depend on gyroscope bias directly.

### Jacobian of R(q)·v with respect to q — 3×4

For a fixed reference vector `v = [vx, vy, vz]`:

```
∂(R·v)/∂q = 2 ·

[ q0·vx + q3·vy - q2·vz    q1·vx + q2·vy + q3·vz   -q2·vx + q1·vy - q0·vz   -q3·vx + q0·vy + q1·vz ]
[-q3·vx + q0·vy + q1·vz    q2·vx - q1·vy + q0·vz    q1·vx + q2·vy + q3·vz   -q0·vx - q3·vy + q2·vz ]
[ q2·vx - q1·vy + q0·vz    q3·vx - q0·vy - q1·vz    q0·vx + q3·vy - q2·vz    q1·vx + q2·vy + q3·vz ]
```

Verified numerically via central finite differences in `tests/test_ekf.py`.

### Measurement Noise Covariance R — 6×6

```
R = diag(σ_a², σ_a², σ_a²,  σ_m², σ_m², σ_m²)
```

---

## 7. EKF Algorithm

```
─── Initialization ───
x₀ = [q_init; 0; 0; 0]
P₀ = diag(0.1, 0.1, 0.1, 0.1, 1e-4, 1e-4, 1e-4)

─── Per cycle (called at rate f_EKF) ───

Step 1 — Predict state
  ω = ω_meas - b
  q̂ = normalize(q + 0.5·dt·Ω(ω)·q)
  b̂ = b

Step 2 — Predict covariance
  F = process_jacobian(q, ω, dt)
  P⁻ = F·P·Fᵀ + Q

Step 3 — Innovation and Kalman gain
  z = normalize([accel; mag])
  h = [R(q̂)·g_ref; R(q̂)·b_ref]
  y = z - h
  H = measurement_jacobian(q̂, g_ref, b_ref)
  S = H·P⁻·Hᵀ + R
  K = P⁻·Hᵀ·S⁻¹

Step 4 — Update state
  x̂ = x̂⁻ + K·y
  q̂ = normalize(q̂)

Step 5 — Update covariance (Joseph form)
  IKH = I - K·H
  P = IKH·P⁻·IKHᵀ + K·R·Kᵀ
```

The **Joseph form** for the covariance update is used instead of the simpler `P = (I-KH)P` because it is symmetric and positive semi-definite even under floating-point rounding — critical for filter stability over thousands of cycles.

---

## 8. Sensor Noise Models

### Gyroscope

```
ω_meas(t) = ω_true(t) + b(t) + η_g(t)

η_g ~ N(0, σ_g²)    white noise
ḃ(t) = η_b(t)        bias random walk: η_b ~ N(0, σ_b²)
```

| Parameter | Default | Description |
|---|---|---|
| σ_g | 0.005 rad/s | Gyro white noise (1σ per sample) |
| σ_b | 1e-5 rad/s/√Hz | Bias instability (random walk rate) |

### Accelerometer

```
a_meas = R(q) · g_ref + η_a

η_a ~ N(0, σ_a²)
```

| Parameter | Default |
|---|---|
| σ_a | 0.05 (normalized) |

### Magnetometer

```
m_meas = R(q) · b_ref + η_m

η_m ~ N(0, σ_m²)
```

| Parameter | Default |
|---|---|
| σ_m | 0.02 (normalized) |

---

## 9. Accuracy Target

The filter shall achieve:

```
Mean angular error < 2 degrees (after 5 s convergence window)
```

under the default noise parameters specified above. Verified by `scripts/validate_ekf.py`.
