# Phase 2 — TRIAD Algorithm: Mathematical Derivation

---

## 1. Purpose

TRIAD (Tri-Axial Attitude Determination) is a deterministic, single-shot method for computing the attitude rotation matrix from two vector measurements. It serves as the baseline estimator in this project and is compared against the EKF.

---

## 2. Problem Statement

Given:
- Two vectors measured in the **body frame**: `b1` (accelerometer), `b2` (magnetometer)
- Two corresponding reference vectors in the **inertial frame**: `r1` (gravity), `r2` (magnetic field)

Find the rotation matrix `R` (inertial → body) such that:

```
R @ r1 ≈ b1
R @ r2 ≈ b2
```

In general, noise prevents exact satisfaction of both constraints simultaneously. TRIAD prioritises the first pair exactly.

---

## 3. Reference Vectors

| Vector | Symbol | Value (normalised) |
|---|---|---|
| Gravity (NED, pointing down) | `g_ref` | `[0, 0, 1]` |
| LEO magnetic field | `b_ref` | `normalize([0.3090, 0, 0.9511])` |

Both sensor inputs are normalised internally — physical units (m/s², µT) and dimensionless normalised vectors give identical results.

---

## 4. Algorithm

### Step 1 — Build the observation triad (body frame)

```
b1 = normalize(accel)
b2 = normalize(mag)

t1 = b1                          (primary axis: matches gravity exactly)
t2 = normalize(b1 × b2)          (secondary axis: orthogonal to b1, in plane of b1,b2)
t3 = t1 × t2                     (completes right-handed triad)
```

`M_obs = [t1 | t2 | t3]`  — 3×3 orthogonal matrix with columns t1, t2, t3

### Step 2 — Build the reference triad (inertial frame)

```
s1 = g_ref                       (primary reference)
s2 = normalize(g_ref × b_ref)    (secondary reference: pre-computed in __init__)
s3 = s1 × s2                     (completes reference triad)
```

`M_ref = [s1 | s2 | s3]`  — 3×3 orthogonal matrix (constant, pre-computed)

### Step 3 — Compute the DCM

We want `R` such that `R @ si = ti` for i = 1, 2, 3.

In matrix form: `R @ M_ref = M_obs`

Since `M_ref` is orthogonal (`M_ref⁻¹ = M_refᵀ`):

```
R = M_obs @ M_refᵀ
```

### Step 4 — Convert DCM to quaternion

`from_dcm(R)` using Shepperd's method (see Phase 1 math doc).

---

## 5. Why TRIAD Prioritises the First Vector

The triad construction sets `t1 = b1` exactly. The second triad vector `t2 = normalize(b1 × b2)` is orthogonal to `t1` but lies in the plane spanned by `b1` and `b2` — it does not equal `normalize(b2)`.

As a result:
- `R @ r1 = b1` — exact match for the first vector
- `R @ r2 ≈ b2` — approximate match for the second (best achievable given orthogonality constraint)

This is why gravity (the more reliable measurement in LEO) is passed as the first observation.

---

## 6. Singularity Condition

The triad construction requires `b1 × b2 ≠ 0`, i.e., the two observation vectors must not be parallel.

The singularity guard checks:

```
|normalize(b1) × normalize(b2)| = |sin θ|
```

where θ is the angle between b1 and b2. The estimator returns `None` when:

```
|sin θ| < singularity_threshold   (default: 0.1, i.e., θ < ~6°)
```

**Physical interpretation**: Near the magnetic poles, the Earth's magnetic field is nearly vertical and nearly parallel to the gravity vector. In that region, TRIAD is geometrically undefined and the EKF (which uses gyroscope integration) should be relied upon.

---

## 7. Performance Characteristics

| Property | TRIAD | EKF |
|---|---|---|
| Computation per step | O(1), no matrix inversion | O(n²), n=7 state |
| Memory | Stateless | Full 7×7 covariance |
| Noise rejection | None (single-shot) | Recursive filtering |
| Accuracy under noise | Degrades with σ | Converges, ~2° typical |
| Gyroscope required | No | Yes (predict step) |
| Singularity-free | No | Yes (gyro propagates through) |

---

## 8. TRIAD Error Budget

For small measurement noise `σ_a` (normalised):

```
σ_TRIAD ≈ σ_a / |sin θ_ref|
```

where `θ_ref` is the angle between `g_ref` and `b_ref`. For the default reference vectors:

```
|sin θ_ref| = |g_ref × b_ref| = |[0,0,1] × [0.309,0,0.951]|
            = |[0*0.951-1*0, 1*0.309-0*0.951, 0*0-0*0.309]|
            = |[0, 0.309, 0]| = 0.309
```

With `σ_a = 0.05` (normalised): `σ_TRIAD ≈ 0.05 / 0.309 ≈ 0.16 rad ≈ 9°`.

This explains why `test_triad_accuracy_under_noise_within_bound` expects mean error < 10°.

The EKF achieves < 2° under the same noise by filtering across many measurements.

---

## 9. Implementation Notes

- `M_refᵀ` is pre-computed once in `__init__` — the reference triad is constant
- Both inputs are normalised in `estimate()` — physical units are accepted without pre-processing
- Zero-length vectors (degenerate sensor readings) return `None` before the singularity check
- `from_dcm` uses Shepperd's branching method to avoid near-zero denominators for all possible DCMs
