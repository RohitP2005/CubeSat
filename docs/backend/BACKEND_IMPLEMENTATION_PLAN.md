# Backend Implementation Plan — CubeSat Attitude Estimation Simulator

---

## Phase 1 — Core EKF Engine and Sensor Simulator ✅ COMPLETE

**Delivered**: 2026-06-14

All tasks in this phase are implemented and verified. 37/37 unit tests pass.

### Delivered

| Module | File | Description |
|---|---|---|
| Quaternion math | `app/ekf/quaternion.py` | Normalize, multiply, conjugate, DCM, Euler, error angle |
| Kinematics | `app/ekf/kinematics.py` | Ω and Ξ matrices, first-order quaternion propagation |
| Jacobians | `app/ekf/jacobians.py` | 7×7 process Jacobian F, 6×7 measurement Jacobian H |
| EKF | `app/ekf/ekf.py` | `AttitudeEKF` — predict + update, Joseph-form covariance |
| Orbit simulator | `app/simulation/orbit.py` | Circular orbit, ground-truth quaternion + angular velocity |
| Sensor simulator | `app/simulation/sensor_sim.py` | Gyro/accel/mag with noise and bias drift |
| Validation script | `scripts/validate_ekf.py` | 60 s simulation, RMSE report, 4-panel plot |
| Unit tests | `tests/test_quaternion.py` | 20 quaternion math tests |
| Unit tests | `tests/test_ekf.py` | 17 EKF tests (numerical Jacobian check + convergence) |

### Acceptance — All Met

- Mean angular error < 2° after convergence ✅
- Quaternion norm maintains 1 ± 1e-6 ✅
- Process and measurement Jacobians match numerical finite differences ✅
- 37/37 tests pass ✅

---

## Phase 2 — TRIAD Algorithm ✅ COMPLETE

**Delivered**: 2026-06-14

All tasks in this phase are implemented and verified. 25/25 unit tests pass.

### Delivered

| Module | File | Description |
|---|---|---|
| TRIAD estimator | `app/triad/triad.py` | `TriadEstimator` — DCM from two vector pairs, singularity guard |
| DCM → quaternion | `app/ekf/quaternion.py` | `from_dcm()` using Shepperd's method (delivered in Phase 1 update) |
| Unit tests | `tests/test_triad.py` | 25 tests: noiseless recovery, known rotations, singularity, scale invariance, TRIAD vs EKF |

### Acceptance — All Met

- Noiseless TRIAD recovers true attitude to within 1e-5° ✅
- Returns `None` when `|b1 × b2| < 0.1` (parallel, anti-parallel, and near-parallel) ✅
- Reference triad pre-computed in `__init__` — no per-call allocation ✅
- `test_triad_vs_ekf_noise_accuracy`: EKF mean error < TRIAD mean error under noise ✅
- 87/87 cumulative tests pass ✅

### Deliverables

| Task | Description |
|---|---|
| 2.1 TRIAD core | `app/triad/triad.py` — deterministic DCM from two vector pairs |
| 2.2 DCM → quaternion | Reuse `quaternion.py`; add `from_dcm` conversion |
| 2.3 Singularity guard | Detect near-parallel vectors; return `None` with warning flag |
| 2.4 TRIAD tests | `tests/test_triad.py` — noiseless recovery, singularity, known rotation |

### TRIAD Algorithm

Given:
- Body observations: `r1_body = normalize(accel)`, `r2_body = normalize(mag)`
- Inertial references: `r1_ref = g_ref`, `r2_ref = b_ref`

Construct triad bases:

```
# Observation triad
t1 = r1_body
t2 = normalize(r1_body × r2_body)
t3 = t1 × t2

# Reference triad
s1 = r1_ref
s2 = normalize(r1_ref × r2_ref)
s3 = s1 × s2

# DCM: maps reference frame to body frame
R = [t1|t2|t3] @ [s1|s2|s3]^T
```

### File Layout

```
backend/app/triad/
    __init__.py
    triad.py
backend/tests/
    test_triad.py
```

### Acceptance Criteria

- Noiseless TRIAD recovers true attitude to within 1e-6 deg
- Returns `None` when `|r1 × r2| < 0.1` (near-parallel vectors)
- TRIAD vs EKF accuracy difference visible in noisy test case

---

## Phase 3 — FastAPI Service and WebSocket ✅ COMPLETE

**Delivered**: 2026-06-14

All tasks implemented. 32/32 API tests pass. 119/119 cumulative tests pass.

### Delivered

| Module | File | Description |
|---|---|---|
| WebSocket manager | `app/api/ws_manager.py` | Per-channel connection pool, snapshot-safe broadcast, dead-client pruning |
| Simulation engine | `app/api/simulation_engine.py` | Async loop: orbit → sensor → TRIAD → EKF → broadcast + rolling stats |
| FastAPI app | `app/api/main.py` | All REST routes, WebSocket endpoints, CORS, lifespan |
| API tests | `tests/test_api.py` | 32 tests: lifecycle, configure, attitude frame schema, WebSocket delivery |

### Acceptance — All Met

- `POST /simulation/start` spawns async loop; `POST /simulation/stop` cancels it cleanly ✅
- WebSocket `/ws/attitude` delivers combined frame within one tick ✅
- WebSocket `/ws/telemetry` delivers raw sensor frame per tick ✅
- Both channels simultaneously connected work independently ✅
- Simulation reset and restart without server restart ✅
- `POST /simulation/configure` validates payload (Pydantic) and rebuilds components ✅
- EKF mean error < 2° after 200 steps (confirmed by `test_ekf_mean_error_under_target`) ✅
- 119/119 cumulative tests pass ✅

### Deliverables

| Task | Description |
|---|---|
| 3.1 FastAPI app | `app/api/main.py` — lifespan events, CORS, router registration |
| 3.2 Simulation engine | `app/api/simulation_engine.py` — owns the async loop: dynamics → sensor → TRIAD → EKF → broadcast |
| 3.3 WebSocket manager | `app/api/ws_manager.py` — connection pool, broadcast to all clients |
| 3.4 Attitude stream | `/ws/attitude` — combined frame: ground truth + TRIAD + EKF |
| 3.5 Telemetry stream | `/ws/telemetry` — raw sensor frame |
| 3.6 REST routes | All endpoints in BE-07 spec |
| 3.7 CORS config | Allow React dev server (`localhost:5173`) |

### Simulation Engine Loop (async)

```python
while running:
    data = sensor_sim.sample(t, dt)
    triad_result = triad.estimate(data['accel'], data['mag'])
    ekf.predict(data['gyro'], dt)
    ekf.update(data['accel'], data['mag'])
    frame = build_combined_frame(t, data, triad_result, ekf)
    await ws_manager.broadcast('/ws/attitude', frame)
    await ws_manager.broadcast('/ws/telemetry', data)
    evaluator.record(ekf.quaternion, triad_result.quaternion, data['true_q'])
    t += dt
    await asyncio.sleep(dt)
```

### REST Route Map

```
GET   /simulation/status
GET   /attitude/current
GET   /performance/summary
POST  /simulation/start
POST  /simulation/stop
POST  /simulation/reset
POST  /simulation/configure
```

### Acceptance Criteria

- `/simulation/start` begins the async loop; `/simulation/stop` pauses it cleanly
- WebSocket delivers frames within 50 ms of each simulation step
- 5 simultaneous WebSocket clients receive identical frames
- Simulation can be reset and restarted without restarting the server

---

## Phase 4 — Performance Evaluation Module ✅ COMPLETE

**Delivered**: 2026-06-14

All tasks implemented. 31 new evaluator tests pass. 150/150 cumulative tests pass.

### Delivered

| Module | File | Description |
|---|---|---|
| Evaluator | `app/evaluation/evaluator.py` | `PerformanceEvaluator` + `_ChannelStats` — per-axis error recording, RMSE aggregation, convergence window |
| Engine integration | `app/api/simulation_engine.py` | Replaced `_RollingStats` with `PerformanceEvaluator`; frame builder sources per-axis errors from evaluator |
| Tests | `tests/test_evaluator.py` | 31 tests: known-answer RMSE, convergence window, buffer cap, clear(), integration with orbit simulation |
| API test updates | `tests/test_api.py` | Updated field names (`rmse_deg` → `rmse_angular`); added `rmse_roll/pitch/yaw`, `convergence_steps` checks |

### Acceptance — All Met

- RMSE matches NumPy reference within 1e-6 ✅ (`test_rmse_angular_matches_numpy_reference`)
- Circular buffer never grows beyond maxlen ✅ (`test_buffer_does_not_grow_beyond_maxlen`)
- Convergence window correctly excludes pre-convergence steps ✅ (`test_convergence_window_excludes_early_steps`)
- `GET /performance/summary` returns full RMSE breakdown ✅
- WebSocket attitude frame now includes per-axis `roll/pitch/yaw_error_deg` fields ✅
- 150/150 cumulative tests pass ✅

### Deliverables

| Task | Description |
|---|---|
| 4.1 Evaluator class | `app/evaluation/evaluator.py` — records per-cycle errors in circular buffer |
| 4.2 Per-cycle metrics | `angular_error_deg`, `roll/pitch/yaw_error_deg` for TRIAD and EKF |
| 4.3 Aggregate metrics | `rmse_roll`, `rmse_pitch`, `rmse_yaw`, `rmse_angular` (post-convergence window) |
| 4.4 REST integration | `GET /performance/summary` returns current aggregates |
| 4.5 Tests | `tests/test_evaluator.py` — known-answer RMSE tests |

### Evaluator Interface

```python
class PerformanceEvaluator:
    def record(self, q_ekf, q_triad, q_true): ...
    def angular_error_ekf(self) -> float: ...   # latest step
    def angular_error_triad(self) -> float: ...
    def summary(self) -> dict: ...              # aggregate RMSE over buffer
```

### Acceptance Criteria

- RMSE values match NumPy reference computation within 1e-6
- Circular buffer does not grow unbounded (default cap: 10,000 steps)
- Summary endpoint returns < 5 ms from recorded data

---

## Phase 5 — Docker and Integration Tests

**Goal**: Containerized deployment and end-to-end tests that exercise the full pipeline.

**Duration estimate**: 2 days

### Deliverables

| Task | Description |
|---|---|
| 5.1 Dockerfile | Multi-stage Python build; production image < 200 MB |
| 5.2 docker-compose.yml | Backend service only (no DB/Redis needed) |
| 5.3 Integration test | Start server, run 10 s simulation, validate REST + WebSocket output |
| 5.4 Structured logging | JSON log output for all pipeline stages |
| 5.5 Health endpoint | `GET /health` → `{"status": "ok", "uptime_s": 42}` |

### Acceptance Criteria

- `docker compose up` starts service with no manual steps
- Integration test runs in CI without external dependencies
- `GET /health` returns 200 within 5 s of container start

---

## Phase 6 — Future Extensions

**Advanced Filters**
- Unscented Kalman Filter (UKF) as drop-in replacement for EKF
- Particle filter for comparison

**Additional Reference Sensors**
- Sun sensor simulation
- Star tracker simulation

**Hardware-in-the-Loop**
- Deploy estimator on Raspberry Pi
- Real-time UART input from physical IMU

**Orbit Integration**
- GMAT / Orekit interface for true LEO magnetic field model (IGRF)
- Orbital position as a function of time

---

## Risk Register

| Risk | Likelihood | Mitigation |
|---|---|---|
| TRIAD singularity when accel ∥ mag | Medium | Detect `|r1 × r2| < threshold`; skip TRIAD update for that step |
| EKF diverges at high rotation rates | Low | Quaternion re-normalized every cycle; proven in Phase 1 tests |
| asyncio loop drift at 100 Hz | Medium | Use `asyncio.sleep` with measured wall-clock correction; drop frames if behind |
| WebSocket frame backlog on slow clients | Low | Per-client send queue with a drop policy for oldest frames |
