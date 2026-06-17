# Frontend Implementation Plan — CubeSat Attitude Estimation Dashboard

---

## Phase 1 — Project Scaffold and Routing

**Goal**: Establish the project foundation, all routes, and page shells.

**Duration estimate**: 2–3 days

### Deliverables

| Task | Description |
|---|---|
| 1.1 Vite + React + TypeScript | Project bootstrap, strict TS config |
| 1.2 TailwindCSS | Configure dark theme base, design tokens |
| 1.3 React Router v6 | Four routes: `/`, `/telemetry`, `/comparison`, `/performance` |
| 1.4 Page shells | Placeholder components for all four pages |
| 1.5 Layout | `NavBar` (left sidebar or top nav) + content area |
| 1.6 Zustand stores | Empty store definitions for all five stores |
| 1.7 Axios client | Base URL from env var, request interceptors |
| 1.8 ESLint + Prettier | Code style enforced |

### Acceptance Criteria

- All four routes render without errors
- Navigation works between all pages
- TS strict mode compiles with zero errors

---

## Phase 2 — Simulation Control and Connection Status

**Goal**: Working simulation control bar with real REST integration. Dashboard knows if the simulator is running.

**Duration estimate**: 2 days

### Deliverables

| Task | Description |
|---|---|
| 2.1 `SimulationBar` component | Persistent top bar: status badge, elapsed time, Start/Stop/Reset buttons |
| 2.2 `simulationStore` | Holds run state, elapsed time, WebSocket connection status |
| 2.3 REST calls | `POST /simulation/start|stop|reset`, `GET /simulation/status` |
| 2.4 Status polling | `useSimulationStatus` hook — polls `GET /simulation/status` every 2 s |
| 2.5 `useWebSocket` hook | Generic WebSocket with auto-reconnect, exponential back-off (max 5 retries) |
| 2.6 Connection indicator | Green/amber/red badge based on WebSocket state |

### Acceptance Criteria

- Start button triggers simulation and status badge changes to RUNNING within 2 s
- Stop button halts simulation; status badge shows STOPPED
- WebSocket reconnection fires automatically without page reload
- All three buttons are disabled when WebSocket is disconnected

---

## Phase 3 — 3D CubeSat Visualization

**Goal**: Real-time 3D CubeSat model rotating to match live attitude data.

**Duration estimate**: 4–5 days

### Deliverables

| Task | Description |
|---|---|
| 3.1 R3F canvas | `AttitudeScene` — lighting (ambient + directional), perspective camera |
| 3.2 `CubeSatMesh` | Parametric box (1U CubeSat proportions), face colors, body axis arrows |
| 3.3 `useAttitudeFeed` hook | Subscribe to `/ws/attitude`, write to `attitudeStore` |
| 3.4 Quaternion → rotation | Apply quaternion directly to Three.js mesh `quaternion` property |
| 3.5 Source toggle | Toggle between Ground Truth / TRIAD / EKF; drives the active mesh rotation |
| 3.6 Ghost overlay | Second translucent mesh showing ground truth when EKF/TRIAD is selected |
| 3.7 `AttitudeReadout` panel | Live Roll / Pitch / Yaw for selected source, + angular error vs truth |

### Acceptance Criteria

- Mesh visually tracks attitude changes with no perceptible lag (within one animation frame)
- FPS stays above 30 at 100 Hz WebSocket rate
- Toggling sources changes which quaternion drives the mesh immediately
- Ghost mesh appears correctly offset when estimates diverge

---

## Phase 4 — Sensor Telemetry Charts

**Goal**: Scrolling live charts for all nine sensor channels.

**Duration estimate**: 3–4 days

### Deliverables

| Task | Description |
|---|---|
| 4.1 `useTelemetryFeed` hook | Subscribe to `/ws/telemetry`, write to `telemetryStore` |
| 4.2 Rolling buffer | `telemetryStore` holds last N samples per axis (configurable cap) |
| 4.3 `TelemetryChart` component | Plotly.js line chart with three axes per sensor group |
| 4.4 Telemetry page | Three chart groups (gyro, accel, mag) with units and axis labels |
| 4.5 Window selector | 10 s / 30 s / 60 s view window selector |
| 4.6 Pause button | Freezes chart render; WebSocket keeps filling buffer |
| 4.7 Debounce | Batch store updates at 60 Hz max to avoid Plotly re-render thrashing |

### Acceptance Criteria

- Charts update within 200 ms of incoming data
- 30 s buffer at 100 Hz (3000 points per axis) does not cause memory growth
- Pause/resume works without data loss or chart glitch

---

## Phase 5 — Estimator Comparison View

**Goal**: Side-by-side Euler angle overlay and angular error chart for TRIAD vs EKF vs ground truth.

**Duration estimate**: 3–4 days

### Deliverables

| Task | Description |
|---|---|
| 5.1 `comparisonStore` | Rolling buffer of Euler angles per source (ground truth, TRIAD, EKF) |
| 5.2 `EulerOverlayChart` | Three subplots (roll, pitch, yaw); each shows all three sources |
| 5.3 `ErrorChart` | Single plot: TRIAD angular error + EKF angular error + 2° target line |
| 5.4 `QuaternionDisplay` | q0–q3 numeric readout comparing EKF vs ground truth |
| 5.5 TRIAD gap handling | Null values when TRIAD returns singularity — chart shows gap, no crash |
| 5.6 Comparison page layout | Two-column layout: Euler overlay (left) + error chart (right) |

### Acceptance Criteria

- Euler overlay updates in sync with attitude WebSocket
- TRIAD gaps render as blank segments (not zero or NaN artifacts)
- Angular error chart shows 2° dashed target line at all zoom levels
- All three sources visually distinguishable (color + line style)

---

## Phase 6 — Performance Metrics View

**Goal**: Aggregate RMSE cards, covariance convergence chart, TRIAD vs EKF comparison bar chart.

**Duration estimate**: 2–3 days

### Deliverables

| Task | Description |
|---|---|
| 6.1 `performanceStore` | Holds latest summary from `GET /performance/summary` + covariance trace buffer |
| 6.2 `usePerformancePoll` hook | Polls `GET /performance/summary` every 2 s; writes to store |
| 6.3 `MetricCard` component | Displays one RMSE or error value with label and unit |
| 6.4 Metrics grid | 2×2 or 3×2 grid of `MetricCard` components |
| 6.5 `RmseBarChart` | Grouped bar chart: TRIAD vs EKF for roll/pitch/yaw |
| 6.6 `CovarianceChart` | EKF covariance trace over time (log-scale y-axis) |
| 6.7 Performance page layout | Cards top row; charts below |

### Acceptance Criteria

- Metric cards update within 2 s of simulation state changes
- Covariance chart shows convergence (decreasing trace) as EKF runs
- RMSE bars clearly show EKF outperforming TRIAD under noise
- All charts render correctly on first load before data arrives (graceful empty state)

---

## Phase 7 — Polish, Testing, and Build

**Goal**: Production-ready application with test coverage and optimized bundle.

**Duration estimate**: 2–3 days

### Deliverables

| Task | Description |
|---|---|
| 7.1 Loading states | Skeleton loaders for charts; spinner during WebSocket initial connect |
| 7.2 Error boundaries | Catch render errors in chart and 3D scene; show fallback message |
| 7.3 WebSocket lost banner | "Simulator disconnected — reconnecting…" banner with countdown |
| 7.4 Unit tests | Zustand store logic, rolling buffer behavior, WebSocket hook state machine |
| 7.5 Component tests | `SimulationBar`, `MetricCard`, `AttitudeReadout` with mock data |
| 7.6 Bundle optimization | Code-split R3F and Plotly behind lazy routes |
| 7.7 Dockerfile | Nginx static file server, `VITE_API_URL` injected at runtime |

---

## Dependencies on Backend

| Frontend Phase | Requires Backend Phase |
|---|---|
| Phase 2 (Simulation controls) | BE-3 REST routes |
| Phase 3 (3D visualization) | BE-3 `/ws/attitude` |
| Phase 4 (Telemetry charts) | BE-3 `/ws/telemetry` |
| Phase 5 (Comparison view) | BE-3 `/ws/attitude` (requires TRIAD data in frame) |
| Phase 6 (Performance metrics) | BE-4 `GET /performance/summary` |

Frontend Phases 1–2 can begin with mock WebSocket data. Phase 3+ requires BE-3 complete.

---

## Risk Register

| Risk | Likelihood | Mitigation |
|---|---|---|
| R3F FPS drop at 100 Hz WebSocket | Medium | Decouple WebSocket store writes from `requestAnimationFrame`; mesh updates at render rate |
| Plotly thrash at 100 Hz | High | Debounce store updates to 60 Hz max; use `Plotly.extendTraces` not full re-render |
| WebSocket buffer buildup | Medium | Rolling buffer with hard cap (e.g. 6000 samples at 100 Hz = 60 s); drop oldest |
| TRIAD null frames crash charts | Low | Chart components guard for `null` before rendering; render gap not zero |
