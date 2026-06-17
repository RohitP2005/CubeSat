# Frontend Specification — CubeSat Attitude Estimation Dashboard

## Project Name

CubeSat Attitude Estimation Dashboard

## Domain

Spacecraft Simulation Visualization / Real-Time Data Display

---

## Technology Stack

| Component | Technology |
|---|---|
| Language | TypeScript |
| Framework | React 18 |
| 3D Rendering | React Three Fiber + Three.js |
| Styling | TailwindCSS |
| Charting | Plotly.js |
| State Management | Zustand |
| Real-Time | WebSocket (native browser API) |
| HTTP Client | Axios |
| Build Tool | Vite |
| Testing | Vitest + React Testing Library |

---

## Objective

Visualize simulated CubeSat attitude in real time, display raw sensor telemetry, compare TRIAD and EKF estimates against ground truth, and present algorithm performance metrics — all driven by live WebSocket feeds from the backend simulator.

No authentication, session management, or historical replay is required. The dashboard connects to the simulator, which runs on the same machine.

---

## Functional Requirements

### FE-01 — Simulation Control Bar

Persistent top bar visible on all pages. Displays:

- Simulation status (RUNNING / STOPPED / RESET)
- Simulation time elapsed (seconds)
- WebSocket connection indicator
- Start / Stop / Reset buttons (call `POST /simulation/start|stop|reset`)

No confirmation modal required — these are simulation controls, not destructive operations.

---

### FE-02 — 3D CubeSat Visualization

Full-page 3D scene using React Three Fiber.

**CubeSat model**:
- Parametric box mesh with face panel colors (solar panels, body)
- Color-coded body axes: X = red, Y = green, Z = blue

**Attitude sources** (toggle):
- Ground Truth (ghost/translucent mesh)
- TRIAD estimate (solid mesh)
- EKF estimate (solid mesh, default)

**Display panel**:
- Roll / Pitch / Yaw in degrees for the selected source
- Angular error vs ground truth for TRIAD and EKF (side by side)

**Data source**: `/ws/attitude`

**Target frame rate**: 30–60 FPS

---

### FE-03 — Sensor Telemetry Dashboard

Time-series line charts for all nine raw sensor channels.

#### Gyroscope

```
gx  gy  gz   (rad/s)
```

#### Accelerometer

```
ax  ay  az   (normalized)
```

#### Magnetometer

```
mx  my  mz   (normalized)
```

- Scrolling 30 s window (configurable: 10 s / 30 s / 60 s)
- Three subplots, one per sensor group
- Each axis in a distinct color
- Pause button freezes chart without disconnecting WebSocket
- **Data source**: `/ws/telemetry`

---

### FE-04 — Estimator Comparison View

Side-by-side comparison of all three attitude sources over time.

#### Euler Angle Overlay

Three subplots (roll, pitch, yaw), each showing:
- Ground Truth (black)
- TRIAD (orange)
- EKF (blue)

Scrolling 30 s window. Gaps appear when TRIAD returns null (singularity).

#### Quaternion Components

Four subplots (q0–q3), each showing:
- Ground Truth vs EKF

#### Angular Error Over Time

Single plot showing:
- TRIAD angular error (orange)
- EKF angular error (blue)
- 2° accuracy target line (dashed red)

**Data source**: `/ws/attitude` (`triad.angular_error_deg`, `ekf.angular_error_deg`)

---

### FE-05 — Performance Metrics View

Aggregate statistics for the current simulation run.

**Metric cards**:

| Card | Content |
|---|---|
| EKF RMSE | Roll / Pitch / Yaw RMSE in degrees |
| TRIAD RMSE | Roll / Pitch / Yaw RMSE in degrees |
| EKF Mean Error | Mean angular error (degrees) |
| TRIAD Mean Error | Mean angular error (degrees) |
| EKF vs TRIAD | Improvement ratio |

**Covariance trace chart**:
- EKF covariance trace over time (log scale)
- Shows filter convergence

**RMSE bar chart**:
- Side-by-side bars for TRIAD vs EKF per axis

**Data source**: `GET /performance/summary` (polled every 2 s)

---

## Scope Exclusions

The following are explicitly out of scope for this dashboard:

- Authentication / login
- Historical session replay
- Multi-satellite monitoring
- Operator command interface beyond Start / Stop / Reset
- Database-backed telemetry history
- Mission management

---

## API Consumption

### REST Endpoints

```
GET   /simulation/status
GET   /attitude/current
GET   /performance/summary
POST  /simulation/start
POST  /simulation/stop
POST  /simulation/reset
```

### WebSocket Channels

```
/ws/attitude    — combined frame: ground_truth + triad + ekf
/ws/telemetry   — raw sensor frame: gyro + accel + mag
```

---

## Application Routes

```
/              — 3D CubeSat visualization (default)
/telemetry     — sensor charts
/comparison    — estimator comparison (Euler overlay + error)
/performance   — RMSE and aggregate metrics
```

---

## State Management (Zustand Stores)

| Store | State Held |
|---|---|
| `simulationStore` | Run state (running/stopped), elapsed time, WebSocket status |
| `attitudeStore` | Latest combined frame: ground truth, TRIAD, EKF |
| `telemetryStore` | Rolling buffer (last N frames) for gyro / accel / mag |
| `comparisonStore` | Rolling buffer of Euler angles per source for overlay charts |
| `performanceStore` | Latest RMSE summary, covariance trace buffer |

---

## Non-Functional Requirements

| Requirement | Target |
|---|---|
| 3D render FPS | 30 FPS minimum at 100 Hz WebSocket rate |
| Chart update lag | < 200 ms behind WebSocket data |
| WebSocket reconnect | Automatic with exponential back-off (max 5 retries) |
| Bundle size | < 2 MB gzipped initial load |
| Browser support | Chrome 110+, Firefox 115+, Edge 110+ |

---

## Component Structure

```
src/
├── pages/
│   ├── Visualization.tsx      — 3D scene (default route)
│   ├── Telemetry.tsx          — sensor charts
│   ├── Comparison.tsx         — Euler overlay + error charts
│   └── Performance.tsx        — RMSE metrics + covariance chart
├── components/
│   ├── layout/
│   │   ├── SimulationBar.tsx  — top bar: status + controls
│   │   └── NavBar.tsx         — page navigation
│   ├── cubesat/
│   │   ├── AttitudeScene.tsx  — R3F canvas + lighting
│   │   └── CubeSatMesh.tsx    — parametric box with axes
│   ├── charts/
│   │   ├── TelemetryChart.tsx     — 3-axis scrolling chart
│   │   ├── EulerOverlayChart.tsx  — ground truth / TRIAD / EKF
│   │   ├── ErrorChart.tsx         — angular error over time
│   │   ├── CovarianceChart.tsx    — EKF covariance trace
│   │   └── RmseBarChart.tsx       — TRIAD vs EKF bars
│   ├── panels/
│   │   ├── AttitudeReadout.tsx    — Roll / Pitch / Yaw display
│   │   ├── QuaternionDisplay.tsx  — q0–q3 numeric
│   │   └── MetricCard.tsx         — RMSE / error card
│   └── ui/
│       ├── StatusBadge.tsx
│       └── ToggleButton.tsx
├── stores/
│   ├── simulationStore.ts
│   ├── attitudeStore.ts
│   ├── telemetryStore.ts
│   ├── comparisonStore.ts
│   └── performanceStore.ts
├── hooks/
│   ├── useWebSocket.ts
│   ├── useAttitudeFeed.ts
│   └── useTelemetryFeed.ts
├── api/
│   └── client.ts
└── App.tsx
```
