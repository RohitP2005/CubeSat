# CubeSat Attitude Estimation — Running Locally

Two processes must run simultaneously: the **Python backend** and the **React frontend**.

---

## Quick Start

Open two terminals side by side.

### Terminal 1 — Backend

```bash
cd c:\CubeSat\backend

# Install dependencies (first time only)
pip install -r requirements.txt

# Start the server
uvicorn app.api.main:app --reload --port 8000
```

Server starts at **http://localhost:8000**
Interactive API docs at **http://localhost:8000/docs**

---

### Terminal 2 — Frontend

```bash
cd c:\CubeSat\frontend

# Install dependencies (first time only — uses bun or npm)
npm install        # or: bun install

# Start the dev server
npm run dev        # or: bun run dev
```

Dashboard starts at **http://localhost:8080** (TanStack Start / Nitro default)
> If port 8080 is busy the terminal will print the actual port.

---

## Operating Modes

The top bar contains a three-way **Demo · Simulate · Live** selector.

### Demo (default)
Synthetic data generated in the browser at 50 Hz. No backend required. All views are fully functional.

### Simulate
Connects to the FastAPI backend, which runs a physics-based simulation at 100 Hz.

1. Start the backend (Terminal 1 above)
2. Click **Simulate** in the top bar — the LINK badge turns green
3. Click **Start** to begin streaming
4. An amber banner on each page confirms you are in simulation mode

### Live
Uses a real TLE orbit (ISS by default) propagated via SGP4, with sensor readings modelled from real orbital geometry.

1. Start the backend
2. Click **Live** in the top bar
3. Click **Start** — the backend fetches the latest ISS TLE and begins streaming
4. Panels that require ground-truth comparison (Angular Error, RMSE, Comparison charts) show a **"Live value not available"** overlay, because true satellite attitude cannot be independently verified

---

## Environment Variables

No environment variables are required for local development. Both services use their default ports.

If you need to change the backend URL (e.g. running on a different machine or port):

**Frontend** — create `c:\CubeSat\frontend\.env.local`:
```
VITE_API_BASE_URL=http://your-host:8000
```

The frontend reads this at build/dev time and derives the WebSocket URL automatically
(`http://` → `ws://`, `https://` → `wss://`).

**Backend** — CORS is pre-configured to allow:
```
http://localhost:5173
http://localhost:3000
```

If the frontend runs on a different origin, add it to `allow_origins` in
`backend/app/api/main.py`.

---

## Backend Configuration

The simulation parameters can be changed via the REST API without restarting the server:

```bash
curl -X POST http://localhost:8000/simulation/configure \
  -H "Content-Type: application/json" \
  -d '{
    "dt": 0.01,
    "altitude_km": 500,
    "tumble_rate_deg_s": 0.1,
    "sigma_gyro": 0.005,
    "sigma_accel": 0.05,
    "sigma_mag": 0.02,
    "rng_seed": 42
  }'
```

| Parameter | Default | Description |
|---|---|---|
| `dt` | `0.01` | Simulation time step in seconds (100 Hz) |
| `altitude_km` | `500` | Orbit altitude |
| `tumble_rate_deg_s` | `0.1` | Tumble amplitude |
| `sigma_gyro` | `0.005` | Gyroscope white noise (rad/s) |
| `sigma_accel` | `0.05` | Accelerometer noise (normalised) |
| `sigma_mag` | `0.02` | Magnetometer noise (normalised) |
| `rng_seed` | `42` | Reproducible noise seed |

---

## Verify Backend Is Working

```bash
# Health check
curl http://localhost:8000/health

# Simulation status
curl http://localhost:8000/simulation/status

# Start simulation, wait, get attitude
curl -X POST http://localhost:8000/simulation/start
curl http://localhost:8000/attitude/current

# Performance metrics
curl http://localhost:8000/performance/summary
```

---

## Running Tests

```bash
cd c:\CubeSat\backend
python -m pytest tests/ -v
```

152 tests across EKF math, TRIAD algorithm, simulation module, performance evaluator,
and full API integration.

---

## Architecture Summary

```
Frontend (React / TanStack Start)
  http://localhost:3000
        │
        │  REST  POST /simulation/start|stop|reset|configure
        │        GET  /simulation/status   (polled every 2 s)
        │        GET  /performance/summary (polled every 2 s)
        │
        │  WS    /ws/attitude   → 3D viewer, charts, comparison
        │        /ws/telemetry  → gyro/accel/mag sensor charts
        ▼
Backend (FastAPI / uvicorn)
  http://localhost:8000

  Simulation loop (100 Hz asyncio task):
    CircularOrbit → SensorSimulator → TRIAD + EKF → PerformanceEvaluator
    → broadcast /ws/attitude  (combined frame: ground truth + TRIAD + EKF)
    → broadcast /ws/telemetry (raw sensor readings)
```
