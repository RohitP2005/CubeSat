# CubeSat Attitude Estimation Simulator — System Overview

## Project Summary

A self-contained simulator for spacecraft attitude determination. The system generates synthetic CubeSat rotational motion, produces realistic IMU and magnetometer measurements, estimates attitude using two algorithms (TRIAD and EKF), and streams all data to a React visualization dashboard.

This is a **simulator** — not a flight software system. There is no authentication, no database persistence, no mission management, and no hardware communication layer. All data originates from the internal dynamics simulator.

---

## Scope

### Included

- Spacecraft rotational dynamics simulation
- IMU simulation (gyroscope + accelerometer)
- Magnetometer simulation
- Gaussian noise and bias modelling
- TRIAD attitude determination
- Extended Kalman Filter (EKF) attitude estimation
- Algorithm performance comparison
- Real-time WebSocket streaming
- React visualization dashboard

### Excluded

- Attitude control (reaction wheels, magnetorquers)
- Orbit propagation
- Ground station communications
- Authentication / multi-user
- Mission management / sessions
- Database persistence
- Hardware telemetry ingestion

---

## Architecture

```
Spacecraft Dynamics Simulator
            │
            ▼
      Sensor Models
      (Gyro / Accel / Mag + Noise)
            │
            ▼
     Sensor Fusion Layer
     ├── TRIAD Algorithm
     └── EKF (7-state with bias)
            │
            ▼
     Attitude Estimates
     (Ground Truth / TRIAD / EKF)
            │
            ▼
        FastAPI Service
        ├── REST API
        └── WebSocket Stream
            │
            ▼
      React Dashboard
```

---

## Component Summary

| Component | Technology | Responsibility |
|---|---|---|
| Dynamics Simulator | Python / NumPy | Generate ground-truth quaternion trajectory |
| Sensor Simulator | Python / NumPy | Gyro / accel / mag with noise and bias drift |
| TRIAD Module | Python / NumPy | Deterministic attitude from two vector pairs |
| EKF Module | Python / NumPy | Recursive state estimation (7-state with gyro bias) |
| Performance Evaluator | Python / NumPy | RMSE, quaternion error, angular error |
| API Service | FastAPI | REST + WebSocket server |
| Dashboard | React / TypeScript | 3D visualization, charts, comparison views |

---

## Data Flow

```
Dynamics Simulator  →  true quaternion + true angular velocity
        │
        ▼
Sensor Simulator    →  noisy gyro, accel, mag measurements
        │
        ├─────────────────────────┐
        ▼                         ▼
   TRIAD Estimator           EKF Estimator
        │                         │
        └──────────┬──────────────┘
                   ▼
         Performance Evaluator  →  error metrics vs ground truth
                   │
                   ▼
             FastAPI Service
             ├── REST:  GET /attitude/current
             │          GET /simulation/status
             │          POST /simulation/start
             │          POST /simulation/stop
             │          POST /simulation/reset
             └── WS:    /ws/attitude  (streams all three estimates)
                        /ws/telemetry (streams raw sensor frames)
                   │
                   ▼
           React Dashboard
           ├── 3D CubeSat Viewer
           ├── Sensor Charts
           ├── Estimator Comparison
           └── Performance Metrics
```

---

## Development Phases

| Phase | Component | Status |
|---|---|---|
| BE-1 | Core EKF engine + sensor simulator | **Complete** |
| BE-2 | TRIAD algorithm | Pending |
| BE-3 | FastAPI service + WebSocket | Pending |
| BE-4 | Performance evaluation module | Pending |
| BE-5 | Docker + integration tests | Pending |
| FE-1 | React scaffold + routing | Pending |
| FE-2 | 3D CubeSat visualization | Pending |
| FE-3 | Sensor telemetry charts | Pending |
| FE-4 | Estimator comparison view | Pending |
| FE-5 | Performance metrics view | Pending |
| FE-6 | Polish + build | Pending |

---

## Repository Structure

```
cubesat/
├── backend/
│   ├── app/
│   │   ├── ekf/               # Quaternion math, kinematics, Jacobians, EKF
│   │   ├── triad/             # TRIAD algorithm
│   │   ├── simulation/        # Dynamics and sensor simulators
│   │   ├── api/               # FastAPI routes, WebSocket manager
│   │   └── evaluation/        # RMSE, error metrics
│   ├── scripts/
│   │   └── validate_ekf.py
│   ├── tests/
│   └── requirements.txt
├── frontend/
│   ├── src/
│   │   ├── pages/
│   │   ├── components/
│   │   ├── stores/
│   │   └── hooks/
│   └── package.json
├── docs/
│   ├── system/
│   ├── backend/
│   └── frontend/
└── docker-compose.yml
```
