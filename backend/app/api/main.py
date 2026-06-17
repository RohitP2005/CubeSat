"""
FastAPI application — CubeSat Attitude Estimation Simulator.

REST routes:
  GET   /health
  GET   /simulation/status
  POST  /simulation/start
  POST  /simulation/stop
  POST  /simulation/reset
  POST  /simulation/configure
  GET   /attitude/current
  GET   /performance/summary

WebSocket channels:
  /ws/attitude    — combined frame: ground_truth + triad + ekf
  /ws/telemetry   — raw sensor frame: gyro + accel + mag

Response shapes are aligned with the frontend TypeScript types:
  - state values are uppercase  ("RUNNING" / "STOPPED")
  - elapsed time field is       "elapsed_s"
  - attitude frames use         "t" for timestamp, nested "euler" for angles (radians)
  - telemetry frames use        "t" + array fields "gyro", "accel", "mag"
  - performance summary uses    nested "rmse.{roll,pitch,yaw}" + "improvement_ratio"
"""

from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from .simulation_engine import SimulationConfig, SimulationEngine
from .ws_manager import WebSocketManager

# ── Singletons ────────────────────────────────────────────────────────────────

ws_manager = WebSocketManager()
engine     = SimulationEngine(ws_manager)


# ── Lifespan ──────────────────────────────────────────────────────────────────

@asynccontextmanager
async def lifespan(_app: FastAPI):
    yield
    await engine.stop()


# ── App ───────────────────────────────────────────────────────────────────────

app = FastAPI(
    title="CubeSat Attitude Estimation Simulator",
    version="0.3.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:8080",
        "http://localhost:5173",
        "http://localhost:3000",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _state_upper() -> str:
    """Return engine run-state as uppercase string expected by the frontend."""
    return engine.state.upper()   # "running" → "RUNNING", "stopped" → "STOPPED"


# ── Health ────────────────────────────────────────────────────────────────────

@app.get("/health", tags=["system"])
async def health() -> dict:
    return {"status": "ok"}


# ── Simulation control ────────────────────────────────────────────────────────

@app.get("/simulation/status", tags=["simulation"])
async def simulation_status() -> dict:
    return {
        "state":      _state_upper(),
        "step_count": engine.step_count,
        "elapsed_s":  engine.elapsed_time,   # frontend field name
        "config":     engine.config.model_dump(),
    }


@app.post("/simulation/start", tags=["simulation"])
async def simulation_start() -> dict:
    await engine.start()
    return {"state": _state_upper()}


@app.post("/simulation/stop", tags=["simulation"])
async def simulation_stop() -> dict:
    await engine.stop()
    return {"state": _state_upper()}


@app.post("/simulation/reset", tags=["simulation"])
async def simulation_reset() -> dict:
    await engine.reset()
    return {"state": _state_upper(), "step_count": engine.step_count}


@app.post("/simulation/configure", tags=["simulation"])
async def simulation_configure(config: SimulationConfig) -> dict:
    await engine.configure(config)
    return {"state": _state_upper(), "config": config.model_dump()}


# ── Attitude & performance ────────────────────────────────────────────────────

@app.get("/attitude/current", tags=["attitude"])
async def attitude_current():
    frame = engine.latest_attitude
    if frame is None:
        return JSONResponse(
            status_code=404,
            content={"error": "no_data", "message": "Start the simulation first."},
        )
    return frame


@app.get("/performance/summary", tags=["attitude"])
async def performance_summary() -> dict:
    """
    Returns performance metrics shaped for the frontend PerformanceSummary type:
      ekf.rmse.{roll,pitch,yaw}  — per-axis RMSE in degrees
      ekf.mean_error_deg         — mean total angular error
      triad.*                    — same structure for TRIAD
      improvement_ratio          — triad_mean / ekf_mean
    """
    raw = engine.performance_summary
    ekf_raw   = raw.get("ekf",   {})
    triad_raw = raw.get("triad", {})

    ekf_mean   = ekf_raw.get("mean_error_deg")   or 0.0
    triad_mean = triad_raw.get("mean_error_deg")  or 0.0

    def _rmse_block(r: dict) -> dict:
        return {
            "rmse": {
                "roll":  r.get("rmse_roll"),
                "pitch": r.get("rmse_pitch"),
                "yaw":   r.get("rmse_yaw"),
            },
            "mean_error_deg": r.get("mean_error_deg"),
            "sample_count":   r.get("sample_count", 0),
        }

    return {
        "step_count":        raw.get("step_count", 0),
        "elapsed_s":         raw.get("elapsed_time", 0.0),
        "convergence_steps": raw.get("convergence_steps", 100),
        "ekf":               _rmse_block(ekf_raw),
        "triad":             _rmse_block(triad_raw),
        "improvement_ratio": (triad_mean / ekf_mean) if ekf_mean > 0 else 1.0,
    }


# ── WebSocket endpoints ───────────────────────────────────────────────────────

@app.websocket("/ws/attitude")
async def ws_attitude(ws: WebSocket) -> None:
    await ws_manager.connect(ws, "/ws/attitude")
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        ws_manager.disconnect(ws, "/ws/attitude")


@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket) -> None:
    await ws_manager.connect(ws, "/ws/telemetry")
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        ws_manager.disconnect(ws, "/ws/telemetry")
