"""
SimulationEngine — owns the simulation loop and all algorithm state.

Three data-source modes (set via SimulationConfig.mode):

  simulation  (default)
      Full physics simulation: CircularOrbit → SensorSimulator → TRIAD + EKF.
      All values are synthetic.  No network calls.

  live
      Same physics pipeline as 'simulation', but:
        • Fetches a real TLE from CelesTrak at startup (sgp4 required).
        • Each tick appends a real orbital position + IGRF B-field to the
          broadcast frame ("orbital" key).
        • TRIAD's magnetic-field reference vector is updated with the real
          IGRF direction each tick for more physically accurate estimation.
        • Sensor readings (gyro / accel / mag) are still simulated —
          live IMU telemetry is not publicly accessible.

Lifecycle:
  start()     — fetch TLE if live, then spawn asyncio loop task
  stop()      — cancel task and wait for clean exit
  reset()     — stop + rebuild all components from current config
  configure() — stop + apply new SimulationConfig + rebuild
"""

import asyncio
import contextlib
import time
from typing import Any, Literal, Optional

import numpy as np
from pydantic import BaseModel, Field

from ..ekf.ekf import AttitudeEKF
from ..ekf.quaternion import to_euler
from ..evaluation.evaluator import PerformanceEvaluator
from ..simulation.orbit import CircularOrbit
from ..simulation.sensor_sim import SensorSimulator
from ..simulation.tle_orbit import TLEOrbit
from ..triad.triad import TriadEstimator
from .ws_manager import WebSocketManager


# ── Configuration ─────────────────────────────────────────────────────────────

class SimulationConfig(BaseModel):
    dt:                float = Field(0.01,   gt=0,  description="Time step [s]")
    altitude_km:       float = Field(500.0,  gt=0,  description="Orbit altitude [km]")
    tumble_rate_deg_s: float = Field(0.1,    ge=0,  description="Tumble amplitude [deg/s]")
    sigma_gyro:        float = Field(0.005,  ge=0,  description="Gyro white noise [rad/s]")
    sigma_accel:       float = Field(0.05,   ge=0,  description="Accel noise (normalised)")
    sigma_mag:         float = Field(0.02,   ge=0,  description="Mag noise (normalised)")
    rng_seed:          int   = Field(42,            description="RNG seed for reproducibility")
    mode:              Literal["simulation", "live"] = Field(
                           "simulation",            description="Data source mode")
    norad_id:          int   = Field(25544,         description="NORAD satellite ID (live mode)")


# ── Engine ────────────────────────────────────────────────────────────────────

class SimulationEngine:
    """Singleton simulation engine, one instance per FastAPI app."""

    def __init__(self, ws_manager: WebSocketManager) -> None:
        self._ws     = ws_manager
        self._config = SimulationConfig()
        self._running = False
        self._task: Optional[asyncio.Task] = None

        # Simulation state (reset by _build)
        self._t:               float       = 0.0
        self._step_count:      int         = 0
        self._latest_attitude: Optional[dict] = None

        # Algorithm components
        self._orbit:      Optional[CircularOrbit]       = None
        self._sensor_sim: Optional[SensorSimulator]     = None
        self._ekf:        Optional[AttitudeEKF]         = None
        self._triad:      Optional[TriadEstimator]      = None
        self._evaluator:  Optional[PerformanceEvaluator] = None

        # Live-mode components
        self._tle_orbit:  Optional[TLEOrbit] = None
        self._tle_fetched: bool = False

        self._build()

    # ── Internal rebuild ──────────────────────────────────────────────────────

    def _build(self) -> None:
        """Construct fresh algorithm components from current config."""
        cfg = self._config
        self._orbit = CircularOrbit(
            altitude_km=cfg.altitude_km,
            tumble_rate_deg_s=cfg.tumble_rate_deg_s,
        )
        self._sensor_sim = SensorSimulator(
            self._orbit,
            sigma_gyro=cfg.sigma_gyro,
            sigma_accel=cfg.sigma_accel,
            sigma_mag=cfg.sigma_mag,
            rng_seed=cfg.rng_seed,
        )
        self._ekf   = AttitudeEKF(
            sigma_gyro=cfg.sigma_gyro,
            sigma_accel=cfg.sigma_accel,
            sigma_mag=cfg.sigma_mag,
        )
        self._triad     = TriadEstimator()
        self._evaluator = PerformanceEvaluator()

        # Live-mode TLE orbit
        if cfg.mode == "live":
            self._tle_orbit  = TLEOrbit(norad_id=cfg.norad_id)
            self._tle_fetched = False
        else:
            self._tle_orbit  = None
            self._tle_fetched = False

        self._t            = 0.0
        self._step_count   = 0
        self._latest_attitude = None

    # ── Public properties ─────────────────────────────────────────────────────

    @property
    def state(self) -> str:
        return "running" if self._running else "stopped"

    @property
    def step_count(self) -> int:
        return self._step_count

    @property
    def elapsed_time(self) -> float:
        return round(self._t, 6)

    @property
    def config(self) -> SimulationConfig:
        return self._config

    @property
    def latest_attitude(self) -> Optional[dict]:
        return self._latest_attitude

    @property
    def performance_summary(self) -> dict:
        s = self._evaluator.summary()
        s["elapsed_time"] = self.elapsed_time
        return s

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    async def start(self) -> None:
        if self._running:
            return
        # Attempt TLE fetch before starting the loop (best-effort, non-blocking)
        if self._config.mode == "live" and self._tle_orbit and not self._tle_fetched:
            self._tle_fetched = await self._tle_orbit.fetch_tle()
        self._running = True
        self._task = asyncio.create_task(self._loop(), name="sim-loop")

    async def stop(self) -> None:
        if not self._running:
            return
        self._running = False
        if self._task and not self._task.done():
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task
        self._task = None

    async def reset(self) -> None:
        await self.stop()
        self._build()

    async def configure(self, config: SimulationConfig) -> None:
        await self.stop()
        self._config = config
        self._build()

    # ── Simulation loop ───────────────────────────────────────────────────────

    async def _loop(self) -> None:
        while self._running:
            tick_start = time.perf_counter()
            await self._tick()
            elapsed   = time.perf_counter() - tick_start
            sleep_dur = max(0.0, self._config.dt - elapsed)
            await asyncio.sleep(sleep_dur)

    async def _tick(self) -> None:
        """One simulation step: sense → estimate → broadcast → advance clock."""
        data   = self._sensor_sim.sample(self._t, self._config.dt)
        accel  = np.array([data["ax"], data["ay"], data["az"]])
        mag    = np.array([data["mx"], data["my"], data["mz"]])
        gyro   = np.array([data["gx"], data["gy"], data["gz"]])
        q_true = data["true_q"]

        # In live mode: get real orbital frame and update TRIAD's B-ref
        orbital_frame: Optional[dict] = None
        if self._config.mode == "live" and self._tle_orbit:
            orbital_frame = self._tle_orbit.current_frame()
            if orbital_frame.get("available"):
                b_hat = self._tle_orbit.mag_field_normalized()
                if b_hat is not None:
                    self._triad.set_mag_reference(b_hat)

        # Run TRIAD (stateless, uses current reference vectors)
        triad_result = self._triad.estimate(accel, mag)

        # Run EKF (stateful)
        self._ekf.predict(gyro, self._config.dt)
        self._ekf.update(accel, mag)

        # Record errors
        q_triad_arr = triad_result["quaternion"] if triad_result is not None else None
        self._evaluator.record(self._ekf.quaternion, q_triad_arr, q_true)

        # Build combined attitude frame
        self._latest_attitude = self._build_attitude_frame(data, triad_result, orbital_frame)

        # Build telemetry frame
        telemetry = {
            "t":     round(data["timestamp"], 6),
            "gyro":  [data["gx"], data["gy"], data["gz"]],
            "accel": [data["ax"], data["ay"], data["az"]],
            "mag":   [data["mx"], data["my"], data["mz"]],
            # In live mode, these are simulated — annotate for the frontend
            "data_source": "simulated" if self._config.mode == "live" else "simulation",
        }

        await self._ws.broadcast("/ws/attitude",  self._latest_attitude)
        await self._ws.broadcast("/ws/telemetry", telemetry)

        self._t          += self._config.dt
        self._step_count += 1

    # ── Frame builder ─────────────────────────────────────────────────────────

    def _build_attitude_frame(
        self,
        data:          dict,
        triad_result:  Optional[dict],
        orbital_frame: Optional[dict] = None,
    ) -> dict:
        q_true = data["true_q"]
        r_t, p_t, y_t = to_euler(q_true)

        q_ekf = self._ekf.quaternion
        r_e, p_e, y_e = to_euler(q_ekf)

        ekf_latest   = self._evaluator.latest_ekf()
        triad_latest = self._evaluator.latest_triad()

        frame: dict[str, Any] = {
            "t":    round(data["timestamp"], 6),
            "mode": self._config.mode,
            "ground_truth": {
                "quaternion": q_true.tolist(),
                "euler": {"roll": float(r_t), "pitch": float(p_t), "yaw": float(y_t)},
            },
            "ekf": {
                "quaternion":        q_ekf.tolist(),
                "euler":             {"roll": float(r_e), "pitch": float(p_e), "yaw": float(y_e)},
                "angular_error_deg": ekf_latest["angular_error_deg"] if ekf_latest else 0.0,
                "covariance_trace":  float(self._ekf.covariance_trace),
            },
            "triad": None,
        }

        if triad_result is not None and triad_latest is not None:
            q_tr = triad_result["quaternion"]
            r_tr, p_tr, y_tr = to_euler(q_tr)
            frame["triad"] = {
                "quaternion":        q_tr.tolist(),
                "euler":             {"roll": float(r_tr), "pitch": float(p_tr), "yaw": float(y_tr)},
                "angular_error_deg": triad_latest["angular_error_deg"],
            }

        # Live mode: attach real orbital + B-field data
        if orbital_frame is not None:
            frame["orbital"] = orbital_frame

        return frame
