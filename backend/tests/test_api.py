"""
Integration tests for the FastAPI simulation service.

Field names are aligned with the frontend TypeScript types:
  - state values          : "RUNNING" / "STOPPED"  (uppercase)
  - elapsed field         : "elapsed_s"
  - attitude timestamp    : "t"
  - attitude euler angles : nested "euler.{roll,pitch,yaw}" in radians
  - telemetry frame       : "t" + array fields "gyro", "accel", "mag"
  - performance summary   : "rmse.{roll,pitch,yaw}" nested + "improvement_ratio"
"""

import time
import pytest
from fastapi.testclient import TestClient

from app.api.main import app, engine


# ── Fixtures ──────────────────────────────────────────────────────────────────

@pytest.fixture
def client():
    with TestClient(app) as c:
        yield c


@pytest.fixture(autouse=True)
def fresh_engine(client):
    client.post("/simulation/stop")
    client.post("/simulation/reset")
    yield
    client.post("/simulation/stop")


def _wait_for_steps(client, min_steps: int = 5, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if client.get("/simulation/status").json().get("step_count", 0) >= min_steps:
            return
        time.sleep(0.02)
    pytest.fail(f"Simulation did not reach {min_steps} steps within {timeout}s")


# ── Health ────────────────────────────────────────────────────────────────────

def test_health(client):
    r = client.get("/health")
    assert r.status_code == 200
    assert r.json() == {"status": "ok"}


# ── Simulation status ─────────────────────────────────────────────────────────

def test_initial_status_is_stopped(client):
    data = client.get("/simulation/status").json()
    assert data["state"]      == "STOPPED"
    assert data["step_count"] == 0
    assert data["elapsed_s"]  == pytest.approx(0.0)


def test_status_contains_config(client):
    cfg = client.get("/simulation/status").json()["config"]
    assert "dt"          in cfg
    assert "sigma_gyro"  in cfg
    assert "altitude_km" in cfg


# ── Start / Stop ──────────────────────────────────────────────────────────────

def test_start_returns_running(client):
    r = client.post("/simulation/start")
    assert r.status_code == 200
    assert r.json()["state"] == "RUNNING"


def test_stop_returns_stopped(client):
    client.post("/simulation/start")
    r = client.post("/simulation/stop")
    assert r.status_code == 200
    assert r.json()["state"] == "STOPPED"


def test_start_idempotent(client):
    client.post("/simulation/start")
    r = client.post("/simulation/start")
    assert r.json()["state"] == "RUNNING"


def test_stop_idempotent(client):
    assert client.post("/simulation/stop").json()["state"] == "STOPPED"


def test_simulation_advances_step_count(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=5)
    assert client.get("/simulation/status").json()["step_count"] >= 5


def test_elapsed_s_advances(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=10)
    assert client.get("/simulation/status").json()["elapsed_s"] > 0.0


# ── Reset ─────────────────────────────────────────────────────────────────────

def test_reset_clears_step_count(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=5)
    client.post("/simulation/reset")
    data = client.get("/simulation/status").json()
    assert data["state"]      == "STOPPED"
    assert data["step_count"] == 0


def test_reset_then_restart(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=3)
    client.post("/simulation/reset")
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=3)
    assert client.get("/simulation/status").json()["state"] == "RUNNING"


# ── Configure ─────────────────────────────────────────────────────────────────

def test_configure_accepts_valid_payload(client):
    r = client.post("/simulation/configure", json={"dt": 0.02, "sigma_gyro": 0.01})
    assert r.status_code == 200
    cfg = r.json()["config"]
    assert cfg["dt"]         == pytest.approx(0.02)
    assert cfg["sigma_gyro"] == pytest.approx(0.01)


def test_configure_rejects_invalid_dt(client):
    assert client.post("/simulation/configure", json={"dt": -0.01}).status_code == 422


def test_configure_and_restart(client):
    client.post("/simulation/configure", json={"dt": 0.005, "rng_seed": 7})
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=5)
    assert client.get("/simulation/status").json()["state"] == "RUNNING"


# ── Attitude/current ──────────────────────────────────────────────────────────

def test_attitude_404_before_start(client):
    assert client.get("/attitude/current").status_code == 404


def test_attitude_available_after_start(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=1)
    assert client.get("/attitude/current").status_code == 200


def test_attitude_frame_top_keys(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=1)
    data = client.get("/attitude/current").json()
    for key in ("t", "ground_truth", "ekf"):
        assert key in data, f"Missing key: {key}"


def test_attitude_ground_truth_structure(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=1)
    gt = client.get("/attitude/current").json()["ground_truth"]
    assert "quaternion" in gt
    assert "euler"      in gt
    assert len(gt["quaternion"]) == 4
    for axis in ("roll", "pitch", "yaw"):
        assert axis in gt["euler"], f"Missing euler.{axis}"


def test_attitude_ground_truth_quaternion_is_unit(client):
    import numpy as np
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=1)
    q = client.get("/attitude/current").json()["ground_truth"]["quaternion"]
    assert abs(np.linalg.norm(q) - 1.0) < 1e-6


def test_attitude_ekf_structure(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=1)
    ekf = client.get("/attitude/current").json()["ekf"]
    for key in ("quaternion", "euler", "angular_error_deg", "covariance_trace"):
        assert key in ekf, f"Missing ekf key: {key}"
    for axis in ("roll", "pitch", "yaw"):
        assert axis in ekf["euler"], f"Missing ekf.euler.{axis}"


def test_attitude_triad_is_present_or_null(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=1)
    triad = client.get("/attitude/current").json()["triad"]
    if triad is not None:
        for key in ("quaternion", "euler", "angular_error_deg"):
            assert key in triad, f"Missing triad key: {key}"


def test_attitude_euler_in_radians(client):
    """euler values should be in radians: |roll| < 2π, |pitch| < π/2 + ε."""
    import math
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=1)
    ekf_euler = client.get("/attitude/current").json()["ekf"]["euler"]
    assert abs(ekf_euler["roll"])  < 2 * math.pi + 0.1
    assert abs(ekf_euler["pitch"]) < math.pi / 2 + 0.1


def test_attitude_t_advances(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=2)
    t1 = client.get("/attitude/current").json()["t"]
    _wait_for_steps(client, min_steps=20)
    t2 = client.get("/attitude/current").json()["t"]
    assert t2 > t1


# ── Performance summary ───────────────────────────────────────────────────────

def test_performance_summary_structure(client):
    r = client.get("/performance/summary")
    assert r.status_code == 200
    data = r.json()
    assert "step_count"        in data
    assert "convergence_steps" in data
    assert "elapsed_s"         in data
    assert "ekf"               in data
    assert "triad"             in data
    assert "improvement_ratio" in data


def test_performance_summary_zero_before_start(client):
    data = client.get("/performance/summary").json()
    assert data["step_count"] == 0
    assert data["ekf"]["sample_count"] == 0


def test_performance_summary_populated_after_run(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=50)
    client.post("/simulation/stop")
    data = client.get("/performance/summary").json()
    assert data["step_count"] >= 50
    ekf = data["ekf"]
    assert ekf["sample_count"]   >= 50
    assert ekf["mean_error_deg"] is not None
    assert ekf["rmse"]["roll"]   is not None
    assert ekf["rmse"]["pitch"]  is not None
    assert ekf["rmse"]["yaw"]    is not None


def test_ekf_mean_error_under_target(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=200)
    client.post("/simulation/stop")
    ekf = client.get("/performance/summary").json()["ekf"]
    assert ekf["mean_error_deg"] < 2.0, (
        f"EKF mean error {ekf['mean_error_deg']:.3f}° exceeds 2° target"
    )


def test_improvement_ratio_is_positive(client):
    client.post("/simulation/start")
    _wait_for_steps(client, min_steps=200)
    client.post("/simulation/stop")
    ratio = client.get("/performance/summary").json()["improvement_ratio"]
    assert ratio > 0


# ── WebSocket — attitude channel ──────────────────────────────────────────────

def test_ws_attitude_delivers_frame(client):
    with client.websocket_connect("/ws/attitude") as ws:
        client.post("/simulation/start")
        frame = ws.receive_json()
        assert "ground_truth" in frame
        assert "ekf"          in frame


def test_ws_attitude_frame_schema(client):
    import numpy as np
    with client.websocket_connect("/ws/attitude") as ws:
        client.post("/simulation/start")
        frame = ws.receive_json()
    gt  = frame["ground_truth"]
    ekf = frame["ekf"]
    assert len(gt["quaternion"]) == 4
    assert abs(np.linalg.norm(gt["quaternion"]) - 1.0) < 1e-6
    assert "euler" in gt
    assert ekf["covariance_trace"]  > 0
    assert ekf["angular_error_deg"] >= 0
    assert "euler" in ekf


def test_ws_attitude_multiple_frames(client):
    """Sequential frames must have monotonically increasing 't'."""
    with client.websocket_connect("/ws/attitude") as ws:
        client.post("/simulation/start")
        t_prev = -1.0
        for _ in range(5):
            frame = ws.receive_json()
            assert frame["t"] > t_prev
            t_prev = frame["t"]


# ── WebSocket — telemetry channel ─────────────────────────────────────────────

def test_ws_telemetry_delivers_frame(client):
    with client.websocket_connect("/ws/telemetry") as ws:
        client.post("/simulation/start")
        frame = ws.receive_json()
        for key in ("t", "gyro", "accel", "mag"):
            assert key in frame, f"Missing telemetry key: {key}"
        assert len(frame["gyro"])  == 3
        assert len(frame["accel"]) == 3
        assert len(frame["mag"])   == 3


def test_ws_telemetry_accel_physical_units(client):
    """Accelerometer magnitude should be near G_EARTH (9.81 m/s²)."""
    import numpy as np
    with client.websocket_connect("/ws/telemetry") as ws:
        client.post("/simulation/start")
        frames = [ws.receive_json() for _ in range(10)]
    mags = [np.linalg.norm(f["accel"]) for f in frames]
    assert abs(float(np.mean(mags)) - 9.81) < 1.0


def test_ws_both_channels_simultaneously(client):
    with (
        client.websocket_connect("/ws/attitude")  as ws_att,
        client.websocket_connect("/ws/telemetry") as ws_tel,
    ):
        client.post("/simulation/start")
        att_frame = ws_att.receive_json()
        tel_frame = ws_tel.receive_json()
        assert "ground_truth" in att_frame
        assert "gyro"         in tel_frame
