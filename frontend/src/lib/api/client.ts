import axios from "axios";

const baseURL =
  (typeof import.meta !== "undefined" && (import.meta as any).env?.VITE_API_BASE_URL) ||
  "http://localhost:8000";

export const api = axios.create({
  baseURL,
  timeout: 5000,
});

api.interceptors.response.use(
  (r) => r,
  (err) => {
    // Silent fail — UI surfaces disconnected state via WebSocket / status badge.
    return Promise.reject(err);
  },
);

export const wsBaseURL = baseURL.replace(/^http/, "ws");

export type SimulationStatus = {
  state: "RUNNING" | "STOPPED" | "RESET";
  elapsed_s: number;
};

export type SimulationConfigPatch = {
  mode?: "simulation" | "live";
  norad_id?: number;
  dt?: number;
  altitude_km?: number;
  tumble_rate_deg_s?: number;
  sigma_gyro?: number;
  sigma_accel?: number;
  sigma_mag?: number;
};

export const simulationApi = {
  start: () => api.post("/simulation/start"),
  stop: () => api.post("/simulation/stop"),
  reset: () => api.post("/simulation/reset"),
  status: () => api.get<SimulationStatus>("/simulation/status"),
  configure: (cfg: SimulationConfigPatch) => api.post("/simulation/configure", cfg),
};

export const performanceApi = {
  summary: () => api.get("/performance/summary"),
};
