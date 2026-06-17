import { create } from "zustand";

export type RunState = "RUNNING" | "STOPPED" | "RESET";
export type WsState = "connected" | "connecting" | "disconnected";
export type AppMode = "demo" | "simulate" | "live";

interface SimulationState {
  mode: AppMode;
  runState: RunState;
  elapsed: number;
  wsState: WsState;
  setMode: (m: AppMode) => void;
  setRunState: (s: RunState) => void;
  setElapsed: (e: number) => void;
  setWsState: (s: WsState) => void;
}

export const useSimulationStore = create<SimulationState>((set) => ({
  mode: "demo",
  runState: "STOPPED",
  elapsed: 0,
  wsState: "disconnected",
  setMode: (mode) => set({ mode }),
  setRunState: (runState) => set({ runState }),
  setElapsed: (elapsed) => set({ elapsed }),
  setWsState: (wsState) => set({ wsState }),
}));
