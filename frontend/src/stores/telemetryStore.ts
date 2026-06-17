import { create } from "zustand";

export interface TelemetryFrame {
  t: number;
  gyro: [number, number, number];
  accel: [number, number, number];
  mag: [number, number, number];
}

const MAX_BUFFER = 6000; // 60s @ 100Hz

interface TelemetryState {
  buffer: TelemetryFrame[];
  paused: boolean;
  windowSec: 10 | 30 | 60;
  push: (f: TelemetryFrame) => void;
  setPaused: (b: boolean) => void;
  setWindow: (w: 10 | 30 | 60) => void;
  clear: () => void;
}

export const useTelemetryStore = create<TelemetryState>((set, get) => ({
  buffer: [],
  paused: false,
  windowSec: 30,
  push: (f) => {
    if (get().paused) return;
    const buf = get().buffer;
    const next = buf.length >= MAX_BUFFER ? buf.slice(-MAX_BUFFER + 1) : buf.slice();
    next.push(f);
    set({ buffer: next });
  },
  setPaused: (paused) => set({ paused }),
  setWindow: (windowSec) => set({ windowSec }),
  clear: () => set({ buffer: [] }),
}));
