import { create } from "zustand";
import type { Quaternion } from "./attitudeStore";

export interface ComparisonFrame {
  t: number;
  truth: { roll: number; pitch: number; yaw: number; q: Quaternion };
  triad: { roll: number; pitch: number; yaw: number; err: number | null } | null;
  ekf: { roll: number; pitch: number; yaw: number; err: number; q: Quaternion };
}

const MAX_BUFFER = 6000;

interface ComparisonState {
  buffer: ComparisonFrame[];
  push: (f: ComparisonFrame) => void;
  clear: () => void;
}

export const useComparisonStore = create<ComparisonState>((set, get) => ({
  buffer: [],
  push: (f) => {
    const buf = get().buffer;
    const next = buf.length >= MAX_BUFFER ? buf.slice(-MAX_BUFFER + 1) : buf.slice();
    next.push(f);
    set({ buffer: next });
  },
  clear: () => set({ buffer: [] }),
}));
