import { create } from "zustand";

export type Quaternion = [number, number, number, number]; // [w, x, y, z]
export type Euler = { roll: number; pitch: number; yaw: number };

export interface AttitudeSource {
  quaternion: Quaternion;
  euler: Euler;
  angular_error_deg?: number | null;
}

export interface AttitudeFrame {
  t: number;
  ground_truth: AttitudeSource;
  triad: AttitudeSource | null;
  ekf: AttitudeSource;
}

interface AttitudeState {
  latest: AttitudeFrame | null;
  set: (f: AttitudeFrame) => void;
}

export const useAttitudeStore = create<AttitudeState>((set) => ({
  latest: null,
  set: (latest) => set({ latest }),
}));
