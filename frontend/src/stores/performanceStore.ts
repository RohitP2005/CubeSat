import { create } from "zustand";

export interface PerformanceSummary {
  ekf: {
    rmse: { roll: number; pitch: number; yaw: number };
    mean_error_deg: number;
  };
  triad: {
    rmse: { roll: number; pitch: number; yaw: number };
    mean_error_deg: number;
  };
  improvement_ratio: number;
}

export interface CovarianceSample {
  t: number;
  trace: number;
}

interface PerformanceState {
  summary: PerformanceSummary | null;
  covariance: CovarianceSample[];
  setSummary: (s: PerformanceSummary) => void;
  pushCovariance: (s: CovarianceSample) => void;
  clear: () => void;
}

const MAX = 6000;

export const usePerformanceStore = create<PerformanceState>((set, get) => ({
  summary: null,
  covariance: [],
  setSummary: (summary) => set({ summary }),
  pushCovariance: (s) => {
    const c = get().covariance;
    const next = c.length >= MAX ? c.slice(-MAX + 1) : c.slice();
    next.push(s);
    set({ covariance: next });
  },
  clear: () => set({ covariance: [], summary: null }),
}));
