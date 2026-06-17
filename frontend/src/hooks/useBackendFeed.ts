import { useCallback, useEffect, useRef } from "react";
import { useSimulationStore } from "@/stores/simulationStore";
import { useAttitudeStore, type AttitudeFrame } from "@/stores/attitudeStore";
import { useTelemetryStore } from "@/stores/telemetryStore";
import { useComparisonStore } from "@/stores/comparisonStore";
import { usePerformanceStore } from "@/stores/performanceStore";
import { useWebSocket } from "@/hooks/useWebSocket";
import { wsBaseURL, performanceApi } from "@/lib/api/client";

/**
 * Active for simulate and live modes.
 * Subscribes to both WS channels, fans incoming frames into every store,
 * and polls /performance/summary every 2 s for aggregated metrics.
 */
export function useBackendFeed() {
  const mode = useSimulationStore((s) => s.mode);

  const setAttitude = useAttitudeStore((s) => s.set);
  const pushTelem = useTelemetryStore((s) => s.push);
  const pushCmp = useComparisonStore((s) => s.push);
  const pushCov = usePerformanceStore((s) => s.pushCovariance);
  const setSummary = usePerformanceStore((s) => s.setSummary);

  const active = mode === "simulate" || mode === "live";

  // --- attitude WebSocket ---
  const handleAttitude = useCallback(
    (msg: any) => {
      if (!msg || typeof msg.t !== "number") return;

      const frame: AttitudeFrame = {
        t: msg.t,
        ground_truth: msg.ground_truth,
        triad: msg.triad ?? null,
        ekf: {
          quaternion: msg.ekf.quaternion,
          euler: msg.ekf.euler,
          angular_error_deg: msg.ekf.angular_error_deg ?? null,
        },
      };
      setAttitude(frame);

      // Feed comparison store
      pushCmp({
        t: msg.t,
        truth: {
          roll: msg.ground_truth.euler.roll,
          pitch: msg.ground_truth.euler.pitch,
          yaw: msg.ground_truth.euler.yaw,
          q: msg.ground_truth.quaternion,
        },
        triad: msg.triad
          ? {
              roll: msg.triad.euler.roll,
              pitch: msg.triad.euler.pitch,
              yaw: msg.triad.euler.yaw,
              err: msg.triad.angular_error_deg ?? null,
            }
          : null,
        ekf: {
          roll: msg.ekf.euler.roll,
          pitch: msg.ekf.euler.pitch,
          yaw: msg.ekf.euler.yaw,
          err: msg.ekf.angular_error_deg ?? 0,
          q: msg.ekf.quaternion,
        },
      });

      // Covariance trace (backend includes it in ekf object)
      if (typeof msg.ekf.covariance_trace === "number") {
        pushCov({ t: msg.t, trace: msg.ekf.covariance_trace });
      }
    },
    [setAttitude, pushCmp, pushCov],
  );

  // --- telemetry WebSocket ---
  const handleTelemetry = useCallback(
    (msg: any) => {
      if (!msg || typeof msg.t !== "number") return;
      pushTelem({ t: msg.t, gyro: msg.gyro, accel: msg.accel, mag: msg.mag });
    },
    [pushTelem],
  );

  useWebSocket(active ? `${wsBaseURL}/ws/attitude` : null, handleAttitude);
  useWebSocket(active ? `${wsBaseURL}/ws/telemetry` : null, handleTelemetry);

  // --- performance polling ---
  const summaryTimerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  useEffect(() => {
    if (!active) {
      if (summaryTimerRef.current !== null) {
        clearInterval(summaryTimerRef.current);
        summaryTimerRef.current = null;
      }
      return;
    }

    const fetch = async () => {
      try {
        const { data } = await performanceApi.summary();
        setSummary({
          ekf: {
            rmse: {
              roll: data.ekf.rmse_roll ?? data.ekf.rmse?.roll ?? 0,
              pitch: data.ekf.rmse_pitch ?? data.ekf.rmse?.pitch ?? 0,
              yaw: data.ekf.rmse_yaw ?? data.ekf.rmse?.yaw ?? 0,
            },
            mean_error_deg: data.ekf.mean_error_deg ?? 0,
          },
          triad: {
            rmse: {
              roll: data.triad.rmse_roll ?? data.triad.rmse?.roll ?? 0,
              pitch: data.triad.rmse_pitch ?? data.triad.rmse?.pitch ?? 0,
              yaw: data.triad.rmse_yaw ?? data.triad.rmse?.yaw ?? 0,
            },
            mean_error_deg: data.triad.mean_error_deg ?? 0,
          },
          improvement_ratio: data.improvement_ratio ?? 1,
        });
      } catch {
        /* backend not yet available — silently skip */
      }
    };

    fetch();
    summaryTimerRef.current = setInterval(fetch, 2000);
    return () => {
      if (summaryTimerRef.current !== null) {
        clearInterval(summaryTimerRef.current);
        summaryTimerRef.current = null;
      }
    };
  }, [active, setSummary]);
}
