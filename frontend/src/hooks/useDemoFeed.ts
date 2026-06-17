import { useEffect, useRef } from "react";
import { useSimulationStore } from "@/stores/simulationStore";
import { useAttitudeStore, type AttitudeFrame } from "@/stores/attitudeStore";
import { useTelemetryStore } from "@/stores/telemetryStore";
import { useComparisonStore } from "@/stores/comparisonStore";
import { usePerformanceStore } from "@/stores/performanceStore";

// Quaternion helpers
function quatFromEuler(roll: number, pitch: number, yaw: number): [number, number, number, number] {
  const cr = Math.cos(roll / 2),
    sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2),
    sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2),
    sy = Math.sin(yaw / 2);
  return [
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ];
}

function eulerFromQuat(q: [number, number, number, number]) {
  const [w, x, y, z] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sinp = 2 * (w * y - z * x);
  const pitch = Math.abs(sinp) >= 1 ? (Math.sign(sinp) * Math.PI) / 2 : Math.asin(sinp);
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return { roll, pitch, yaw };
}

function rad2deg(r: number) {
  return (r * 180) / Math.PI;
}

// Angular error between two unit quaternions in degrees
function quatAngularError(a: [number, number, number, number], b: [number, number, number, number]) {
  const dot = Math.min(1, Math.max(-1, Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3])));
  return rad2deg(2 * Math.acos(dot));
}

function gauss(sigma: number) {
  const u = Math.max(Math.random(), 1e-9);
  const v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v) * sigma;
}

/**
 * Synthetic data feeder — generates plausible attitude + telemetry frames
 * when no backend is connected (demoMode = true) and simulation is RUNNING.
 * Drives all stores so the dashboard is fully functional in preview.
 */
export function useDemoFeed() {
  const mode = useSimulationStore((s) => s.mode);
  const runState = useSimulationStore((s) => s.runState);
  const setElapsed = useSimulationStore((s) => s.setElapsed);
  const setWsState = useSimulationStore((s) => s.setWsState);
  const setAttitude = useAttitudeStore((s) => s.set);
  const pushTelem = useTelemetryStore((s) => s.push);
  const pushCmp = useComparisonStore((s) => s.push);
  const pushCov = usePerformanceStore((s) => s.pushCovariance);
  const setSummary = usePerformanceStore((s) => s.setSummary);

  const startRef = useRef<number | null>(null);
  const errAccumRef = useRef({
    ekfRoll: [] as number[],
    ekfPitch: [] as number[],
    ekfYaw: [] as number[],
    triadRoll: [] as number[],
    triadPitch: [] as number[],
    triadYaw: [] as number[],
    ekfErr: [] as number[],
    triadErr: [] as number[],
  });

  useEffect(() => {
    if (mode !== "demo") return;
    setWsState(runState === "RUNNING" ? "connected" : "disconnected");
  }, [mode, runState, setWsState]);

  useEffect(() => {
    if (mode !== "demo" || runState !== "RUNNING") {
      startRef.current = null;
      return;
    }
    startRef.current = performance.now();
    let last = performance.now();
    let raf = 0;
    // 50 Hz tick — within spec target, keeps CPU sane.
    const interval = 1000 / 50;
    let acc = 0;

    const tick = () => {
      raf = requestAnimationFrame(tick);
      const now = performance.now();
      const dt = now - last;
      last = now;
      acc += dt;
      while (acc >= interval) {
        acc -= interval;
        const t = (now - (startRef.current ?? now)) / 1000;
        setElapsed(t);

        // Ground truth: slow precessing rotation
        const roll = 0.5 * Math.sin(0.4 * t);
        const pitch = 0.4 * Math.sin(0.3 * t + 0.6);
        const yaw = 0.25 * t; // continuous spin
        const gtQ = quatFromEuler(roll, pitch, yaw);

        // EKF: small noise, low bias
        const ekfRoll = roll + gauss(0.01);
        const ekfPitch = pitch + gauss(0.01);
        const ekfYaw = yaw + gauss(0.012);
        const ekfQ = quatFromEuler(ekfRoll, ekfPitch, ekfYaw);
        const ekfErr = quatAngularError(gtQ, ekfQ);

        // TRIAD: noisier, occasionally null (singularity)
        const singularity = Math.random() < 0.02;
        let triadFrame: any = null;
        if (!singularity) {
          const tRoll = roll + gauss(0.05);
          const tPitch = pitch + gauss(0.05);
          const tYaw = yaw + gauss(0.06);
          const tQ = quatFromEuler(tRoll, tPitch, tYaw);
          const tErr = quatAngularError(gtQ, tQ);
          triadFrame = {
            quaternion: tQ,
            euler: { roll: tRoll, pitch: tPitch, yaw: tYaw },
            angular_error_deg: tErr,
          };
        }

        const frame: AttitudeFrame = {
          t,
          ground_truth: { quaternion: gtQ, euler: { roll, pitch, yaw } },
          triad: triadFrame,
          ekf: {
            quaternion: ekfQ,
            euler: { roll: ekfRoll, pitch: ekfPitch, yaw: ekfYaw },
            angular_error_deg: ekfErr,
          },
        };
        setAttitude(frame);

        // Telemetry: gyro (rad/s) is derivative-ish + noise
        pushTelem({
          t,
          gyro: [
            0.2 * Math.cos(0.4 * t) + gauss(0.02),
            0.12 * Math.cos(0.3 * t + 0.6) + gauss(0.02),
            0.25 + gauss(0.02),
          ],
          accel: [
            Math.sin(yaw) + gauss(0.03),
            -Math.cos(yaw) * Math.sin(roll) + gauss(0.03),
            Math.cos(yaw) * Math.cos(roll) + gauss(0.03),
          ],
          mag: [
            Math.cos(yaw) * Math.cos(pitch) + gauss(0.04),
            Math.sin(yaw) + gauss(0.04),
            -Math.sin(pitch) + gauss(0.04),
          ],
        });

        pushCmp({
          t,
          truth: { roll, pitch, yaw, q: gtQ },
          triad: triadFrame
            ? {
                roll: triadFrame.euler.roll,
                pitch: triadFrame.euler.pitch,
                yaw: triadFrame.euler.yaw,
                err: triadFrame.angular_error_deg,
              }
            : null,
          ekf: { roll: ekfRoll, pitch: ekfPitch, yaw: ekfYaw, err: ekfErr, q: ekfQ },
        });

        // Covariance trace: exponentially decaying toward steady state
        const cov = 0.5 * Math.exp(-t / 4) + 0.005 + Math.random() * 0.001;
        pushCov({ t, trace: cov });

        // Accumulate for RMSE
        const acc2 = errAccumRef.current;
        acc2.ekfRoll.push(ekfRoll - roll);
        acc2.ekfPitch.push(ekfPitch - pitch);
        acc2.ekfYaw.push(ekfYaw - yaw);
        acc2.ekfErr.push(ekfErr);
        if (triadFrame) {
          acc2.triadRoll.push(triadFrame.euler.roll - roll);
          acc2.triadPitch.push(triadFrame.euler.pitch - pitch);
          acc2.triadYaw.push(triadFrame.euler.yaw - yaw);
          acc2.triadErr.push(triadFrame.angular_error_deg);
        }
      }
    };
    raf = requestAnimationFrame(tick);

    // Update perf summary every 1s
    const summaryTimer = setInterval(() => {
      const a = errAccumRef.current;
      const rmse = (arr: number[]) =>
        arr.length ? rad2deg(Math.sqrt(arr.reduce((s, x) => s + x * x, 0) / arr.length)) : 0;
      const mean = (arr: number[]) => (arr.length ? arr.reduce((s, x) => s + x, 0) / arr.length : 0);
      const ekfMean = mean(a.ekfErr);
      const triadMean = mean(a.triadErr);
      setSummary({
        ekf: {
          rmse: { roll: rmse(a.ekfRoll), pitch: rmse(a.ekfPitch), yaw: rmse(a.ekfYaw) },
          mean_error_deg: ekfMean,
        },
        triad: {
          rmse: { roll: rmse(a.triadRoll), pitch: rmse(a.triadPitch), yaw: rmse(a.triadYaw) },
          mean_error_deg: triadMean,
        },
        improvement_ratio: ekfMean > 0 ? triadMean / ekfMean : 1,
      });
    }, 1000);

    return () => {
      cancelAnimationFrame(raf);
      clearInterval(summaryTimer);
    };
  }, [mode, runState, setAttitude, pushTelem, pushCmp, pushCov, setSummary, setElapsed]);
}
