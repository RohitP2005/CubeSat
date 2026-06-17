import { createFileRoute } from "@tanstack/react-router";
import { Panel } from "@/components/panels/Panel";
import { MetricCard } from "@/components/panels/MetricCard";
import { CovarianceChart } from "@/components/charts/CovarianceChart";
import { RmseBarChart } from "@/components/charts/RmseBarChart";
import { usePerformanceStore } from "@/stores/performanceStore";
import { LiveUnavailableMask } from "@/components/ui/LiveUnavailableMask";

export const Route = createFileRoute("/performance")({
  head: () => ({
    meta: [
      { title: "Performance — CubeSat ADCS" },
      { name: "description", content: "EKF & TRIAD RMSE, mean error, and filter convergence." },
    ],
  }),
  component: PerformancePage,
});

function f(n: number | undefined, d = 3) {
  return n == null || Number.isNaN(n) ? "—" : n.toFixed(d);
}

function PerformancePage() {
  const s = usePerformanceStore((st) => st.summary);

  return (
    <div className="space-y-4 p-4">
      <LiveUnavailableMask>
        <div className="grid gap-3 md:grid-cols-2 xl:grid-cols-5">
        <MetricCard
          label="EKF RMSE"
          value={f(
            s ? Math.sqrt((s.ekf.rmse.roll ** 2 + s.ekf.rmse.pitch ** 2 + s.ekf.rmse.yaw ** 2) / 3) : undefined,
          )}
          unit="°"
          sub={
            <span className="font-mono">
              R {f(s?.ekf.rmse.roll)} · P {f(s?.ekf.rmse.pitch)} · Y {f(s?.ekf.rmse.yaw)}
            </span>
          }
          accent="ekf"
        />
        <MetricCard
          label="TRIAD RMSE"
          value={f(
            s
              ? Math.sqrt(
                  (s.triad.rmse.roll ** 2 + s.triad.rmse.pitch ** 2 + s.triad.rmse.yaw ** 2) / 3,
                )
              : undefined,
          )}
          unit="°"
          sub={
            <span className="font-mono">
              R {f(s?.triad.rmse.roll)} · P {f(s?.triad.rmse.pitch)} · Y {f(s?.triad.rmse.yaw)}
            </span>
          }
          accent="triad"
        />
        <MetricCard
          label="EKF Mean Error"
          value={f(s?.ekf.mean_error_deg)}
          unit="°"
          accent="ekf"
        />
        <MetricCard
          label="TRIAD Mean Error"
          value={f(s?.triad.mean_error_deg)}
          unit="°"
          accent="triad"
        />
        <MetricCard
          label="EKF vs TRIAD"
          value={s ? `${s.improvement_ratio.toFixed(2)}×` : "—"}
          sub="improvement ratio"
        />
        </div>
      </LiveUnavailableMask>

      <div className="grid gap-4 lg:grid-cols-2">
        <Panel title="EKF Covariance Trace (filter convergence)">
          <CovarianceChart />
        </Panel>
        <LiveUnavailableMask>
          <Panel title="RMSE — TRIAD vs EKF per axis">
            <RmseBarChart />
          </Panel>
        </LiveUnavailableMask>
      </div>
    </div>
  );
}
