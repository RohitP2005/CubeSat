import { usePerformanceStore } from "@/stores/performanceStore";
import { PlotlyChart } from "./PlotlyChart";

export function CovarianceChart() {
  const cov = usePerformanceStore((s) => s.covariance);
  const data = [
    {
      x: cov.map((c) => c.t),
      y: cov.map((c) => c.trace),
      type: "scattergl" as const,
      mode: "lines" as const,
      name: "tr(P)",
      line: { color: "#60a5fa", width: 1.6 },
    },
  ];
  return (
    <PlotlyChart
      data={data}
      layout={{
        xaxis: { title: "t (s)" },
        yaxis: { title: "tr(P)", type: "log" },
        showlegend: false,
        margin: { l: 56, r: 16, t: 16, b: 40 },
      }}
      style={{ width: "100%", height: 280 }}
    />
  );
}
