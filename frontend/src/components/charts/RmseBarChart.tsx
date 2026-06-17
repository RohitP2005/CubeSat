import { usePerformanceStore } from "@/stores/performanceStore";
import { PlotlyChart } from "./PlotlyChart";

export function RmseBarChart() {
  const s = usePerformanceStore((st) => st.summary);
  if (!s) {
    return (
      <div className="flex h-[280px] items-center justify-center rounded-md border border-border bg-card/30 text-xs text-muted-foreground">
        Waiting for simulation data…
      </div>
    );
  }
  const axes = ["Roll", "Pitch", "Yaw"];
  const data = [
    {
      x: axes,
      y: [s.triad.rmse.roll, s.triad.rmse.pitch, s.triad.rmse.yaw],
      name: "TRIAD",
      type: "bar" as const,
      marker: { color: "#fbbf24" },
    },
    {
      x: axes,
      y: [s.ekf.rmse.roll, s.ekf.rmse.pitch, s.ekf.rmse.yaw],
      name: "EKF",
      type: "bar" as const,
      marker: { color: "#60a5fa" },
    },
  ];
  return (
    <PlotlyChart
      data={data}
      layout={{
        barmode: "group",
        yaxis: { title: "RMSE (deg)" },
        legend: { orientation: "h", y: 1.15 },
        margin: { l: 56, r: 16, t: 24, b: 36 },
      }}
      style={{ width: "100%", height: 280 }}
    />
  );
}
