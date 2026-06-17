import { useComparisonStore } from "@/stores/comparisonStore";
import { PlotlyChart } from "./PlotlyChart";

export function ErrorChart() {
  const buffer = useComparisonStore((s) => s.buffer);
  const win = 30;
  const tNow = buffer.length ? buffer[buffer.length - 1].t : 0;
  const cutoff = tNow - win;
  const slice = buffer.filter((f) => f.t >= cutoff);

  const data = [
    {
      x: slice.map((f) => f.t),
      y: slice.map((f) => (f.triad ? f.triad.err : null)),
      type: "scattergl" as const,
      mode: "lines" as const,
      name: "TRIAD error",
      connectgaps: false,
      line: { color: "#fbbf24", width: 1.5 },
    },
    {
      x: slice.map((f) => f.t),
      y: slice.map((f) => f.ekf.err),
      type: "scattergl" as const,
      mode: "lines" as const,
      name: "EKF error",
      line: { color: "#60a5fa", width: 1.5 },
    },
    {
      x: [cutoff, tNow],
      y: [2, 2],
      type: "scatter" as const,
      mode: "lines" as const,
      name: "2° target",
      line: { color: "#ef4444", width: 1.2, dash: "dash" },
    },
  ];

  return (
    <PlotlyChart
      data={data}
      layout={{
        xaxis: { title: "t (s)", range: [cutoff, tNow] },
        yaxis: { title: "angular error (deg)", rangemode: "tozero" },
        legend: { orientation: "h", y: 1.18 },
        margin: { l: 56, r: 16, t: 24, b: 40 },
      }}
      style={{ width: "100%", height: 260 }}
    />
  );
}
