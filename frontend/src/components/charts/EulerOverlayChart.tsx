import { useComparisonStore } from "@/stores/comparisonStore";
import { PlotlyChart } from "./PlotlyChart";

const rad2deg = (r: number) => (r * 180) / Math.PI;
type Axis = "roll" | "pitch" | "yaw";

export function EulerOverlayChart({ axis }: { axis: Axis }) {
  const buffer = useComparisonStore((s) => s.buffer);
  const win = 30;
  const tNow = buffer.length ? buffer[buffer.length - 1].t : 0;
  const cutoff = tNow - win;
  const slice = buffer.filter((f) => f.t >= cutoff);

  const data = [
    {
      x: slice.map((f) => f.t),
      y: slice.map((f) => rad2deg(f.truth[axis])),
      type: "scattergl" as const,
      mode: "lines" as const,
      name: "Ground Truth",
      line: { color: "#e2e8f0", width: 2 },
    },
    {
      x: slice.map((f) => f.t),
      y: slice.map((f) => (f.triad ? rad2deg(f.triad[axis]) : null)),
      type: "scattergl" as const,
      mode: "lines" as const,
      name: "TRIAD",
      connectgaps: false,
      line: { color: "#fbbf24", width: 1.5 },
    },
    {
      x: slice.map((f) => f.t),
      y: slice.map((f) => rad2deg(f.ekf[axis])),
      type: "scattergl" as const,
      mode: "lines" as const,
      name: "EKF",
      line: { color: "#60a5fa", width: 1.5 },
    },
  ];

  return (
    <PlotlyChart
      data={data}
      layout={{
        title: { text: axis.toUpperCase(), font: { size: 12 }, x: 0, xanchor: "left" },
        xaxis: { title: "t (s)", range: [cutoff, tNow] },
        yaxis: { title: "deg" },
        margin: { l: 56, r: 16, t: 32, b: 40 },
        legend: { orientation: "h", y: 1.18 },
      }}
      style={{ width: "100%", height: 220 }}
    />
  );
}
