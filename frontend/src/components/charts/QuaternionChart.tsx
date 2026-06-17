import { useComparisonStore } from "@/stores/comparisonStore";
import { PlotlyChart } from "./PlotlyChart";

type Idx = 0 | 1 | 2 | 3;
const names = ["q₀", "q₁", "q₂", "q₃"];

export function QuaternionChart({ idx }: { idx: Idx }) {
  const buffer = useComparisonStore((s) => s.buffer);
  const win = 30;
  const tNow = buffer.length ? buffer[buffer.length - 1].t : 0;
  const cutoff = tNow - win;
  const slice = buffer.filter((f) => f.t >= cutoff);

  const data = [
    {
      x: slice.map((f) => f.t),
      y: slice.map((f) => f.truth.q[idx]),
      type: "scattergl" as const,
      mode: "lines" as const,
      name: "Truth",
      line: { color: "#e2e8f0", width: 2 },
    },
    {
      x: slice.map((f) => f.t),
      y: slice.map((f) => f.ekf.q[idx]),
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
        title: { text: names[idx], font: { size: 12 }, x: 0, xanchor: "left" },
        xaxis: { title: "t (s)", range: [cutoff, tNow] },
        yaxis: { title: "" },
        margin: { l: 48, r: 16, t: 30, b: 36 },
        showlegend: false,
      }}
      style={{ width: "100%", height: 180 }}
    />
  );
}
