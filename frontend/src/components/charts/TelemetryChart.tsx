import { useTelemetryStore } from "@/stores/telemetryStore";
import { PlotlyChart } from "./PlotlyChart";

type Channel = "gyro" | "accel" | "mag";

const titles: Record<Channel, { title: string; unit: string; keys: [string, string, string] }> = {
  gyro: { title: "Gyroscope", unit: "rad/s", keys: ["gx", "gy", "gz"] },
  accel: { title: "Accelerometer", unit: "normalized", keys: ["ax", "ay", "az"] },
  mag: { title: "Magnetometer", unit: "normalized", keys: ["mx", "my", "mz"] },
};

const axisColors = ["#ef4444", "#22c55e", "#3b82f6"];

export function TelemetryChart({ channel }: { channel: Channel }) {
  const buffer = useTelemetryStore((s) => s.buffer);
  const win = useTelemetryStore((s) => s.windowSec);
  const meta = titles[channel];

  const tNow = buffer.length ? buffer[buffer.length - 1].t : 0;
  const cutoff = tNow - win;
  const slice = buffer.filter((f) => f.t >= cutoff);

  const data = [0, 1, 2].map((i) => ({
    x: slice.map((f) => f.t),
    y: slice.map((f) => f[channel][i]),
    type: "scattergl" as const,
    mode: "lines" as const,
    name: meta.keys[i],
    line: { color: axisColors[i], width: 1.5 },
  }));

  return (
    <PlotlyChart
      data={data}
      layout={{
        title: { text: `${meta.title} (${meta.unit})`, font: { size: 12 }, x: 0, xanchor: "left" },
        xaxis: { title: "t (s)", range: [cutoff, tNow] },
        yaxis: { title: meta.unit, autorange: true },
        margin: { l: 56, r: 16, t: 32, b: 40 },
        legend: { orientation: "h", y: 1.15 },
      }}
      style={{ width: "100%", height: 220 }}
    />
  );
}
