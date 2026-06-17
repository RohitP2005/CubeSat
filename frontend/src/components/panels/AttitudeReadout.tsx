import { useAttitudeStore } from "@/stores/attitudeStore";

const rad2deg = (r: number) => (r * 180) / Math.PI;
function fmt(n: number | undefined | null, d = 2) {
  if (n === undefined || n === null || Number.isNaN(n)) return "—";
  return n.toFixed(d);
}

type Source = "ekf" | "triad" | "ground_truth";

export function AttitudeReadout({ source }: { source: Source }) {
  const frame = useAttitudeStore((s) => s.latest);
  const data =
    source === "ekf"
      ? frame?.ekf
      : source === "triad"
        ? frame?.triad
        : frame?.ground_truth;

  const e = data?.euler;

  const labels: [string, string, number | undefined | null][] = [
    ["Roll", "X", e ? rad2deg(e.roll) : null],
    ["Pitch", "Y", e ? rad2deg(e.pitch) : null],
    ["Yaw", "Z", e ? rad2deg(e.yaw) : null],
  ];

  return (
    <div className="grid grid-cols-3 gap-2">
      {labels.map(([label, axis, v]) => (
        <div key={label} className="rounded-md border border-border/70 bg-background/40 p-3">
          <div className="flex items-center justify-between">
            <span className="text-[10px] font-semibold uppercase tracking-[0.18em] text-muted-foreground">
              {label}
            </span>
            <span
              className="text-[10px] font-mono"
              style={{
                color:
                  axis === "X" ? "var(--color-axis-x)" : axis === "Y" ? "var(--color-axis-y)" : "var(--color-axis-z)",
              }}
            >
              {axis}
            </span>
          </div>
          <div className="mt-1 font-mono text-xl tabular-nums text-foreground">
            {fmt(v, 2)}
            <span className="ml-1 text-xs text-muted-foreground">°</span>
          </div>
        </div>
      ))}
    </div>
  );
}
