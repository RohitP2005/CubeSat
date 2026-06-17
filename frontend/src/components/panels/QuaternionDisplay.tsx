import { useAttitudeStore } from "@/stores/attitudeStore";

type Source = "ekf" | "ground_truth";

const labels = ["q₀ (w)", "q₁ (x)", "q₂ (y)", "q₃ (z)"];

export function QuaternionDisplay({ source }: { source: Source }) {
  const frame = useAttitudeStore((s) => s.latest);
  const q = source === "ekf" ? frame?.ekf.quaternion : frame?.ground_truth.quaternion;

  return (
    <div className="grid grid-cols-4 gap-2">
      {labels.map((l, i) => (
        <div key={l} className="rounded-md border border-border/70 bg-background/40 p-2.5">
          <div className="text-[10px] font-semibold uppercase tracking-wider text-muted-foreground">
            {l}
          </div>
          <div className="mt-1 font-mono text-sm tabular-nums">
            {q ? q[i].toFixed(4) : "—"}
          </div>
        </div>
      ))}
    </div>
  );
}
