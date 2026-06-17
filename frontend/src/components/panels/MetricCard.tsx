import { cn } from "@/lib/utils";

type Props = {
  label: string;
  value: string | number;
  unit?: string;
  sub?: React.ReactNode;
  accent?: "ekf" | "triad" | "neutral";
};

export function MetricCard({ label, value, unit, sub, accent = "neutral" }: Props) {
  const ring =
    accent === "ekf"
      ? "border-[color:var(--color-ekf)]/40 bg-[color:var(--color-ekf)]/5"
      : accent === "triad"
        ? "border-[color:var(--color-triad)]/40 bg-[color:var(--color-triad)]/5"
        : "border-border bg-card/40";
  return (
    <div className={cn("rounded-lg border p-4", ring)}>
      <div className="text-[10px] font-semibold uppercase tracking-[0.2em] text-muted-foreground">
        {label}
      </div>
      <div className="mt-2 flex items-baseline gap-1.5">
        <span className="font-mono text-2xl tabular-nums text-foreground">{value}</span>
        {unit && <span className="text-xs text-muted-foreground">{unit}</span>}
      </div>
      {sub && <div className="mt-1 text-xs text-muted-foreground">{sub}</div>}
    </div>
  );
}
