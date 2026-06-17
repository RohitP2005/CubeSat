import { cn } from "@/lib/utils";

type Props = {
  state: "RUNNING" | "STOPPED" | "RESET" | "connected" | "connecting" | "disconnected";
  label?: string;
};

const styles: Record<string, string> = {
  RUNNING: "bg-success/15 text-success border-success/30",
  STOPPED: "bg-muted-foreground/10 text-muted-foreground border-muted-foreground/20",
  RESET: "bg-warning/15 text-warning border-warning/30",
  connected: "bg-success/15 text-success border-success/30",
  connecting: "bg-warning/15 text-warning border-warning/30",
  disconnected: "bg-destructive/15 text-destructive border-destructive/30",
};

const dot: Record<string, string> = {
  RUNNING: "bg-success animate-pulse",
  STOPPED: "bg-muted-foreground",
  RESET: "bg-warning",
  connected: "bg-success animate-pulse",
  connecting: "bg-warning animate-pulse",
  disconnected: "bg-destructive",
};

export function StatusBadge({ state, label }: Props) {
  return (
    <span
      className={cn(
        "inline-flex items-center gap-2 rounded-full border px-2.5 py-1 text-xs font-medium uppercase tracking-wider",
        styles[state],
      )}
    >
      <span className={cn("h-1.5 w-1.5 rounded-full", dot[state])} />
      {label ?? state}
    </span>
  );
}
