import { Play, Square, RotateCcw, Radio } from "lucide-react";
import { useSimulationStore, type AppMode } from "@/stores/simulationStore";
import { simulationApi } from "@/lib/api/client";
import { StatusBadge } from "@/components/ui/StatusBadge";

function fmtElapsed(s: number) {
  const m = Math.floor(s / 60);
  const sec = s - m * 60;
  return `${m.toString().padStart(2, "0")}:${sec.toFixed(2).padStart(5, "0")}`;
}

const MODES: { value: AppMode; label: string; title: string }[] = [
  { value: "demo", label: "Demo", title: "Synthetic data — no backend required" },
  { value: "simulate", label: "Simulate", title: "Physics simulation running on the backend" },
  { value: "live", label: "Live", title: "Live TLE orbit data from the backend" },
];

export function SimulationBar() {
  const { mode, runState, elapsed, wsState, setMode, setRunState, setElapsed } =
    useSimulationStore();

  const isDemo = mode === "demo";
  // Allow controls when in demo mode OR when WebSocket is connected
  const disabled = !isDemo && wsState !== "connected";

  const switchMode = async (next: AppMode) => {
    if (next === mode) return;
    setMode(next);
    if (next !== "demo") {
      try {
        await simulationApi.configure({ mode: next === "live" ? "live" : "simulation" });
      } catch {
        /* backend may not be running yet */
      }
    }
  };

  const handle = async (action: "start" | "stop" | "reset") => {
    if (isDemo) {
      if (action === "start") setRunState("RUNNING");
      if (action === "stop") setRunState("STOPPED");
      if (action === "reset") {
        setRunState("RESET");
        setElapsed(0);
        setTimeout(() => setRunState("STOPPED"), 250);
      }
      return;
    }
    try {
      await simulationApi[action]();
    } catch {
      /* status polling will catch up */
    }
  };

  return (
    <header className="sticky top-0 z-30 border-b border-border bg-background/85 backdrop-blur-md">
      <div className="flex h-14 items-center justify-between gap-4 px-5">
        {/* Logo */}
        <div className="flex items-center gap-3">
          <div className="flex h-8 w-8 items-center justify-center rounded-md bg-primary/15 text-primary">
            <Radio className="h-4 w-4" />
          </div>
          <div className="leading-tight">
            <div className="text-[11px] uppercase tracking-[0.18em] text-muted-foreground">
              CubeSat ADCS
            </div>
            <div className="text-sm font-semibold">Attitude Estimation Dashboard</div>
          </div>
        </div>

        <div className="flex items-center gap-3">
          {/* Status badges */}
          <StatusBadge state={runState} />
          <div className="hidden items-center gap-1.5 rounded-md border border-border bg-card/50 px-2.5 py-1 font-mono text-xs text-muted-foreground sm:flex">
            <span className="text-[10px] uppercase tracking-wider">T+</span>
            <span className="text-foreground">{fmtElapsed(elapsed)}</span>
          </div>
          <StatusBadge
            state={wsState}
            label={
              wsState === "connected" ? "LINK UP" : wsState === "connecting" ? "LINKING" : "LINK DOWN"
            }
          />

          {/* Simulation controls */}
          <div className="ml-1 flex items-center gap-1">
            <button
              onClick={() => handle("start")}
              disabled={disabled || runState === "RUNNING"}
              className="inline-flex items-center gap-1.5 rounded-md bg-success/20 px-3 py-1.5 text-xs font-medium text-success transition-colors hover:bg-success/30 disabled:cursor-not-allowed disabled:opacity-40"
            >
              <Play className="h-3.5 w-3.5" /> Start
            </button>
            <button
              onClick={() => handle("stop")}
              disabled={disabled || runState !== "RUNNING"}
              className="inline-flex items-center gap-1.5 rounded-md bg-destructive/20 px-3 py-1.5 text-xs font-medium text-destructive transition-colors hover:bg-destructive/30 disabled:cursor-not-allowed disabled:opacity-40"
            >
              <Square className="h-3.5 w-3.5" /> Stop
            </button>
            <button
              onClick={() => handle("reset")}
              disabled={disabled}
              className="inline-flex items-center gap-1.5 rounded-md bg-secondary px-3 py-1.5 text-xs font-medium text-secondary-foreground transition-colors hover:bg-secondary/80 disabled:cursor-not-allowed disabled:opacity-40"
            >
              <RotateCcw className="h-3.5 w-3.5" /> Reset
            </button>
          </div>

          {/* 3-way mode selector */}
          <div className="ml-2 flex items-center rounded-md border border-border bg-card/50 p-0.5">
            {MODES.map((m) => (
              <button
                key={m.value}
                title={m.title}
                onClick={() => switchMode(m.value)}
                className={`rounded px-2.5 py-1 text-xs font-medium transition-colors ${
                  mode === m.value
                    ? "bg-primary text-primary-foreground"
                    : "text-muted-foreground hover:text-foreground"
                }`}
              >
                {m.label}
              </button>
            ))}
          </div>
        </div>
      </div>
    </header>
  );
}
