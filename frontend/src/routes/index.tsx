import { createFileRoute } from "@tanstack/react-router";
import { useState } from "react";
import { AttitudeScene } from "@/components/cubesat/AttitudeScene";
import { Panel } from "@/components/panels/Panel";
import { AttitudeReadout } from "@/components/panels/AttitudeReadout";
import { ToggleButton } from "@/components/ui/ToggleButton";
import { useAttitudeStore } from "@/stores/attitudeStore";
import { LiveUnavailableMask } from "@/components/ui/LiveUnavailableMask";

export const Route = createFileRoute("/")({
  head: () => ({
    meta: [
      { title: "Visualization — CubeSat ADCS" },
      { name: "description", content: "Real-time 3D CubeSat attitude visualization." },
    ],
  }),
  component: VisualizationPage,
});

type Source = "ekf" | "triad" | "ground_truth";

function VisualizationPage() {
  const [source, setSource] = useState<Source>("ekf");
  const [showGhost, setShowGhost] = useState(true);
  const frame = useAttitudeStore((s) => s.latest);

  return (
    <div className="grid h-full grid-rows-[1fr_auto] gap-4 p-4 lg:grid-cols-[1fr_320px] lg:grid-rows-1">
      <div className="relative min-h-[420px] overflow-hidden rounded-lg border border-border bg-card/30">
        <AttitudeScene source={source} showGhost={showGhost} />
        <div className="pointer-events-none absolute left-4 top-4 flex flex-col gap-1 text-[10px] uppercase tracking-[0.2em] text-muted-foreground">
          <span className="rounded bg-background/60 px-1.5 py-0.5 backdrop-blur-sm">
            <span style={{ color: "var(--color-axis-x)" }}>■</span> body x
          </span>
          <span className="rounded bg-background/60 px-1.5 py-0.5 backdrop-blur-sm">
            <span style={{ color: "var(--color-axis-y)" }}>■</span> body y
          </span>
          <span className="rounded bg-background/60 px-1.5 py-0.5 backdrop-blur-sm">
            <span style={{ color: "var(--color-axis-z)" }}>■</span> body z
          </span>
        </div>
      </div>

      <div className="flex flex-col gap-4">
        <Panel title="Attitude Source">
          <div className="grid grid-cols-3 gap-1.5">
            <ToggleButton active={source === "ekf"} onClick={() => setSource("ekf")}>
              EKF
            </ToggleButton>
            <ToggleButton active={source === "triad"} onClick={() => setSource("triad")}>
              TRIAD
            </ToggleButton>
            <ToggleButton
              active={source === "ground_truth"}
              onClick={() => setSource("ground_truth")}
            >
              Truth
            </ToggleButton>
          </div>
          <label className="mt-3 flex cursor-pointer items-center gap-2 text-xs text-muted-foreground">
            <input
              type="checkbox"
              checked={showGhost}
              onChange={(e) => setShowGhost(e.target.checked)}
              className="accent-primary"
            />
            Show ground-truth ghost
          </label>
        </Panel>

        <Panel title={`${source.replace("_", " ")} — Euler angles`}>
          <AttitudeReadout source={source} />
        </Panel>

        <LiveUnavailableMask>
          <Panel title="Angular Error vs Truth">
            <div className="grid grid-cols-2 gap-2">
              <ErrorTile label="TRIAD" value={frame?.triad?.angular_error_deg} color="var(--color-triad)" />
              <ErrorTile label="EKF" value={frame?.ekf.angular_error_deg} color="var(--color-ekf)" />
            </div>
          </Panel>
        </LiveUnavailableMask>
      </div>
    </div>
  );
}

function ErrorTile({
  label,
  value,
  color,
}: {
  label: string;
  value: number | null | undefined;
  color: string;
}) {
  return (
    <div className="rounded-md border border-border/70 bg-background/40 p-3">
      <div className="flex items-center justify-between">
        <span className="text-[10px] font-semibold uppercase tracking-[0.18em] text-muted-foreground">
          {label}
        </span>
        <span style={{ color }} className="text-[10px]">
          ●
        </span>
      </div>
      <div className="mt-1 font-mono text-xl tabular-nums">
        {value == null ? "—" : value.toFixed(2)}
        <span className="ml-1 text-xs text-muted-foreground">°</span>
      </div>
    </div>
  );
}
