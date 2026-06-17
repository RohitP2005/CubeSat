import { createFileRoute } from "@tanstack/react-router";
import { Pause, Play } from "lucide-react";
import { TelemetryChart } from "@/components/charts/TelemetryChart";
import { Panel } from "@/components/panels/Panel";
import { ToggleButton } from "@/components/ui/ToggleButton";
import { useTelemetryStore } from "@/stores/telemetryStore";

export const Route = createFileRoute("/telemetry")({
  head: () => ({
    meta: [
      { title: "Telemetry — CubeSat ADCS" },
      { name: "description", content: "Live gyroscope, accelerometer, magnetometer telemetry." },
    ],
  }),
  component: TelemetryPage,
});

function TelemetryPage() {
  const paused = useTelemetryStore((s) => s.paused);
  const setPaused = useTelemetryStore((s) => s.setPaused);
  const windowSec = useTelemetryStore((s) => s.windowSec);
  const setWindow = useTelemetryStore((s) => s.setWindow);

  const actions = (
    <div className="flex items-center gap-2">
      <div className="flex gap-1">
        {[10, 30, 60].map((w) => (
          <ToggleButton
            key={w}
            active={windowSec === w}
            onClick={() => setWindow(w as 10 | 30 | 60)}
          >
            {w}s
          </ToggleButton>
        ))}
      </div>
      <button
        onClick={() => setPaused(!paused)}
        className="inline-flex items-center gap-1.5 rounded-md border border-border bg-card/50 px-2.5 py-1.5 text-xs hover:text-foreground"
      >
        {paused ? <Play className="h-3.5 w-3.5" /> : <Pause className="h-3.5 w-3.5" />}
        {paused ? "Resume" : "Pause"}
      </button>
    </div>
  );

  return (
    <div className="space-y-4 p-4">
      <Panel title="Sensor Telemetry" actions={actions}>
        <div className="space-y-3">
          <TelemetryChart channel="gyro" />
          <TelemetryChart channel="accel" />
          <TelemetryChart channel="mag" />
        </div>
      </Panel>
    </div>
  );
}
