import { createFileRoute } from "@tanstack/react-router";
import { EulerOverlayChart } from "@/components/charts/EulerOverlayChart";
import { QuaternionChart } from "@/components/charts/QuaternionChart";
import { ErrorChart } from "@/components/charts/ErrorChart";
import { Panel } from "@/components/panels/Panel";
import { LiveUnavailableMask } from "@/components/ui/LiveUnavailableMask";

export const Route = createFileRoute("/comparison")({
  head: () => ({
    meta: [
      { title: "Comparison — CubeSat ADCS" },
      { name: "description", content: "TRIAD vs EKF vs Ground Truth comparison plots." },
    ],
  }),
  component: ComparisonPage,
});

function ComparisonPage() {
  return (
    <div className="space-y-4 p-4">
      <LiveUnavailableMask>
        <Panel title="Euler Angle Overlay (Truth · TRIAD · EKF)">
          <div className="grid gap-3 lg:grid-cols-3">
            <EulerOverlayChart axis="roll" />
            <EulerOverlayChart axis="pitch" />
            <EulerOverlayChart axis="yaw" />
          </div>
        </Panel>
      </LiveUnavailableMask>

      <LiveUnavailableMask>
        <Panel title="Quaternion Components — Truth vs EKF">
          <div className="grid gap-3 sm:grid-cols-2 lg:grid-cols-4">
            <QuaternionChart idx={0} />
            <QuaternionChart idx={1} />
            <QuaternionChart idx={2} />
            <QuaternionChart idx={3} />
          </div>
        </Panel>
      </LiveUnavailableMask>

      <LiveUnavailableMask>
        <Panel title="Angular Error vs Truth">
          <ErrorChart />
        </Panel>
      </LiveUnavailableMask>
    </div>
  );
}
