import { useSimulationStore } from "@/stores/simulationStore";
import { cn } from "@/lib/utils";

type Props = {
  children: React.ReactNode;
  className?: string;
};

/**
 * Wraps any chart or panel. In live mode, floats an overlay that reads
 * "Live value not available" — used for panels whose data depends on
 * ground-truth attitude, which is not a real measurement in live mode.
 */
export function LiveUnavailableMask({ children, className }: Props) {
  const mode = useSimulationStore((s) => s.mode);

  return (
    <div className={cn("relative", className)}>
      {children}
      {mode === "live" && (
        <div className="absolute inset-0 z-10 flex items-center justify-center rounded-lg bg-background/75 backdrop-blur-[2px]">
          <div className="flex items-center gap-2 rounded-md border border-amber-500/30 bg-amber-500/10 px-3 py-1.5 text-xs font-medium text-amber-400">
            <span className="h-1.5 w-1.5 shrink-0 rounded-full bg-amber-500" />
            Live value not available
          </div>
        </div>
      )}
    </div>
  );
}
