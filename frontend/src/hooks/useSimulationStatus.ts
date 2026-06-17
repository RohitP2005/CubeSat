import { useEffect } from "react";
import { simulationApi } from "@/lib/api/client";
import { useSimulationStore } from "@/stores/simulationStore";

export function useSimulationStatus() {
  const mode = useSimulationStore((s) => s.mode);
  const setRunState = useSimulationStore((s) => s.setRunState);
  const setElapsed = useSimulationStore((s) => s.setElapsed);

  useEffect(() => {
    if (mode === "demo") return;
    let cancelled = false;
    const poll = async () => {
      try {
        const { data } = await simulationApi.status();
        if (cancelled) return;
        setRunState(data.state);
        setElapsed(data.elapsed_s);
      } catch {
        /* ignore */
      }
    };
    poll();
    const id = setInterval(poll, 2000);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, [mode, setRunState, setElapsed]);
}
