import { useEffect, useRef } from "react";
import { useSimulationStore } from "@/stores/simulationStore";

type Handler = (msg: any) => void;

export function useWebSocket(url: string | null, onMessage: Handler) {
  const setWsState = useSimulationStore((s) => s.setWsState);
  const mode = useSimulationStore((s) => s.mode);
  const retriesRef = useRef(0);
  const wsRef = useRef<WebSocket | null>(null);
  const stoppedRef = useRef(false);
  const handlerRef = useRef(onMessage);
  handlerRef.current = onMessage;

  useEffect(() => {
    if (mode === "demo" || !url) {
      return;
    }
    stoppedRef.current = false;

    const connect = () => {
      if (stoppedRef.current) return;
      setWsState("connecting");
      try {
        const ws = new WebSocket(url);
        wsRef.current = ws;
        ws.onopen = () => {
          retriesRef.current = 0;
          setWsState("connected");
        };
        ws.onmessage = (e) => {
          try {
            handlerRef.current(JSON.parse(e.data));
          } catch {
            /* ignore */
          }
        };
        ws.onerror = () => {
          /* handled in onclose */
        };
        ws.onclose = () => {
          setWsState("disconnected");
          if (stoppedRef.current) return;
          if (retriesRef.current >= 5) return;
          const delay = Math.min(1000 * 2 ** retriesRef.current, 16000);
          retriesRef.current += 1;
          setTimeout(connect, delay);
        };
      } catch {
        setWsState("disconnected");
      }
    };
    connect();

    return () => {
      stoppedRef.current = true;
      wsRef.current?.close();
    };
  }, [url, mode, setWsState]);
}
