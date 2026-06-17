import { useEffect, useState } from "react";

type Props = {
  data: any[];
  layout?: any;
  config?: any;
  style?: React.CSSProperties;
  className?: string;
};

/**
 * Plotly is browser-only; load on the client after mount to keep SSR happy.
 */
export function PlotlyChart({ data, layout, config, style, className }: Props) {
  const [Plot, setPlot] = useState<any>(null);

  useEffect(() => {
    let mounted = true;
    import("react-plotly.js").then((m) => {
      if (mounted) setPlot(() => m.default);
    });
    return () => {
      mounted = false;
    };
  }, []);

  if (!Plot) {
    return (
      <div
        className={
          "flex items-center justify-center rounded-md border border-border bg-card/40 text-xs text-muted-foreground " +
          (className ?? "")
        }
        style={style ?? { width: "100%", height: 240 }}
      >
        Loading chart…
      </div>
    );
  }

  const baseLayout = {
    paper_bgcolor: "rgba(0,0,0,0)",
    plot_bgcolor: "rgba(0,0,0,0)",
    font: { color: "#cbd5e1", family: "ui-sans-serif, system-ui", size: 11 },
    margin: { l: 48, r: 16, t: 24, b: 36 },
    xaxis: { gridcolor: "rgba(148,163,184,0.15)", zerolinecolor: "rgba(148,163,184,0.25)" },
    yaxis: { gridcolor: "rgba(148,163,184,0.15)", zerolinecolor: "rgba(148,163,184,0.25)" },
    legend: { orientation: "h", y: -0.2 },
    ...layout,
  };

  return (
    <Plot
      data={data}
      layout={baseLayout}
      config={{ displayModeBar: false, responsive: true, ...config }}
      style={style ?? { width: "100%", height: 240 }}
      className={className}
      useResizeHandler
    />
  );
}
