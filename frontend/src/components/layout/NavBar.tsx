import { Link } from "@tanstack/react-router";
import { Box, Activity, GitCompare, Gauge } from "lucide-react";
import { cn } from "@/lib/utils";

const items: Array<{ to: "/" | "/telemetry" | "/comparison" | "/performance"; label: string; icon: any; exact?: boolean }> = [
  { to: "/", label: "Visualization", icon: Box, exact: true },
  { to: "/telemetry", label: "Telemetry", icon: Activity },
  { to: "/comparison", label: "Comparison", icon: GitCompare },
  { to: "/performance", label: "Performance", icon: Gauge },
];

export function NavBar() {
  return (
    <aside className="hidden w-56 shrink-0 flex-col border-r border-border bg-card/30 md:flex">
      <nav className="flex flex-col gap-1 p-3">
        <div className="px-2 pb-2 pt-1 text-[10px] font-semibold uppercase tracking-[0.2em] text-muted-foreground">
          Views
        </div>
        {items.map(({ to, label, icon: Icon, exact }) => (
          <Link
            key={to}
            to={to}
            activeOptions={{ exact: exact ?? false }}
            className={cn(
              "group flex items-center gap-2.5 rounded-md px-2.5 py-2 text-sm text-muted-foreground transition-colors hover:bg-accent/50 hover:text-foreground",
            )}
            activeProps={{
              className:
                "bg-primary/15 text-primary hover:bg-primary/20 hover:text-primary [&>svg]:text-primary",
            }}
          >
            <Icon className="h-4 w-4 text-muted-foreground group-hover:text-foreground" />
            {label}
          </Link>
        ))}
      </nav>
      <div className="mt-auto p-3 text-[10px] leading-relaxed text-muted-foreground">
        <div className="rounded-md border border-border/60 bg-card/40 p-2.5">
          <div className="mb-1 font-semibold uppercase tracking-wider text-foreground/80">Channels</div>
          <div className="font-mono">/ws/attitude</div>
          <div className="font-mono">/ws/telemetry</div>
        </div>
      </div>
    </aside>
  );
}
