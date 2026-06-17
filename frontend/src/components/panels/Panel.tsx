import { cn } from "@/lib/utils";

type Props = {
  title: string;
  children: React.ReactNode;
  className?: string;
  actions?: React.ReactNode;
};

export function Panel({ title, children, className, actions }: Props) {
  return (
    <div className={cn("rounded-lg border border-border bg-card/40 backdrop-blur-sm", className)}>
      <div className="flex items-center justify-between border-b border-border/70 px-4 py-2.5">
        <div className="text-[11px] font-semibold uppercase tracking-[0.18em] text-muted-foreground">
          {title}
        </div>
        {actions}
      </div>
      <div className="p-4">{children}</div>
    </div>
  );
}
