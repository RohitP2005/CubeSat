import { cn } from "@/lib/utils";
import type { ButtonHTMLAttributes } from "react";

interface Props extends ButtonHTMLAttributes<HTMLButtonElement> {
  active?: boolean;
}

export function ToggleButton({ active, className, children, ...props }: Props) {
  return (
    <button
      type="button"
      className={cn(
        "rounded-md border px-3 py-1.5 text-xs font-medium transition-colors",
        active
          ? "border-primary/60 bg-primary/15 text-primary"
          : "border-border bg-card/50 text-muted-foreground hover:text-foreground hover:border-foreground/30",
        className,
      )}
      {...props}
    >
      {children}
    </button>
  );
}
