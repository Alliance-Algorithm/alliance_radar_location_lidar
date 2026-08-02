#!/usr/bin/env python3
"""Analyze camera-fusion location continuity from location_recorder CSV.

Metrics per robot class (hero/eng/inf3/inf4/sentry/drone):
  - active_frames / total_frames  (active = x,y != 0 AND confidence > 0)
  - gaps: inactive runs longer than --gap-threshold frames
  - jumps: consecutive active-frame displacement > --jump-threshold (after unit factor)
  - flicker: active -> inactive -> active transitions
  - x/y min/max/mean/std over active frames
"""
import argparse
import csv
import math
import sys

CLASSES = ["hero", "eng", "inf3", "inf4", "sentry", "drone"]


def is_active(r: dict, xk: str, yk: str, ck: str) -> bool:
    x, y, conf = float(r[xk]), float(r[yk]), float(r[ck])
    return x != 0.0 and y != 0.0 and conf > 0.0


def load(path: str) -> list[dict]:
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def analyze(rows: list[dict], gap_thr: int, jump_thr: float, unit: float):
    report = {}
    for cls in CLASSES:
        xk, yk, ck = f"{cls}_x", f"{cls}_y", f"{cls}_conf"
        active = [i for i, r in enumerate(rows) if is_active(r, xk, yk, ck)]

        gaps = 0
        max_gap = 0
        for a, b in zip(active, active[1:]):
            g = b - a - 1
            if g > gap_thr:
                gaps += 1
            max_gap = max(max_gap, g)

        jumps = 0
        max_jump = 0.0
        for a, b in zip(active, active[1:]):
            dx = (float(rows[b][xk]) - float(rows[a][xk])) * unit
            dy = (float(rows[b][yk]) - float(rows[a][yk])) * unit
            d = math.hypot(dx, dy)
            if d > jump_thr:
                jumps += 1
            max_jump = max(max_jump, d)

        flicker = 0
        prev_active = False
        for r in rows:
            cur_active = is_active(r, xk, yk, ck)
            if cur_active and not prev_active:
                flicker += 1
            prev_active = cur_active
        # first appearance doesn't count as a re-appearance
        flicker = max(0, flicker - 1)

        xs = [float(rows[i][xk]) * unit for i in active]
        ys = [float(rows[i][yk]) * unit for i in active]

        report[cls] = {
            "active": len(active),
            "total": len(rows),
            "active_ratio": len(active) / len(rows) if rows else 0.0,
            "gaps": gaps,
            "max_gap": max_gap,
            "jumps": jumps,
            "max_jump": max_jump,
            "flicker": flicker,
            "x_min": min(xs) if xs else None,
            "x_max": max(xs) if xs else None,
            "x_mean": sum(xs) / len(xs) if xs else None,
            "y_min": min(ys) if ys else None,
            "y_max": max(ys) if ys else None,
            "y_mean": sum(ys) / len(ys) if ys else None,
        }
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--gap-threshold", type=int, default=10)
    ap.add_argument("--jump-threshold", type=float, default=3.0)
    ap.add_argument("--unit-factor", type=float, default=0.01,
                    help="multiply x/y by this (e.g. cm->m = 0.01) before metrics")
    ap.add_argument("--plot", help="save trajectory PNG (matplotlib)")
    args = ap.parse_args()

    rows = load(args.csv)
    if not rows:
        print("empty CSV")
        sys.exit(1)

    report = analyze(rows, args.gap_threshold, args.jump_threshold, args.unit_factor)
    print(f"rows={len(rows)}  gap_thr={args.gap_threshold}  jump_thr={args.jump_threshold} "
          f"unit={args.unit_factor}")
    for cls, s in report.items():
        print(f"[{cls}] active={s['active']}/{s['total']} ({s['active_ratio']*100:.1f}%) "
              f"gaps={s['gaps']}(max {s['max_gap']}) jumps={s['jumps']}(max {s['max_jump']:.2f}) "
              f"flicker={s['flicker']}")
        if s["x_mean"] is not None:
            print(f"        x[{s['x_min']:.2f},{s['x_max']:.2f}] mean={s['x_mean']:.2f} "
                  f"y[{s['y_min']:.2f},{s['y_max']:.2f}] mean={s['y_mean']:.2f}")

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available, skipping --plot")
            sys.exit(0)
        fig, axes = plt.subplots(6, 1, figsize=(10, 18), sharex=True)
        for ax, cls in zip(axes, CLASSES):
            xk, yk, ck = f"{cls}_x", f"{cls}_y", f"{cls}_conf"
            pts = [(float(r[xk]) * args.unit_factor, float(r[yk]) * args.unit_factor)
                   for r in rows
                   if is_active(r, xk, yk, ck)]
            if pts:
                ax.plot([p[0] for p in pts], [p[1] for p in pts], ".-", ms=2)
            ax.set_title(cls)
            ax.set_aspect("equal", adjustable="datalim")
        fig.tight_layout()
        fig.savefig(args.plot)
        print(f"plot -> {args.plot}")


if __name__ == "__main__":
    main()
