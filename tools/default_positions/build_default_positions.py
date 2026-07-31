#!/usr/bin/env python3
"""Aggregate official RMUC 2026 referee data into default position tables.

For every (camp, robot_id, t) second we emit:
  - x_med/y_med: median position (kept for reference)
  - cell_gx/cell_gy: 1 m grid cell (0..27, 0..14) with the MOST samples
    (mode cell) — the "most likely place" semantics, matching RMUC-OfflineRL's
    420-cell grid. Tie-break: lowest (gx, gy).
  - x_mode/y_mode: center of that mode cell (referee metres)
  - x_p25/x_p75/y_p25/y_p75: interquartile band (reference)
  - n: sample count

Usage: build_default_positions.py --db <official.sqlite> --out <out.sqlite>
"""
import argparse
import sqlite3

import pandas as pd

MOBILE_TYPES = ["英雄", "工程", "步兵3", "步兵4", "空中", "哨兵"]
CLASS_NAMES = {"英雄": "hero", "工程": "engineer", "步兵3": "infantry3",
               "步兵4": "infantry4", "空中": "aerial", "哨兵": "sentry"}
FIELD_X, FIELD_Y = 28.0, 15.0
CELL = 1.0  # metres per grid cell -> 28 x 15 = 420 cells
PAD = 2.0

TS_SELECT = """
SELECT "时刻秒" AS t, robot_id, "机器人类型" AS rtype, 阵营 AS camp, x, y
FROM timeseries
WHERE "机器人类型" IN ({})
""".format(",".join(f"'{m}'" for m in MOBILE_TYPES))

OUT_SCHEMA = """CREATE TABLE default_positions (
    camp TEXT NOT NULL, robot_id INTEGER NOT NULL, robot_class TEXT NOT NULL,
    t INTEGER NOT NULL, x_med REAL NOT NULL, y_med REAL NOT NULL,
    cell_gx INTEGER NOT NULL, cell_gy INTEGER NOT NULL,
    x_mode REAL NOT NULL, y_mode REAL NOT NULL,
    x_p25 REAL, x_p75 REAL, y_p25 REAL, y_p75 REAL, n INTEGER NOT NULL,
    PRIMARY KEY (camp, robot_id, t))"""


def mode_cell(series_x, series_y) -> tuple[int, int, float, float]:
    """Return (gx, gy, center_x, center_y) of the most-frequent 1 m cell."""
    gx = (series_x // CELL).astype(int)
    gy = (series_y // CELL).astype(int)
    counts = pd.DataFrame({"gx": gx, "gy": gy}).value_counts()
    # value_counts is sorted desc; ties keep first (lowest gx, then gy order
    # is already deterministic from value_counts' ordering)
    best_gx, best_gy = counts.index[0]
    return (int(best_gx), int(best_gy),
            float(best_gx) * CELL + CELL / 2.0,
            float(best_gy) * CELL + CELL / 2.0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    con = sqlite3.connect(args.db)
    try:
        df = pd.read_sql(TS_SELECT, con)
    except (sqlite3.Error, pd.errors.DatabaseError) as e:
        con.close()
        raise SystemExit(f"error reading timeseries table from {args.db}: {e}")
    con.close()
    if df.empty:
        raise SystemExit("no timeseries rows found")

    df = df[(df.x != 0.0) | (df.y != 0.0)]  # drop lost tracking
    df = df[(df.x >= -PAD) & (df.x <= FIELD_X + PAD) & (df.y >= -PAD) & (df.y <= FIELD_Y + PAD)]
    df["t"] = df["t"].round().astype(int)
    df["robot_class"] = df["rtype"].map(CLASS_NAMES)

    agg = (df.groupby(["camp", "robot_id", "robot_class", "t"])["x"]
             .agg(x_med="median", x_p25=lambda s: s.quantile(0.25),
                  x_p75=lambda s: s.quantile(0.75), n="size").reset_index())
    agg_y = (df.groupby(["camp", "robot_id", "robot_class", "t"])["y"]
               .agg(y_med="median", y_p25=lambda s: s.quantile(0.25),
                    y_p75=lambda s: s.quantile(0.75)).reset_index())
    out = agg.merge(agg_y, on=["camp", "robot_id", "robot_class", "t"])

    # mode cell per group (may be slow-ish; vectorized per group)
    mode = (df.groupby(["camp", "robot_id", "robot_class", "t"])
              .apply(lambda g: pd.Series(mode_cell(g["x"], g["y"]),
                                         index=["cell_gx", "cell_gy", "x_mode", "y_mode"]),
                     include_groups=False)
              .reset_index())
    out = out.merge(mode, on=["camp", "robot_id", "robot_class", "t"])
    out = out.sort_values(["camp", "robot_id", "t"])

    ocon = sqlite3.connect(args.out)
    ocon.execute(OUT_SCHEMA)
    out.to_sql("default_positions", ocon, if_exists="append", index=False)
    ocon.commit()
    ocon.close()
    print(f"wrote {len(out)} rows to {args.out}")


if __name__ == "__main__":
    main()
