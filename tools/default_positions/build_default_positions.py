#!/usr/bin/env python3
"""Aggregate official RMUC 2026 referee data into default position tables.

Usage: build_default_positions.py --db <official.sqlite> --out <out.sqlite>
"""
import argparse
import sqlite3

import pandas as pd

MOBILE_TYPES = ["英雄", "工程", "步兵3", "步兵4", "空中", "哨兵"]
CLASS_NAMES = {"英雄": "hero", "工程": "engineer", "步兵3": "infantry3",
               "步兵4": "infantry4", "空中": "aerial", "哨兵": "sentry"}
FIELD_X, FIELD_Y = 28.0, 15.0
PAD = 2.0

TS_SELECT = """
SELECT "时刻秒" AS t, robot_id, "机器人类型" AS rtype, 阵营 AS camp, x, y
FROM timeseries
WHERE "机器人类型" IN ({})
""".format(",".join(f"'{m}'" for m in MOBILE_TYPES))

OUT_SCHEMA = """CREATE TABLE default_positions (
    camp TEXT NOT NULL, robot_id INTEGER NOT NULL, robot_class TEXT NOT NULL,
    t INTEGER NOT NULL, x_med REAL NOT NULL, y_med REAL NOT NULL,
    x_p25 REAL, x_p75 REAL, y_p25 REAL, y_p75 REAL, n INTEGER NOT NULL,
    PRIMARY KEY (camp, robot_id, t))"""


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
    out = out.sort_values(["camp", "robot_id", "t"])

    ocon = sqlite3.connect(args.out)
    ocon.execute(OUT_SCHEMA)
    out.to_sql("default_positions", ocon, if_exists="append", index=False)
    ocon.commit()
    ocon.close()
    print(f"wrote {len(out)} rows to {args.out}")


if __name__ == "__main__":
    main()
