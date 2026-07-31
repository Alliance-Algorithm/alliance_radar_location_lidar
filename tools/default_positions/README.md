# Default positions from official RMUC data

Offline aggregation script that converts the official RMUC 2026 regional
referee-system dataset (SQLite, ~4.0M per-second rows across 613 matches) into
a compact `default_positions.sqlite` consumed by the `radar_fusion` node as
per-match-time fallback coordinates per camp / robot class (used when a target
slot has no confirmed track).

## Data source

- Official dataset: [RMUC 2026 区域赛部分赛事数据](https://bbs.robomaster.com/article/1936220)
  (forum post; file `rmuc_2026_region_dataset.7z` → `rmuc_2026_region_dataset.sqlite`).
- Schema conventions cross-checked against [RMUC-OfflineRL](https://github.com/Harkerbest/RMUC-OfflineRL)
  (`build_dataset.py` / `schema.py`).
- The SQLite file contains tables `matches` / `timeseries` / `events` with
  Chinese column names; `game_id` links the three tables. Sample rate 1 Hz,
  one row per robot per second.

## Run

```bash
.venv/bin/python3 tools/default_positions/build_default_positions.py \
    --db /path/to/rmuc_2026_region_dataset.sqlite \
    --out /path/to/default_positions.sqlite
```

Pipeline (see `build_default_positions.py`):

1. Read `timeseries` rows for mobile units only (英雄, 工程, 步兵3, 步兵4, 空中,
   哨兵) — buildings (基地, 前哨站) are excluded.
2. Drop lost-tracking rows (`x == 0 and y == 0`).
3. Clip coordinates to the padded field box (x in [-2, 30], y in [-2, 17]).
4. Keep original field coordinates for both camps (no blue mirroring).
5. Group by `(camp, robot_id, robot_class, t)` across all games and aggregate
   median (x/y), IQR band (p25/p75) and sample count `n`.

Exits non-zero with a message (no traceback) if the `timeseries` table is
missing or contains no rows.

## Output schema

```sql
CREATE TABLE default_positions (
    camp        TEXT NOT NULL,   -- '红' | '蓝'
    robot_id    INTEGER NOT NULL,
    robot_class TEXT NOT NULL,   -- hero / engineer / infantry3 / infantry4 / aerial / sentry
    t           INTEGER NOT NULL, -- seconds since match start, 0-based
    x_med       REAL NOT NULL,
    y_med       REAL NOT NULL,
    x_p25       REAL, x_p75 REAL, y_p25 REAL, y_p75 REAL,
    n           INTEGER NOT NULL,
    PRIMARY KEY (camp, robot_id, t)
);
```

## Tests

Synthetic SQLite fixture (2 games × 3 robots × 3 seconds plus a base-building
row that must be excluded):

```bash
.venv/bin/python3 -m pytest tools/default_positions/test_build.py -v
```

Requires `pandas` and `pytest` in the repo venv (`.venv/`).
