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
5. Group by `(camp, robot_id, robot_class, t)` across all games and aggregate:
   - the **1 m grid mode cell** (`cell_gx/cell_gy`, center `x_mode/y_mode`) —
     the most-likely cell, matching RMUC-OfflineRL's 420-cell semantics
     (cross-team medians land where no team actually goes);
   - median (x/y) and IQR band (p25/p75) as reference columns;
   - sample count `n`.

Exits non-zero with a message (no traceback) if the `timeseries` table is
missing or contains no rows.

## Output schema

```sql
CREATE TABLE default_positions (
    camp        TEXT NOT NULL,   -- '红' | '蓝'
    robot_id    INTEGER NOT NULL,
    robot_class TEXT NOT NULL,   -- hero / engineer / infantry3 / infantry4 / aerial / sentry
    t           INTEGER NOT NULL, -- seconds since match start, 0-based
    x_med       REAL NOT NULL,   -- median (reference)
    y_med       REAL NOT NULL,
    cell_gx     INTEGER NOT NULL, -- mode cell (1 m grid)
    cell_gy     INTEGER NOT NULL,
    x_mode      REAL NOT NULL,   -- mode cell center, referee metres (runtime default)
    y_mode      REAL NOT NULL,
    x_p25       REAL, x_p75 REAL, y_p25 REAL, y_p75 REAL,
    n           INTEGER NOT NULL,
    PRIMARY KEY (camp, robot_id, t)
);
```

The `radar_fusion` runtime loader reads `x_mode`/`y_mode` (see
`ros_ws/src/radar_fusion/src/default_positions.cpp`).

## Tests

Synthetic SQLite fixture (2 games × 3 robots × 3 seconds plus a base-building
row that must be excluded):

```bash
.venv/bin/python3 -m pytest tools/default_positions/test_build.py -v
```

Requires `pandas` and `pytest` in the repo venv (`.venv/`).

## End-to-end check

`e2e_default_positions.py` verifies the live `radar_fusion` node fills
unobserved `LidarLocation` slots with the mode-cell defaults only after a
match has started (`game_progress == 4`):

```bash
ros2 run radar_fusion radar_fusion_node --ros-args \
    -p default_positions_path:=model/default_positions.sqlite \
    -p enemy_color:=red -p enable_camera_fusion:=false &
python3 tools/default_positions/e2e_default_positions.py
```

Expected output: `PRE` all-zero (defaults suppressed before match start),
`POST` non-zero (mode-cell defaults fill unobserved slots after start), and
`RESULT: PASS`.

## Animation

`animate_default_positions.py` renders the per-second mode positions of all
12 (camp × class) robots over the radar-egui minimap backdrop with a 10×10
grid (full-field mapping, referee frame):

```bash
.venv/bin/python3 tools/default_positions/animate_default_positions.py \
    --db model/default_positions.sqlite \
    --bg /path/to/radar-egui/assets/minimap_bg.png \
    --out /tmp/default_positions.mp4
```
