# Default Positions From Official RMUC Data — Design

## Goal

Train (statistically derive) a default-position time series per robot class from
the official RMUC 2026 regional dataset. When the radar has no detection for a
target slot, `radar_fusion` fills that slot with the default coordinate for the
current match time (measured from match start), so the referee client always
receives a plausible position.

## Data Source

Official RMUC 2026 regional dataset (613 matches, ~4.0M per-second rows) in a
SQLite file (`matches` / `timeseries` / `events` tables), downloaded from the
official forum post. Field layout: 28 m × 15 m. `robot_id`: red 1 hero / 2
engineer / 3 infantry3 / 4 infantry4 / 6 aerial / 7 sentry / 10 base / 11
outpost; blue = +100. Sample rate 1 Hz; a match lasts ~420 s.

Reference implementation studied: `RMUC-OfflineRL` (build_dataset.py / schema.py)
for schema handling and time-grid alignment conventions.

## Design

### 1. Offline build script (Python)

`tools/default_positions/build_default_positions.py`

Input: official dataset SQLite. Output: `default_positions.sqlite`.

Pipeline:

1. Read `timeseries` rows for mobile units only (英雄, 工程, 步兵3, 步兵4, 空中,
   哨兵) — exclude buildings (基地, 前哨站).
2. Drop lost-tracking rows (`x == 0 and y == 0`).
3. Clip coordinates into the padded field box (same convention as
   RMUC-OfflineRL: x in [-2, 30], y in [-2, 17]).
4. Keep the ORIGINAL field coordinates for both camps (no blue mirroring).
5. Group by `(camp, robot_id, robot_class, t)` and aggregate:
   - `x_med`, `y_med`: median position
   - `x_p25`, `x_p75`, `y_p25`, `y_p75`: interquartile band
   - `n`: sample count
6. Write `default_positions.sqlite` with a single table:

```sql
CREATE TABLE default_positions (
    camp        TEXT NOT NULL,   -- '红' | '蓝'
    robot_id    INTEGER NOT NULL,
    robot_class TEXT NOT NULL,   -- hero / engineer / infantry3 / infantry4 / aerial / sentry
    t           INTEGER NOT NULL, -- seconds since match start, 0-based
    x_med       REAL NOT NULL,
    y_med       REAL NOT NULL,
    x_p25       REAL, y_p75 REAL, y_p25 REAL, y_p75 REAL,
    n           INTEGER NOT NULL,
    PRIMARY KEY (camp, robot_id, t)
);
```

### 2. Runtime consumer (C++, radar_fusion)

- Load the whole `default_positions` table into memory at startup (12 robot
  classes × ~420 s ≈ 5k rows; trivial).
- Track match start: subscribe to `/bridge/game_state`; when `game_progress == 4`
  (比赛中) record the match start time (ROS time at the moment the transition is
  observed). Until that happens, defaults are disabled.
- Per `/lidar/cluster` frame (or camera detection), compute `t = now - match_start`
  in seconds. For each opponent/ally slot in `LidarLocation`:
  - if the slot has a confirmed track, publish the measured position (existing
    behavior);
  - otherwise fill with `(x_med, y_med)` for that `(camp, robot_class, t)`.
- Unit conversion: official data is already in the referee frame and in meters;
  `LidarLocation` is in millimeters. Default values convert with a plain
  `(x_med * 1000)`, `(y_med * 1000)` — WITHOUT `map_to_rm_offset`, which only
  applies to the localization-map track path. Negative medians (the build clip
  allows x, y down to -2 m) clamp to 0 so they cannot wrap in `uint16_t`. The
  official referee frame (0,0) is the red corner and differs from the
  localization-map origin by (14.0, 7.5); defaults are never translated.
- If `t` exceeds the last available second, clamp to the last row. If a row for
  `(camp, robot_class, t)` has no data (e.g. `n == 0`), fall back to the last
  non-empty row.
- A `default_positions_path` parameter points at the SQLite file; empty path
  disables the feature (no defaults, existing behavior preserved).

### 3. Mapping between LidarLocation slots and default rows

The `LidarLocation` msg has fixed slots: opponent_hero/engineer/infantry_3/
infantry_4/aerial/sentry and ally_* equivalents. Default rows are keyed by
`(camp, robot_class)`. The consumer maps each slot to its class:
hero, engineer, infantry3, infantry4, aerial, sentry. `camp` for opponent slots
is the enemy color (from `enemy_color` param), `camp` for ally slots is the own
color.

## Files

- `tools/default_positions/build_default_positions.py` — offline aggregation
- `tools/default_positions/README.md` — usage (download link, run command)
- `ros_ws/src/radar_fusion/` — runtime changes:
  - new source `src/default_positions.cpp` (+ header) for loading/querying
  - `radar_fusion_node` subscribes `/bridge/game_state`, fills missing slots
- Config: `default_positions_path` parameter (radar_fusion yaml)

## Open Questions

- Where exactly to place the generated SQLite at runtime (container path); the
  parameter makes this deployment-time configurable.
- Whether ally slots also need defaults or only opponent slots. Default: both
  (data exists for both camps).

## Verification

- Build script run on the official dataset produces 12 classes × ~420 s rows;
  spot-check a few (robot, t) medians against raw SQL.
- Unit test: query function returns expected values for known rows, clamps t
  past the end, falls back on empty rows.
- Runtime smoke: with `default_positions_path` set and no tracks, `/lidar/location`
  slots are non-zero and match the table; with tracks, measured positions win.
