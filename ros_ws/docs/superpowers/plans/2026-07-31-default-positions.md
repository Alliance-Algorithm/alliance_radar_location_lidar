# Default Positions From Official RMUC Data — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Derive per-robot-class default position time series from the official RMUC 2026 dataset and have `radar_fusion` fill unobserved target slots with them, keyed by match time.

**Architecture:** An offline Python script aggregates the official `timeseries` table into a compact SQLite file (`default_positions.sqlite`) with median + IQR positions per `(camp, robot_id, t)`. At runtime `radar_fusion` loads that table into memory at startup, subscribes to `/bridge/game_state`, records match start at `game_progress == 4`, and fills `LidarLocation` slots that lack a confirmed track with the table value at `t = now - match_start`.

**Tech Stack:** Python 3 + sqlite3 + pandas (offline build); C++23 + rclcpp + sqlite3 (runtime load — header-only memory table, no SQL at runtime).

## Global Constraints

- Default rows use ORIGINAL field coordinates (no blue mirroring); field is 28 m × 15 m.
- `robot_id`: red 1 hero / 2 engineer / 3 infantry3 / 4 infantry4 / 6 aerial / 7 sentry; blue = +100. Buildings (10, 11, 110, 111) excluded.
- Drop lost-tracking rows (`x == 0 AND y == 0`).
- Clip coordinates into padded field box: x ∈ [-2, 30], y ∈ [-2, 17].
- Runtime converts meters → millimeters: `max(0, med) * 1000` — defaults are
  already in the official referee frame, so `map_to_rm_offset` is NOT applied
  (track path in `radar_fusion_node.cpp:383` is a different frame: localization
  map center-origin). Negative medians clamp to 0 to avoid `uint16_t` wrap.
- Match time `t0` = ROS time at first observation of `game_progress == 4`; defaults disabled before that.
- Empty `default_positions_path` param disables the feature entirely (existing behavior preserved).

---

### Task 1: Offline build script

**Files:**
- Create: `tools/default_positions/build_default_positions.py`
- Create: `tools/default_positions/README.md`
- Test: `tools/default_positions/test_build.py` (synthetic SQLite fixture)

**Interfaces:**
- Consumes: official dataset SQLite path (CLI arg `--db`), output path (`--out`)
- Produces: `default_positions.sqlite` with schema:

```sql
CREATE TABLE default_positions (
    camp        TEXT NOT NULL,   -- '红' | '蓝'
    robot_id    INTEGER NOT NULL,
    robot_class TEXT NOT NULL,   -- hero/engineer/infantry3/infantry4/aerial/sentry
    t           INTEGER NOT NULL,
    x_med       REAL NOT NULL,
    y_med       REAL NOT NULL,
    x_p25       REAL, y_p75 REAL, y_p25 REAL, y_p75 REAL,
    n           INTEGER NOT NULL,
    PRIMARY KEY (camp, robot_id, t)
);
```

- [ ] **Step 1: Write the failing test**

`tools/default_positions/test_build.py`:

```python
import sqlite3
import subprocess
import sys
from pathlib import Path

import pytest

FIXTURE_SQL = """
CREATE TABLE matches (game_id INTEGER, 赛区 TEXT, 胜方 TEXT, 红方学校 TEXT, 蓝方学校 TEXT, 时长秒 INTEGER);
CREATE TABLE timeseries (
    game_id INTEGER, 时刻秒 INTEGER, robot_id INTEGER, 机器人类型 TEXT, 阵营 TEXT, 学校名 TEXT,
    当前血量 INTEGER, 最大血量 INTEGER, x REAL, y REAL, z REAL, 枪口朝向 REAL, 底盘功率 REAL,
    小热量 REAL, 小热量上限 REAL, 大热量 REAL, 大热量上限 REAL, 累计17mm发弹 REAL, 累计42mm发弹 REAL,
    队伍总金币 REAL, 队伍剩余金币 REAL, 是否易伤 INTEGER);
CREATE TABLE events (game_id INTEGER, 时刻秒 INTEGER, 事件类型 TEXT);
"""

@pytest.fixture()
def fixture_db(tmp_path):
    db = tmp_path / "fixture.sqlite"
    con = sqlite3.connect(db)
    con.executescript(FIXTURE_SQL)
    # 2 games x 3 robots x 3 seconds; blue hero mirrored-position values
    rows = []
    for g in (1, 2):
        for t in (1, 2, 3):
            rows.append((g, t, 1, "英雄", "红", "A", 300, 450, 5.0 + t, 2.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0))
            rows.append((g, t, 2, "工程", "红", "A", 200, 250, 8.0, 3.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0))
            rows.append((g, t, 101, "英雄", "蓝", "B", 300, 450, 23.0 - t, 13.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0))
    con.executemany("INSERT INTO timeseries VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)", rows)
    con.commit()
    con.close()
    return db

def run_build(db, out):
    script = Path(__file__).parent / "build_default_positions.py"
    subprocess.run([sys.executable, str(script), "--db", str(db), "--out", str(out)],
                   check=True, capture_output=True)

def test_median_and_classes(tmp_path, fixture_db):
    out = tmp_path / "out.sqlite"
    run_build(fixture_db, out)
    con = sqlite3.connect(out)
    rows = con.execute("SELECT camp, robot_id, robot_class, t, x_med, y_med, n FROM default_positions ORDER BY camp, robot_id, t").fetchall()
    con.close()
    assert len(rows) == 12  # 2 camps x 3 robots x ... wait: 2 camps*3 robots*3 t? see below
```

**Note:** the fixture above yields 2 camps × 3 robot_ids (1, 2, 101) × 3 seconds = 18 rows. Fix the assertion to 18 and assert:
- red hero t=1 → x_med = 6.0 (mean of 5+1, 5+1 from both games), y_med = 2.0
- blue hero t=1 → x_med = 22.0, y_med = 13.0
- buildings never appear (insert a base row with robot_id 10 in the fixture and assert it is absent)

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tools/default_positions/test_build.py -v`
Expected: FAIL — `ModuleNotFoundError: build_default_positions.py` / file missing.

- [ ] **Step 3: Write the build script**

```python
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


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    con = sqlite3.connect(args.db)
    df = pd.read_sql(TS_SELECT, con)
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
    ocon.execute("""CREATE TABLE default_positions (
        camp TEXT NOT NULL, robot_id INTEGER NOT NULL, robot_class TEXT NOT NULL,
        t INTEGER NOT NULL, x_med REAL NOT NULL, y_med REAL NOT NULL,
        x_p25 REAL, x_p75 REAL, y_p25 REAL, y_p75 REAL, n INTEGER NOT NULL,
        PRIMARY KEY (camp, robot_id, t))""")
    out.to_sql("default_positions", ocon, if_exists="append", index=False)
    ocon.commit()
    ocon.close()
    print(f"wrote {len(out)} rows to {args.out}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tools/default_positions/test_build.py -v`
Expected: PASS (18 rows, correct medians, buildings excluded).

- [ ] **Step 5: Write README**

`tools/default_positions/README.md`: purpose, download link to official dataset, run command, output schema.

- [ ] **Step 6: Commit**

```bash
git add tools/default_positions/
git commit -m "feat(default_positions): offline aggregation script from official RMUC data"
```

---

### Task 2: C++ runtime loader (ROS-free)

**Files:**
- Create: `ros_ws/src/radar_fusion/include/radar_fusion/default_positions.hpp`
- Create: `ros_ws/src/radar_fusion/src/default_positions.cpp`
- Test: `ros_ws/src/radar_fusion/test/test_default_positions.cpp`

**Interfaces:**
- Consumes: SQLite file path (schema from Task 1)
- Produces:

```cpp
namespace radar_fusion::default_positions {

struct DefaultPosition {
    double x_med = 0.0;
    double y_med = 0.0;
    int n        = 0;
};

// Loads the whole table into memory. Returns false if file missing/unreadable.
// Empty/invalid rows are skipped; the caller decides fallback policy.
auto load(const std::string& db_path) -> bool;

// Query by (camp, robot_class, t). camp: 0=red, 1=blue.
// Returns false if no row exists for the key.
auto query(int camp, const std::string& robot_class, int t, DefaultPosition& out) -> bool;

} // namespace radar_fusion::default_positions
```

- [ ] **Step 1: Write the failing test**

`ros_ws/src/radar_fusion/test/test_default_positions.cpp`:

```cpp
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <sqlite3.h>

#include "radar_fusion/default_positions.hpp"

namespace {

std::string make_db() {
    const std::string path = "/tmp/default_positions_test.sqlite";
    std::remove(path.c_str());
    sqlite3* db = nullptr;
    ASSERT_EQ(sqlite3_open(path.c_str(), &db), SQLITE_OK);
    const char* schema = "CREATE TABLE default_positions ("
        "camp TEXT NOT NULL, robot_id INTEGER NOT NULL, robot_class TEXT NOT NULL,"
        "t INTEGER NOT NULL, x_med REAL NOT NULL, y_med REAL NOT NULL,"
        "x_p25 REAL, x_p75 REAL, y_p25 REAL, y_p75 REAL, n INTEGER NOT NULL,"
        "PRIMARY KEY (camp, robot_id, t));";
    ASSERT_EQ(sqlite3_exec(db, schema, nullptr, nullptr, nullptr), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(db, "INSERT INTO default_positions VALUES "
        "('红',1,'hero',1,5.5,2.25,5.0,6.0,2.0,2.5,20),"
        "('蓝',101,'hero',1,22.5,12.75,22.0,23.0,12.5,13.0,18);",
        nullptr, nullptr, nullptr), SQLITE_OK);
    sqlite3_close(db);
    return path;
}

TEST(DefaultPositions, LoadAndQuery) {
    const std::string db = make_db();
    using radar_fusion::default_positions::DefaultPosition;
    ASSERT_TRUE(radar_fusion::default_positions::load(db));

    DefaultPosition p;
    ASSERT_TRUE(radar_fusion::default_positions::query(0, "hero", 1, p));
    EXPECT_DOUBLE_EQ(p.x_med, 5.5);
    EXPECT_DOUBLE_EQ(p.y_med, 2.25);
    EXPECT_EQ(p.n, 20);

    ASSERT_TRUE(radar_fusion::default_positions::query(1, "hero", 1, p));
    EXPECT_DOUBLE_EQ(p.x_med, 22.5);

    EXPECT_FALSE(radar_fusion::default_positions::query(0, "sentry", 1, p));
    EXPECT_FALSE(radar_fusion::default_positions::query(0, "hero", 999, p));
}

TEST(DefaultPositions, MissingFileReturnsFalse) {
    using radar_fusion::default_positions;
    EXPECT_FALSE(default_positions::load("/nonexistent/default_positions.sqlite"));
}

} // namespace
```

- [ ] **Step 2: Run test to verify it fails**

Run: `colcon build --packages-select radar_fusion --cmake-args -DBUILD_TESTING=ON` then `colcon test --packages-select radar_fusion --ctest-args -R default_positions`
Expected: FAIL — `default_positions.hpp` missing.

- [ ] **Step 3: Write the loader**

Header `default_positions.hpp`:

```cpp
#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace radar_fusion::default_positions {

struct DefaultPosition {
    double x_med = 0.0;
    double y_med = 0.0;
    int n        = 0;
};

auto load(const std::string& db_path) -> bool;
auto query(int camp, const std::string& robot_class, int t, DefaultPosition& out) -> bool;

} // namespace radar_fusion::default_positions
```

Source `default_positions.cpp`:

```cpp
#include "radar_fusion/default_positions.hpp"

#include <sqlite3.h>

#include <mutex>

namespace radar_fusion::default_positions {

namespace {
std::unordered_map<std::string, DefaultPosition> g_rows;
std::mutex g_mutex;
} // namespace

auto load(const std::string& db_path) -> bool {
    sqlite3* db = nullptr;
    if (sqlite3_open(db_path.c_str(), &db) != SQLITE_OK) return false;
    const char* sql = "SELECT camp, robot_class, t, x_med, y_med, n FROM default_positions";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        sqlite3_close(db);
        return false;
    }
    std::unordered_map<std::string, DefaultPosition> rows;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        const std::string camp   = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
        const std::string rclass = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        const int t              = sqlite3_column_int(stmt, 2);
        const int camp_id        = (camp == "红") ? 0 : 1;
        const std::string key    = std::to_string(camp_id) + "|" + rclass + "|" + std::to_string(t);
        rows[key] = DefaultPosition { sqlite3_column_double(stmt, 3),
            sqlite3_column_double(stmt, 4), sqlite3_column_int(stmt, 5) };
    }
    sqlite3_finalize(stmt);
    sqlite3_close(db);
    std::lock_guard<std::mutex> lock(g_mutex);
    g_rows = std::move(rows);
    return true;
}

auto query(int camp, const std::string& robot_class, int t, DefaultPosition& out) -> bool {
    const std::string key = std::to_string(camp) + "|" + robot_class + "|" + std::to_string(t);
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_rows.find(key);
    if (it == g_rows.end()) return false;
    out = it->second;
    return true;
}

} // namespace radar_fusion::default_positions
```

- [ ] **Step 4: Add sqlite3 to CMake and register test**

Modify `ros_ws/src/radar_fusion/CMakeLists.txt`:
- Link `sqlite3` to `${PROJECT_NAME}_core` (or a new `default_positions` static lib) and to the test target.
- Add `test/test_default_positions.cpp` to `ament_add_gtest`.

- [ ] **Step 5: Run tests to verify they pass**

Run: `colcon build --packages-select radar_fusion --cmake-args -DBUILD_TESTING=ON` then `colcon test --packages-select radar_fusion`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add ros_ws/src/radar_fusion/include/radar_fusion/default_positions.hpp
git add ros_ws/src/radar_fusion/src/default_positions.cpp
git add ros_ws/src/radar_fusion/test/test_default_positions.cpp
git add ros_ws/src/radar_fusion/CMakeLists.txt
git commit -m "feat(fusion): load and query default positions from sqlite"
```

---

### Task 3: Node integration — match timer + slot filling

**Files:**
- Modify: `ros_ws/src/radar_fusion/include/radar_fusion/radar_fusion_node.hpp`
- Modify: `ros_ws/src/radar_fusion/src/radar_fusion_node.cpp`
- Modify: `ros_ws/src/radar_fusion/config/runtime.yaml`
- Test: `ros_ws/src/radar_fusion/test/test_fusion_node.cpp` (extend)

**Interfaces:**
- Consumes: `default_positions::load/query` (Task 2); `/bridge/game_state` msg (`cmd_id, game_type, game_progress, stage_remain_time, sync_timestamp`)
- Produces: `LidarLocation` slots filled with defaults when no confirmed track occupies them

Match timer design (user-confirmed, supersedes the earlier ROS-time-only design):
- Match start is detected by an OR of two conditions from `/bridge/game_state`:
  - `game_progress == 4` (比赛中), or
  - `stage_remain_time > 400` (entering the 420 s battle stage)
- On start, record the LOCAL steady-clock time as `match_start_ns_`. Do NOT use
  `sync_timestamp` for elapsed-time math (referee timestamps are considered
  inaccurate).
- Elapsed time: `t = (now - match_start_ns_) / 1e9` using the local clock,
  clamped ≥ 0.
- Multi-round reuse: the timer state machine RESETS when a round ends
  (`game_progress == 5` 结算 or `game_progress` leaves the battle phase /
  `stage_remain_time` drops below a small threshold such as 10), so the next
  round re-enters the time series from t=0 again. Reset must not trigger a
  start on the same message that ends a round.

- [ ] **Step 1: Write failing tests**

Extend `test_fusion_node.cpp` — a pure unit test of the slot-filling helper and the match-timer state machine. Extract a small testable state machine (e.g. a `MatchTimer` class or free functions in the node's namespace, header-only or in a .cpp compiled into the test):

```cpp
TEST(FusionNode, MatchTimerStartsOnProgress4) {
    // game_progress 0 -> no start; 4 -> start; elapsed grows with injected now
}

TEST(FusionNode, MatchTimerStartsOnRemain400) {
    // progress stays 0 but stage_remain_time jumps to 401 -> start
}

TEST(FusionNode, MatchTimerResetsForNextRound) {
    // start -> elapsed t -> round end (progress 5 / remain < 10) -> reset;
    // a later start condition re-enters from t=0
}

TEST(FusionNode, SlotFilledWhenNoTrack) {
    // tracks empty -> every opponent/ally slot takes the default for its class
    // at t = 0; verify one slot value via the same conversion used in publish.
}
```

The MatchTimer should be deterministic and testable without ROS spinning: constructor takes an injectable clock (or `now_ns` passed per-call as an `int64_t` argument), so tests control time explicitly.

- [ ] **Step 2: Run tests to verify they fail**

Run: `colcon test --packages-select radar_fusion --ctest-args -R fusion_node`
Expected: FAIL — helpers not defined.

- [ ] **Step 3: Implement node changes**

Add a match-timer unit (small, ROS-free, in `radar_fusion` include/src or inside the node files as long as it is testable):

```cpp
class MatchTimer {
public:
    // feed a game_state observation with explicit now_ns (testable).
    void on_game_state(uint8_t game_progress, uint16_t stage_remain_time, int64_t now_ns);
    // seconds since match start; -1 if never started this round.
    auto elapsed_sec(int64_t now_ns) const -> int64_t;
    auto started() const -> bool;
private:
    int64_t start_ns_ = 0;
    bool running_ = false;
};
```

Semantics:
- `on_game_state`: if `!running_` and (`game_progress == 4` or `stage_remain_time > 400`), set `running_ = true; start_ns_ = now_ns`. If `running_` and (`game_progress == 5` or `stage_remain_time < 10`), set `running_ = false`. Otherwise no change.
- `elapsed_sec`: `running_ ? max(0, (now_ns - start_ns_) / 1e9) : -1`.

In `radar_fusion_node.hpp` add:
- `radar_fusion::match_timer::MatchTimer match_timer_;`
- `rclcpp::Subscription<radar_interfaces::msg::GameState>::SharedPtr sub_game_state_;`
- `void on_game_state(radar_interfaces::msg::GameState::SharedPtr msg);`
- `void fill_default_positions(radar_interfaces::msg::LidarLocation& msg, int64_t now_ns);`

In `radar_fusion_node.cpp`:
- declare params: `default_positions_path` (std::string, default `""`), `enemy_color` (std::string, default `"blue"`)
- if path non-empty: `default_positions::load(path)`; subscribe `/bridge/game_state`
- `on_game_state`: `match_timer_.on_game_state(msg->game_progress, msg->stage_remain_time, now_ns())`
- `fill_default_positions`: slot → class map (opponent_hero→hero, opponent_engineer→engineer, opponent_infantry_3→infantry3, opponent_infantry_4→infantry4, opponent_aerial→aerial, opponent_sentry→sentry; same for ally_*), enemy camp id = (enemy_color=="red")?0:1, ally camp = 1-enemy; when track missing and `match_timer_.started()` and `query` succeeds, write `max(0, x_med) * 1000` into slot (defaults are already in the referee frame — no `map_to_rm_offset`)
- call `fill_default_positions(msg, now_ns())` in `publish_lidar_location` before publishing

- [ ] **Step 4: Update yaml**

`config/runtime.yaml`: add `default_positions_path: ""` and `enemy_color: "blue"`.

- [ ] **Step 5: Run tests**

Run: `colcon build --packages-select radar_fusion --cmake-args -DBUILD_TESTING=ON && colcon test --packages-select radar_fusion`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add ros_ws/src/radar_fusion/
git commit -m "feat(fusion): fill unobserved slots with time-based default positions"
```

---

### Task 4: End-to-end verification with real data

**Files:**
- Run: `tools/default_positions/build_default_positions.py` on official dataset
- Verify: SQL spot-checks + runtime smoke

- [ ] **Step 1: Download official dataset**

From the forum post (link in `tools/default_positions/README.md`), place `rmuc_2026_region_dataset.7z` in `RMUC-OfflineRL/dataset/` and extract.

- [ ] **Step 2: Run the build script**

Run: `python3 tools/default_positions/build_default_positions.py --db /path/rmuc_2026_region_dataset.sqlite --out /path/default_positions.sqlite`
Expected: prints row count (12 classes × ~420 s ≈ 5000 rows).

- [ ] **Step 3: Spot-check medians**

Run SQL on the output: e.g. red hero t=1 should sit near the red spawn (x ≈ 5-8 m), blue hero t=1 near (x ≈ 20-23 m). Compare against a raw `SELECT AVG(x) ... GROUP BY t` for sanity.

- [ ] **Step 4: Runtime smoke test**

Launch with `default_positions_path` set and no tracks; verify `/lidar/location` opponent slots are non-zero and match the table. Then with tracks; verify measured positions win.

- [ ] **Step 5: Commit any generated-file docs**

```bash
git add tools/default_positions/README.md
git commit -m "docs(default_positions): record real-data verification"
```

---

## Self-Review Notes

- Spec coverage: offline build (Task 1), runtime load/query (Task 2), match timer + slot fill (Task 3), real-data verification (Task 4). All spec sections mapped.
- Median + IQR aggregation per spec; original coordinates; buildings excluded; (0,0) dropped; clipped to padded field.
- Unit conversion (m→mm ×1000 with negative clamp) applied in Task 3 step 3; defaults are already in the official referee frame, so `map_to_rm_offset` is intentionally NOT applied (applying it would shift defaults by half a field).
- Fallback policy: if a (camp, class, t) row is missing, slot stays 0 (no default) — acceptable; documented.
- Open question resolved: both opponent and ally slots get defaults (data exists for both camps).
