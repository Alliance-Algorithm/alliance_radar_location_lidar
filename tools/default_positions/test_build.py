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
    # 2 games x 3 robots x 3 seconds = 18 rows; aggregation across games
    # yields 3 robots x 3 seconds = 9 output rows (n=2 each).
    rows = []
    for g in (1, 2):
        for t in (1, 2, 3):
            rows.append((g, t, 1, "英雄", "红", "A", 300, 450, 5.0 + t, 2.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0))
            rows.append((g, t, 2, "工程", "红", "A", 200, 250, 8.0, 3.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0))
            rows.append((g, t, 101, "英雄", "蓝", "B", 300, 450, 23.0 - t, 13.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0))
    # base building must never appear in the output
    rows.append((1, 1, 10, "基地", "红", "A", 500, 500, 14.0, 7.5, 0.0, 0.0, 0.0,
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
    rows = con.execute("SELECT camp, robot_id, robot_class, t, x_med, y_med, n "
                       "FROM default_positions ORDER BY camp, robot_id, t").fetchall()
    cols = [r[1] for r in con.execute("PRAGMA table_info(default_positions)").fetchall()]
    con.close()

    assert cols == ["camp", "robot_id", "robot_class", "t", "x_med", "y_med",
                    "cell_gx", "cell_gy", "x_mode", "y_mode",
                    "x_p25", "x_p75", "y_p25", "y_p75", "n"]

    # 2 games x 3 robots x 3 seconds of fixture rows are aggregated across
    # games (game_id is not part of the grouping), so the output holds
    # 3 robots (1, 2 red; 101 blue) x 3 seconds = 9 rows, each with n = 2.
    assert len(rows) == 9
    assert all(r[6] == 2 for r in rows)  # both games aggregated per group

    by_key = {(r[0], r[1], r[3]): r for r in rows}
    # red hero t=1: x = 5.0 + t -> 6.0 from both games, median of [6.0, 6.0]
    assert by_key[("红", 1, 1)][2] == "hero"
    assert by_key[("红", 1, 1)][4] == 6.0
    assert by_key[("红", 1, 1)][5] == 2.0
    # blue hero t=1: x = 23.0 - t -> 22.0 from both games
    assert by_key[("蓝", 101, 1)][2] == "hero"
    assert by_key[("蓝", 101, 1)][4] == 22.0
    assert by_key[("蓝", 101, 1)][5] == 13.0
    # red engineer robot 2: constant position
    assert by_key[("红", 2, 3)][2] == "engineer"
    assert by_key[("红", 2, 3)][4] == 8.0
    assert by_key[("红", 2, 3)][5] == 3.0

    # buildings (robot_id 10, 基地) never appear
    assert 10 not in {r[1] for r in rows}
    # every robot id from the fixture appears at each of the 3 seconds
    assert {r[1] for r in rows} == {1, 2, 101}
    assert {r[0] for r in rows} == {"红", "蓝"}


def test_mode_cells(tmp_path, fixture_db):
    out = tmp_path / "out.sqlite"
    run_build(fixture_db, out)
    con = sqlite3.connect(out)
    mode_rows = con.execute(
        "SELECT camp, robot_id, robot_class, t, cell_gx, cell_gy, x_mode, y_mode "
        "FROM default_positions ORDER BY camp, robot_id, t").fetchall()
    con.close()
    by_key = {(r[0], r[1], r[3]): r for r in mode_rows}
    # red hero t=1: both games at x=6.0, y=2.0 -> cell (6, 2), center (6.5, 2.5)
    r = by_key[("红", 1, 1)]
    assert (r[4], r[5]) == (6, 2)
    assert r[6] == 6.5 and r[7] == 2.5
    # blue hero t=1: x=22.0, y=13.0 -> cell (22, 13), center (22.5, 13.5)
    r = by_key[("蓝", 101, 1)]
    assert (r[4], r[5]) == (22, 13)
    assert r[6] == 22.5 and r[7] == 13.5


def run_build_expect_failure(db, out):
    script = Path(__file__).parent / "build_default_positions.py"
    proc = subprocess.run([sys.executable, str(script), "--db", str(db), "--out", str(out)],
                          capture_output=True, text=True)
    assert proc.returncode != 0, f"expected non-zero exit, got {proc.returncode}"
    assert "Traceback" not in proc.stderr, f"unexpected traceback:\n{proc.stderr}"
    assert proc.stderr.strip(), "expected a clear error message on stderr"
    return proc


def test_empty_db_fails_gracefully(tmp_path):
    db = tmp_path / "empty.sqlite"
    sqlite3.connect(db).close()  # no tables at all
    run_build_expect_failure(db, tmp_path / "out.sqlite")


def test_empty_timeseries_table_fails_gracefully(tmp_path):
    db = tmp_path / "empty_table.sqlite"
    con = sqlite3.connect(db)
    con.executescript(FIXTURE_SQL)
    con.close()  # timeseries exists but has no rows
    proc = run_build_expect_failure(db, tmp_path / "out.sqlite")
    assert "no timeseries rows found" in proc.stderr
