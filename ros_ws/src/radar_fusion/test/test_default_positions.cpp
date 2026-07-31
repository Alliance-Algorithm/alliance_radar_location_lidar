#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <sqlite3.h>

#include "radar_fusion/default_positions.hpp"

namespace {

void make_db(std::string& path) {
    path = "/tmp/default_positions_test.sqlite";
    std::remove(path.c_str());
    sqlite3* db = nullptr;
    ASSERT_EQ(sqlite3_open(path.c_str(), &db), SQLITE_OK);
    const char* schema = "CREATE TABLE default_positions ("
        "camp TEXT NOT NULL, robot_id INTEGER NOT NULL, robot_class TEXT NOT NULL,"
        "t INTEGER NOT NULL, x_med REAL NOT NULL, y_med REAL NOT NULL,"
        "cell_gx INTEGER NOT NULL, cell_gy INTEGER NOT NULL,"
        "x_mode REAL NOT NULL, y_mode REAL NOT NULL,"
        "x_p25 REAL, x_p75 REAL, y_p25 REAL, y_p75 REAL, n INTEGER NOT NULL,"
        "PRIMARY KEY (camp, robot_id, t));";
    ASSERT_EQ(sqlite3_exec(db, schema, nullptr, nullptr, nullptr), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(db, "INSERT INTO default_positions VALUES "
        "('红',1,'hero',1,5.5,2.25,5,2,5.5,2.5,5.0,6.0,2.0,2.5,20),"
        "('蓝',101,'hero',1,22.5,12.75,22,12,22.5,12.5,22.0,23.0,12.5,13.0,18),"
        "('红',1,'hero',5,6.0,2.5,6,2,6.5,2.5,5.5,6.5,2.25,2.75,25),"
        "('蓝',101,'hero',5,23.0,13.0,23,12,23.5,12.5,22.5,23.5,12.75,13.25,22);",
        nullptr, nullptr, nullptr), SQLITE_OK);
    sqlite3_close(db);
}

TEST(DefaultPositions, LoadAndQuery) {
    std::string db;
    make_db(db);
    using radar_fusion::default_positions::DefaultPosition;
    ASSERT_TRUE(radar_fusion::default_positions::load(db));

    DefaultPosition p;
    ASSERT_TRUE(radar_fusion::default_positions::query(0, "hero", 1, p));
    EXPECT_DOUBLE_EQ(p.x_med, 5.5);
    EXPECT_DOUBLE_EQ(p.y_med, 2.5);
    EXPECT_EQ(p.n, 20);

    ASSERT_TRUE(radar_fusion::default_positions::query(1, "hero", 1, p));
    EXPECT_DOUBLE_EQ(p.x_med, 22.5);

    EXPECT_FALSE(radar_fusion::default_positions::query(0, "sentry", 1, p));
    EXPECT_FALSE(radar_fusion::default_positions::query(0, "hero", 999, p));
}

TEST(DefaultPositions, MissingFileReturnsFalse) {
    EXPECT_FALSE(radar_fusion::default_positions::load("/nonexistent/default_positions.sqlite"));
}

TEST(DefaultPositions, QueryClampedClampsToLastAvailableSecond) {
    std::string db;
    make_db(db);
    ASSERT_TRUE(radar_fusion::default_positions::load(db));

    radar_fusion::default_positions::DefaultPosition p;

    // In-range exact hit.
    ASSERT_TRUE(radar_fusion::default_positions::query_clamped(0, "hero", 1, p));
    EXPECT_DOUBLE_EQ(p.x_med, 5.5);

    // t beyond the last row per (camp, class): clamp to the t=5 row.
    ASSERT_TRUE(radar_fusion::default_positions::query_clamped(0, "hero", 500, p));
    EXPECT_DOUBLE_EQ(p.x_med, 6.5);
    EXPECT_DOUBLE_EQ(p.y_med, 2.5);
    ASSERT_TRUE(radar_fusion::default_positions::query_clamped(1, "hero", 500, p));
    EXPECT_DOUBLE_EQ(p.x_med, 23.5);

    // Gaps inside the covered range are not filled: t=3 does not exist.
    EXPECT_FALSE(radar_fusion::default_positions::query_clamped(0, "hero", 3, p));

    // Missing (camp, class): no max-t row, so false.
    EXPECT_FALSE(radar_fusion::default_positions::query_clamped(0, "sentry", 500, p));
}

// Builds a multi-page DB (500 rows -> several leaf pages), then corrupts the
// header byte of the LAST page (invalid B-tree page type). A full table scan
// reads the earlier leaf pages successfully and then fails at sqlite3_step
// with SQLITE_CORRUPT, so load() must return false instead of committing a
// partially scanned table.
bool make_corrupt_db(std::string& path) {
    path = "/tmp/default_positions_corrupt.sqlite";
    std::remove(path.c_str());
    sqlite3* db = nullptr;
    if (sqlite3_open(path.c_str(), &db) != SQLITE_OK) return false;
    // WITHOUT ROWID: the PRIMARY KEY is the rowid, so no trailing autoindex
    // B-tree exists and the last file page is the table's last leaf page.
    const char* schema = "CREATE TABLE default_positions ("
        "camp TEXT NOT NULL, robot_id INTEGER NOT NULL, robot_class TEXT NOT NULL,"
        "t INTEGER NOT NULL, x_med REAL NOT NULL, y_med REAL NOT NULL,"
        "cell_gx INTEGER NOT NULL, cell_gy INTEGER NOT NULL,"
        "x_mode REAL NOT NULL, y_mode REAL NOT NULL,"
        "x_p25 REAL, x_p75 REAL, y_p25 REAL, y_p75 REAL, n INTEGER NOT NULL,"
        "PRIMARY KEY (camp, robot_id, t)) WITHOUT ROWID;";
    if (sqlite3_exec(db, schema, nullptr, nullptr, nullptr) != SQLITE_OK) { sqlite3_close(db); return false; }
    if (sqlite3_exec(db,
        "INSERT INTO default_positions "
        "WITH RECURSIVE cnt(x) AS (SELECT 1 UNION ALL SELECT x+1 FROM cnt WHERE x < 500) "
        "SELECT '红', x, 'hero', 1, 5.5, 2.25, 5, 2, 5.5, 2.5, NULL, NULL, NULL, NULL, 20 FROM cnt;",
        nullptr, nullptr, nullptr) != SQLITE_OK) { sqlite3_close(db); return false; }
    if (sqlite3_close(db) != SQLITE_OK) return false;

    std::ifstream in(path, std::ios::binary);
    if (!in.good()) return false;
    unsigned char header[100];
    in.read(reinterpret_cast<char*>(header), 100);
    if (!in.good()) return false;
    const int page_size = (header[16] << 8) | header[17];
    in.seekg(0, std::ios::end);
    const auto file_size = in.tellg();
    in.close();
    if (file_size < 2LL * page_size) return false;

    std::fstream out(path, std::ios::binary | std::ios::in | std::ios::out);
    if (!out.good()) return false;
    out.seekp(-static_cast<std::streamoff>(page_size), std::ios::end);
    const char invalid_page_type = 0x00;
    out.write(&invalid_page_type, 1);
    out.close();
    return true;
}

TEST(DefaultPositions, UnscannableTableReturnsFalse) {
    std::string db;
    ASSERT_TRUE(make_corrupt_db(db));

    // Sanity-check the fixture: a raw scan must see >= 1 row and then fail
    // with SQLITE_CORRUPT mid-scan (not fail at open/prepare).
    sqlite3* raw = nullptr;
    ASSERT_EQ(sqlite3_open(db.c_str(), &raw), SQLITE_OK);
    sqlite3_stmt* stmt = nullptr;
    ASSERT_EQ(sqlite3_prepare_v2(raw, "SELECT 1 FROM default_positions", -1, &stmt, nullptr), SQLITE_OK);
    int rc = SQLITE_OK;
    int rows = 0;
    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) ++rows;
    ASSERT_EQ(rc, SQLITE_CORRUPT);
    ASSERT_GT(rows, 0);
    sqlite3_finalize(stmt);
    sqlite3_close(raw);

    EXPECT_FALSE(radar_fusion::default_positions::load(db));
}

} // namespace
