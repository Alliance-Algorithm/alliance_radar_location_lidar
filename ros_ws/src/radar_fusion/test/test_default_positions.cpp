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
        "x_p25 REAL, x_p75 REAL, y_p25 REAL, y_p75 REAL, n INTEGER NOT NULL,"
        "PRIMARY KEY (camp, robot_id, t));";
    ASSERT_EQ(sqlite3_exec(db, schema, nullptr, nullptr, nullptr), SQLITE_OK);
    ASSERT_EQ(sqlite3_exec(db, "INSERT INTO default_positions VALUES "
        "('红',1,'hero',1,5.5,2.25,5.0,6.0,2.0,2.5,20),"
        "('蓝',101,'hero',1,22.5,12.75,22.0,23.0,12.5,13.0,18);",
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
    EXPECT_DOUBLE_EQ(p.y_med, 2.25);
    EXPECT_EQ(p.n, 20);

    ASSERT_TRUE(radar_fusion::default_positions::query(1, "hero", 1, p));
    EXPECT_DOUBLE_EQ(p.x_med, 22.5);

    EXPECT_FALSE(radar_fusion::default_positions::query(0, "sentry", 1, p));
    EXPECT_FALSE(radar_fusion::default_positions::query(0, "hero", 999, p));
}

TEST(DefaultPositions, MissingFileReturnsFalse) {
    EXPECT_FALSE(radar_fusion::default_positions::load("/nonexistent/default_positions.sqlite"));
}

} // namespace
