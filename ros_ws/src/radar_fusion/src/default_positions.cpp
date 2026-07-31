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
