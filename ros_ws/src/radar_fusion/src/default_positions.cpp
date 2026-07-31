#include "radar_fusion/default_positions.hpp"

#include <sqlite3.h>

#include <mutex>

namespace radar_fusion::default_positions {

namespace {
std::unordered_map<std::string, DefaultPosition> g_rows;
std::unordered_map<std::string, int> g_max_t;
std::mutex g_mutex;

auto key(int camp, const std::string& robot_class, int t) -> std::string {
    return std::to_string(camp) + "|" + robot_class + "|" + std::to_string(t);
}

auto base_key(int camp, const std::string& robot_class) -> std::string {
    return std::to_string(camp) + "|" + robot_class;
}
} // namespace

auto load(const std::string& db_path) -> bool {
    sqlite3* db = nullptr;
    if (sqlite3_open(db_path.c_str(), &db) != SQLITE_OK) return false;
    const char* sql = "SELECT camp, robot_class, t, x_mode, y_mode, n FROM default_positions";
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        sqlite3_close(db);
        return false;
    }
    std::unordered_map<std::string, DefaultPosition> rows;
    std::unordered_map<std::string, int> max_t;
    int rc = SQLITE_OK;
    while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
        const std::string camp   = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
        const std::string rclass = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        const int t              = sqlite3_column_int(stmt, 2);
        const int camp_id        = (camp == "红") ? 0 : 1;
        const std::string row_key = key(camp_id, rclass, t);
        rows[row_key] = DefaultPosition { sqlite3_column_double(stmt, 3),
            sqlite3_column_double(stmt, 4), sqlite3_column_int(stmt, 5) };
        const std::string bk = base_key(camp_id, rclass);
        if (t > max_t[bk]) max_t[bk] = t;
    }
    if (rc != SQLITE_DONE) {
        sqlite3_finalize(stmt);
        sqlite3_close(db);
        return false;
    }
    sqlite3_finalize(stmt);
    sqlite3_close(db);
    std::lock_guard<std::mutex> lock(g_mutex);
    g_rows  = std::move(rows);
    g_max_t = std::move(max_t);
    return true;
}

auto query(int camp, const std::string& robot_class, int t, DefaultPosition& out) -> bool {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_rows.find(key(camp, robot_class, t));
    if (it == g_rows.end()) return false;
    out = it->second;
    return true;
}

auto query_clamped(int camp, const std::string& robot_class, int t, DefaultPosition& out)
    -> bool {
    std::lock_guard<std::mutex> lock(g_mutex);
    const std::string bk = base_key(camp, robot_class);
    const auto mt        = g_max_t.find(bk);
    if (mt == g_max_t.end()) return false;
    if (t > mt->second) t = mt->second;
    auto it = g_rows.find(key(camp, robot_class, t));
    if (it == g_rows.end()) return false;
    out = it->second;
    return true;
}

} // namespace radar_fusion::default_positions
