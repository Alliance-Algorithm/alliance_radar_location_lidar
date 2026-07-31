#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

namespace radar_fusion::default_positions {

struct DefaultPosition {
    double x_med = 0.0;  // mode-cell center (referee frame metres)
    double y_med = 0.0;  // mode-cell center (referee frame metres)
    int n        = 0;
};

auto load(const std::string& db_path) -> bool;
auto query(int camp, const std::string& robot_class, int t, DefaultPosition& out) -> bool;
// Like query, but when t exceeds the largest second available for
// (camp, robot_class), clamps t to that last row instead of failing.
auto query_clamped(int camp, const std::string& robot_class, int t, DefaultPosition& out)
    -> bool;

} // namespace radar_fusion::default_positions
