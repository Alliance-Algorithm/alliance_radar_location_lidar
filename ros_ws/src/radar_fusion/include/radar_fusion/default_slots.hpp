#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <string>

#include "radar_fusion/default_positions.hpp"
#include "radar_interfaces/msg/lidar_location.hpp"

namespace radar_fusion::default_positions {

// One LidarLocation slot: the x/y fields it writes and the robot class it
// queries defaults for.
struct SlotSpec {
    std::uint16_t radar_interfaces::msg::LidarLocation::*x;
    std::uint16_t radar_interfaces::msg::LidarLocation::*y;
    const char* robot_class;
};

inline constexpr std::array<SlotSpec, 6> kOpponentSlots = {
    SlotSpec { &radar_interfaces::msg::LidarLocation::opponent_hero_x,
        &radar_interfaces::msg::LidarLocation::opponent_hero_y, "hero" },
    SlotSpec { &radar_interfaces::msg::LidarLocation::opponent_engineer_x,
        &radar_interfaces::msg::LidarLocation::opponent_engineer_y, "engineer" },
    SlotSpec { &radar_interfaces::msg::LidarLocation::opponent_infantry_3_x,
        &radar_interfaces::msg::LidarLocation::opponent_infantry_3_y, "infantry3" },
    SlotSpec { &radar_interfaces::msg::LidarLocation::opponent_infantry_4_x,
        &radar_interfaces::msg::LidarLocation::opponent_infantry_4_y, "infantry4" },
    SlotSpec { &radar_interfaces::msg::LidarLocation::opponent_aerial_x,
        &radar_interfaces::msg::LidarLocation::opponent_aerial_y, "aerial" },
    SlotSpec { &radar_interfaces::msg::LidarLocation::opponent_sentry_x,
        &radar_interfaces::msg::LidarLocation::opponent_sentry_y, "sentry" },
};

inline constexpr std::array<SlotSpec, 6> kAllySlots = {
    SlotSpec { &radar_interfaces::msg::LidarLocation::ally_hero_x,
        &radar_interfaces::msg::LidarLocation::ally_hero_y, "hero" },
    SlotSpec { &radar_interfaces::msg::LidarLocation::ally_engineer_x,
        &radar_interfaces::msg::LidarLocation::ally_engineer_y, "engineer" },
    SlotSpec { &radar_interfaces::msg::LidarLocation::ally_infantry_3_x,
        &radar_interfaces::msg::LidarLocation::ally_infantry_3_y, "infantry3" },
    SlotSpec { &radar_interfaces::msg::LidarLocation::ally_infantry_4_x,
        &radar_interfaces::msg::LidarLocation::ally_infantry_4_y, "infantry4" },
    SlotSpec { &radar_interfaces::msg::LidarLocation::ally_aerial_x,
        &radar_interfaces::msg::LidarLocation::ally_aerial_y, "aerial" },
    SlotSpec { &radar_interfaces::msg::LidarLocation::ally_sentry_x,
        &radar_interfaces::msg::LidarLocation::ally_sentry_y, "sentry" },
};

using SlotQuery = std::function<bool(int camp, const std::string& robot_class, int t,
    DefaultPosition& out)>;

// Fill every unoccupied slot with the default position for its robot class.
// Occupied slots are left untouched. Conversion matches the track path in
// publish_lidar_location: (meters + offset) * 1000 -> uint16 mm.
inline void fill_empty_slots(radar_interfaces::msg::LidarLocation& msg,
    const std::array<SlotSpec, 6>& slots, const std::array<bool, 6>& occupied, int camp, int t,
    double offset_x, double offset_y, const SlotQuery& query) {
    for (std::size_t i = 0; i < slots.size(); ++i) {
        if (occupied[i]) continue;
        DefaultPosition p;
        if (!query(camp, slots[i].robot_class, t, p)) continue;
        msg.*(slots[i].x) = static_cast<std::uint16_t>((p.x_med + offset_x) * 1000.0);
        msg.*(slots[i].y) = static_cast<std::uint16_t>((p.y_med + offset_y) * 1000.0);
    }
}

} // namespace radar_fusion::default_positions
