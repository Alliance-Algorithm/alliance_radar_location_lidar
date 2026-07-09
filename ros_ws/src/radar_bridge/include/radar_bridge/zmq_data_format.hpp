#pragma once

// ── ZMQ data format — mirrors radar-egui/src/zmq/data_format.rs ──
//
// PUB: radar_bridge (C++)  ──→  radar-egui (Rust)
// SUB: radar_bridge (C++)  ←──  radar-egui (Rust)
//
// ⚠ 该仓库只负责 LiDAR 定位数据输出，
//    SDR 和 Laser 由其他模块提供，不在此文件定义。

#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

// =====================================================================
// PUB — radar_bridge → radar-egui
// =====================================================================
namespace radar_bridge::zmqdata::pub {

constexpr uint16_t kLidarLocationCmd = 0x2001;

struct LidarLocation {
    uint16_t cmd_id = kLidarLocationCmd;

    // opponent
    uint16_t opponent_hero_x       = 0;
    uint16_t opponent_hero_y       = 0;
    uint16_t opponent_engineer_x   = 0;
    uint16_t opponent_engineer_y   = 0;
    uint16_t opponent_infantry_3_x = 0;
    uint16_t opponent_infantry_3_y = 0;
    uint16_t opponent_infantry_4_x = 0;
    uint16_t opponent_infantry_4_y = 0;
    uint16_t opponent_aerial_x     = 0;
    uint16_t opponent_aerial_y     = 0;
    uint16_t opponent_sentry_x     = 0;
    uint16_t opponent_sentry_y     = 0;

    // ally
    uint16_t ally_hero_x       = 0;
    uint16_t ally_hero_y       = 0;
    uint16_t ally_engineer_x   = 0;
    uint16_t ally_engineer_y   = 0;
    uint16_t ally_infantry_3_x = 0;
    uint16_t ally_infantry_3_y = 0;
    uint16_t ally_infantry_4_x = 0;
    uint16_t ally_infantry_4_y = 0;
    uint16_t ally_aerial_x     = 0;
    uint16_t ally_aerial_y     = 0;
    uint16_t ally_sentry_x     = 0;
    uint16_t ally_sentry_y     = 0;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(LidarLocation, cmd_id, opponent_hero_x, opponent_hero_y,
    opponent_engineer_x, opponent_engineer_y, opponent_infantry_3_x, opponent_infantry_3_y,
    opponent_infantry_4_x, opponent_infantry_4_y, opponent_aerial_x, opponent_aerial_y,
    opponent_sentry_x, opponent_sentry_y, ally_hero_x, ally_hero_y, ally_engineer_x,
    ally_engineer_y, ally_infantry_3_x, ally_infantry_3_y, ally_infantry_4_x, ally_infantry_4_y,
    ally_aerial_x, ally_aerial_y, ally_sentry_x, ally_sentry_y)

} // namespace radar_bridge::zmqdata::pub

// =====================================================================
// SUB — radar-egui → radar_bridge
// =====================================================================
namespace radar_bridge::zmqdata::sub {

constexpr uint16_t kGameStateCmd = 0x1001;

struct TransmitGameState {
    uint16_t cmd_id            = kGameStateCmd;
    uint8_t game_type          = 0;
    uint8_t game_progress      = 0;
    uint16_t stage_remain_time = 0;
    uint64_t sync_timestamp    = 0;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    TransmitGameState, cmd_id, game_type, game_progress, stage_remain_time, sync_timestamp)

} // namespace radar_bridge::zmqdata::sub

// =====================================================================
// Helper templates (namespace-level, usable by both pub & sub)
// =====================================================================

template <typename T> inline std::string zmq_json_encode(const T& msg) {
    return nlohmann::json(msg).dump();
}

template <typename T> inline T zmq_json_decode(const nlohmann::json& json) { return json.get<T>(); }
