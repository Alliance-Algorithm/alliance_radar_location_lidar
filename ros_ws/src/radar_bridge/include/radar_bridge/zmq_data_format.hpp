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
constexpr uint16_t kRadarMarkCmd = 0x1002;
constexpr uint16_t kRadarSyncCmd = 0x1003;

struct TransmitGameState {
    uint16_t cmd_id            = kGameStateCmd;
    uint8_t game_type          = 0;
    uint8_t game_progress      = 0;
    uint16_t stage_remain_time = 0;
    uint64_t sync_timestamp    = 0;
};

struct TransmitRadarMarkProcess {
    uint16_t cmd_id = kRadarMarkCmd;

    // vulnerability / marked : opponent
    uint8_t opponent_hero_vulnerable       = 0;
    uint8_t opponent_engineer_vulnerable   = 0;
    uint8_t opponent_infantry_3_vulnerable = 0;
    uint8_t opponent_infantry_4_vulnerable = 0;
    uint8_t opponent_aerial_marked         = 0;
    uint8_t opponent_sentry_vulnerable     = 0;

    // vulnerability / marked : ally
    uint8_t ally_hero_marked       = 0;
    uint8_t ally_engineer_marked   = 0;
    uint8_t ally_infantry_3_marked = 0;
    uint8_t ally_infantry_4_marked = 0;
    uint8_t ally_aerial_marked     = 0;
    uint8_t ally_sentry_marked     = 0;

    // aerial targeted / countered
    uint8_t opponent_aerial_targeted  = 0;
    uint8_t opponent_aerial_countered = 0;
    uint8_t ally_aerial_targeted      = 0;
    uint8_t ally_aerial_countered     = 0;
};

struct TransmitRadarSync {
    uint16_t cmd_id                = kRadarSyncCmd;
    uint8_t double_weakness_chance = 0;
    uint8_t double_weakness_active = 0;
    uint8_t encryption_rank        = 0;
    uint8_t key_modifiable         = 0;
};

struct GuiData {
    TransmitGameState game_state;
    TransmitRadarMarkProcess radar_mark;
    TransmitRadarSync radar_sync;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    TransmitGameState, cmd_id, game_type, game_progress, stage_remain_time, sync_timestamp)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TransmitRadarMarkProcess, cmd_id, opponent_hero_vulnerable,
    opponent_engineer_vulnerable, opponent_infantry_3_vulnerable, opponent_infantry_4_vulnerable,
    opponent_aerial_marked, opponent_sentry_vulnerable, ally_hero_marked, ally_engineer_marked,
    ally_infantry_3_marked, ally_infantry_4_marked, ally_aerial_marked, ally_sentry_marked,
    opponent_aerial_targeted, opponent_aerial_countered, ally_aerial_targeted,
    ally_aerial_countered)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TransmitRadarSync, cmd_id, double_weakness_chance,
    double_weakness_active, encryption_rank, key_modifiable)

} // namespace radar_bridge::zmqdata::sub

// =====================================================================
// Helper templates (namespace-level, usable by both pub & sub)
// =====================================================================

template <typename T> inline std::string zmq_json_encode(const T& msg) {
    return nlohmann::json(msg).dump();
}

template <typename T> inline T zmq_json_decode(const nlohmann::json& json) { return json.get<T>(); }
