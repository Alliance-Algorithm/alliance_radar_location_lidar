#pragma once

// ── ZMQ data format — mirrors radar-egui/src/zmq/data_format.rs ──
//
// PUB: radar_bridge (C++)  ──→  radar-egui (Rust)
// SUB: radar_bridge (C++)  ←──  radar-egui (Rust)
//
// ⚠ 该仓库只负责 LiDAR 定位数据输出，
//    SDR 和 Laser 由其他模块提供，不在此文件定义。

#include <array>
#include <cstdint>
#include <string>

#include <nlohmann/json.hpp>

// =====================================================================
// Message-type constants
// =====================================================================

// PUB (radar_bridge → radar-egui) — 本仓库负责输出的数据
constexpr uint16_t kZmqSubLidarLocation = 0x2001;

// SUB (radar-egui → radar_bridge) — 接收 egui 决策指令
constexpr uint16_t kZmqPubGameState     = 0x1001;
constexpr uint16_t kZmqPubRadarMark     = 0x1002;
constexpr uint16_t kZmqPubRadarSync     = 0x1003;

// =====================================================================
// PUB transmit — radar_bridge → radar-egui
// =====================================================================

/// LiDAR 目标定位数据 (kZmqSubLidarLocation)
struct ReceiveLidarLocation {
    uint16_t cmd_id              = kZmqSubLidarLocation;
    uint16_t opponent_hero_x     = 0;
    uint16_t opponent_hero_y     = 0;
    uint16_t opponent_engineer_x = 0;
    uint16_t opponent_engineer_y = 0;
    uint16_t opponent_infantry_3_x = 0;
    uint16_t opponent_infantry_3_y = 0;
    uint16_t opponent_infantry_4_x = 0;
    uint16_t opponent_infantry_4_y = 0;
    uint16_t opponent_aerial_x   = 0;
    uint16_t opponent_aerial_y   = 0;
    uint16_t opponent_sentry_x   = 0;
    uint16_t opponent_sentry_y   = 0;
    uint16_t ally_hero_x         = 0;
    uint16_t ally_hero_y         = 0;
    uint16_t ally_engineer_x     = 0;
    uint16_t ally_engineer_y     = 0;
    uint16_t ally_infantry_3_x   = 0;
    uint16_t ally_infantry_3_y   = 0;
    uint16_t ally_infantry_4_x   = 0;
    uint16_t ally_infantry_4_y   = 0;
    uint16_t ally_aerial_x       = 0;
    uint16_t ally_aerial_y       = 0;
    uint16_t ally_sentry_x       = 0;
    uint16_t ally_sentry_y       = 0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ReceiveLidarLocation,
    cmd_id,
    opponent_hero_x, opponent_hero_y,
    opponent_engineer_x, opponent_engineer_y,
    opponent_infantry_3_x, opponent_infantry_3_y,
    opponent_infantry_4_x, opponent_infantry_4_y,
    opponent_aerial_x, opponent_aerial_y,
    opponent_sentry_x, opponent_sentry_y,
    ally_hero_x, ally_hero_y,
    ally_engineer_x, ally_engineer_y,
    ally_infantry_3_x, ally_infantry_3_y,
    ally_infantry_4_x, ally_infantry_4_y,
    ally_aerial_x, ally_aerial_y,
    ally_sentry_x, ally_sentry_y)

// =====================================================================
// SUB receive — radar-egui → radar_bridge
// =====================================================================

/// 游戏状态广播 (kZmqPubGameState)
struct TransmitGameState {
    uint16_t cmd_id            = kZmqPubGameState;
    uint8_t  game_type         = 0;
    uint8_t  game_progress     = 0;
    uint16_t stage_remain_time = 0;
    uint64_t sync_timestamp    = 0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TransmitGameState,
    cmd_id, game_type, game_progress, stage_remain_time, sync_timestamp)

/// 雷达标记进度广播 (kZmqPubRadarMark)
struct TransmitRadarMarkProcess {
    uint16_t cmd_id                         = kZmqPubRadarMark;
    uint8_t  opponent_hero_vulnerable       = 0;
    uint8_t  opponent_engineer_vulnerable   = 0;
    uint8_t  opponent_infantry_3_vulnerable = 0;
    uint8_t  opponent_infantry_4_vulnerable = 0;
    uint8_t  opponent_aerial_marked         = 0;
    uint8_t  opponent_sentry_vulnerable     = 0;
    uint8_t  ally_hero_marked               = 0;
    uint8_t  ally_engineer_marked           = 0;
    uint8_t  ally_infantry_3_marked         = 0;
    uint8_t  ally_infantry_4_marked         = 0;
    uint8_t  ally_aerial_marked             = 0;
    uint8_t  ally_sentry_marked             = 0;
    uint8_t  opponent_aerial_targeted       = 0;
    uint8_t  opponent_aerial_countered      = 0;
    uint8_t  ally_aerial_targeted           = 0;
    uint8_t  ally_aerial_countered          = 0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TransmitRadarMarkProcess,
    cmd_id,
    opponent_hero_vulnerable, opponent_engineer_vulnerable,
    opponent_infantry_3_vulnerable, opponent_infantry_4_vulnerable,
    opponent_aerial_marked, opponent_sentry_vulnerable,
    ally_hero_marked, ally_engineer_marked,
    ally_infantry_3_marked, ally_infantry_4_marked,
    ally_aerial_marked, ally_sentry_marked,
    opponent_aerial_targeted, opponent_aerial_countered,
    ally_aerial_targeted, ally_aerial_countered)

/// 雷达自主决策同步广播 (kZmqPubRadarSync)
struct TransmitRadarSync {
    uint16_t cmd_id                = kZmqPubRadarSync;
    uint8_t  double_weakness_chance = 0;
    uint8_t  double_weakness_active = 0;
    uint8_t  encryption_rank       = 0;
    uint8_t  key_modifiable        = 0;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TransmitRadarSync,
    cmd_id, double_weakness_chance, double_weakness_active,
    encryption_rank, key_modifiable)

// =====================================================================
// Helper templates
// =====================================================================

template <typename T>
inline std::string zmq_json_encode(const T& msg) {
    return nlohmann::json(msg).dump();
}

template <typename T>
inline T zmq_json_decode(const std::string& json_str) {
    return nlohmann::json::parse(json_str).get<T>();
}
