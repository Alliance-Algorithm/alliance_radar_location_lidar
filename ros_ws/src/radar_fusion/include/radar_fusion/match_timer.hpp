#pragma once

#include <algorithm>
#include <cstdint>

namespace radar_fusion::match_timer {

// Match-round timer driven by /bridge/game_state observations.
//
// Start: game_progress == 4 (比赛中) OR stage_remain_time > 400 (entering the
// 420 s battle stage). Reset: game_progress == 5 (结算) OR
// stage_remain_time < 10 while NOT in the battle phase (during the battle
// the remaining time counts down through <10 without ending the round). A
// message that ends a round never starts the next one (reset takes
// precedence), so the next round re-enters the time series from t = 0 on a
// later start condition.
class MatchTimer {
public:
    // Feed one game_state observation with an explicit local steady-clock
    // reading so the state machine is deterministic and testable without ROS.
    void on_game_state(
        std::uint8_t game_progress, std::uint16_t stage_remain_time, std::int64_t now_ns);

    // Seconds since match start (local clock), clamped >= 0; -1 if this round
    // has not started.
    [[nodiscard]] auto elapsed_sec(std::int64_t now_ns) const -> std::int64_t;

    [[nodiscard]] auto started() const -> bool;

private:
    std::int64_t start_ns_ = 0;
    bool running_          = false;
};

inline void MatchTimer::on_game_state(
    std::uint8_t game_progress, std::uint16_t stage_remain_time, std::int64_t now_ns) {
    const bool in_battle   = game_progress == 4;
    const bool round_ended = game_progress == 5 || (!in_battle && stage_remain_time < 10);
    if (round_ended) {
        running_  = false;
        start_ns_ = 0;
        return;
    }
    if (!running_ && (game_progress == 4 || stage_remain_time > 400)) {
        running_  = true;
        start_ns_ = now_ns;
    }
}

inline auto MatchTimer::elapsed_sec(std::int64_t now_ns) const -> std::int64_t {
    if (!running_) return -1;
    return std::max<std::int64_t>(0, (now_ns - start_ns_) / 1'000'000'000);
}

inline auto MatchTimer::started() const -> bool { return running_; }

} // namespace radar_fusion::match_timer
