#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <utility>

#include <Eigen/Geometry>

#include "radar_lidar/data_format.hpp"

namespace radar_lidar::odin_tune {

/// @brief 按时间戳缓存 odometry 位姿，支持最近邻查询
/// 时间戳须递增插入（Odin odometry 按序到达）；内部按时间差丢弃过旧条目。
class PoseBuffer {
public:
    /// @param max_span_ns 缓存条目的最大时间跨度（ns），超过视为失配
    explicit PoseBuffer(std::int64_t max_span_ns = 2'000'000'000LL)
        : max_span_ns_(max_span_ns) { }

    void add(types::Timestamp stamp, const Eigen::Isometry3d& pose) {
        entries_.emplace_back(stamp, pose);
        while (entries_.size() > 1
            && stamp - entries_.front().first > max_span_ns_) {
            entries_.pop_front();
        }
    }

    auto lookup(types::Timestamp stamp) const -> std::optional<Eigen::Isometry3d> {
        if (entries_.empty()) {
            return std::nullopt;
        }
        const auto* best = &entries_.front();
        std::int64_t best_delta = std::llabs(stamp - best->first);
        for (const auto& entry : entries_) {
            const std::int64_t delta = std::llabs(stamp - entry.first);
            if (delta < best_delta) {
                best_delta = delta;
                best       = &entry;
            }
        }
        if (best_delta > max_span_ns_) {
            return std::nullopt;
        }
        return best->second;
    }

    void clear() {
        entries_.clear();
    }

private:
    std::deque<std::pair<types::Timestamp, Eigen::Isometry3d>> entries_;
    std::int64_t max_span_ns_;
};

} // namespace radar_lidar::odin_tune
