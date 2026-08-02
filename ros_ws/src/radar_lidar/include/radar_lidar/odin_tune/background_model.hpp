#pragma once

#include <deque>
#include <utility>

#include <Eigen/Geometry>

#include "radar_lidar/data_format.hpp"

namespace radar_lidar::odin_tune {

/// @brief 滑动窗口帧缓存：缓存 (雷达系点云, odom 位姿)，支持对齐到任意目标位姿
/// 对齐公式：p_target = T_odom_target^{-1} * T_odom_i * p_i
class BackgroundModel {
public:
    explicit BackgroundModel(int max_frames)
        : max_frames_(max_frames) { }

    void add(const types::PointCloud& points, const Eigen::Isometry3d& odom_pose) {
        frames_.emplace_back(Frame { points, odom_pose });
        while (static_cast<int>(frames_.size()) > max_frames_) {
            frames_.pop_front();
        }
    }

    auto align_to(const Eigen::Isometry3d& target_pose) const -> types::PointCloud {
        const Eigen::Isometry3d target_inv = target_pose.inverse();
        types::PointCloud result;
        for (const auto& frame : frames_) {
            const Eigen::Isometry3d rel = target_inv * frame.pose;
            result.reserve(result.size() + frame.points.size());
            for (const auto& p : frame.points) {
                result.push_back(rel * p);
            }
        }
        return result;
    }

    auto frame_count() const -> int { return static_cast<int>(frames_.size()); }

    void clear() { frames_.clear(); }

private:
    struct Frame {
        types::PointCloud points;
        Eigen::Isometry3d pose;
    };
    std::deque<Frame> frames_;
    int max_frames_;
};

} // namespace radar_lidar::odin_tune
