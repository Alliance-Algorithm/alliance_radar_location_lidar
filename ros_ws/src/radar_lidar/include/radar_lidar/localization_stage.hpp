#pragma once

#include <expected>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "radar_lidar/data_format.hpp"
#include "radar_lidar/frame_accumulator.hpp"
#include "radar_lidar/spherical_grid.hpp"

namespace radar_lidar::map_data {
class MapData;
}

namespace radar_lidar::localization {

/// @brief Stage 1: 点云 → 位姿 (GICP scan-to-map)
/// 支持球面网格预处理 + 帧累积 + 一次性锁定
class LocalizationStage {
public:
    LocalizationStage(std::shared_ptr<const map_data::MapData> map, config::LocalizationConfig cfg);

    /// @brief 对单帧点云执行 GICP 配准
    /// @param scan 当前帧点云
    /// @return 位姿估计 或 错误信息
    auto process(const types::Frame& scan) -> std::expected<types::PoseEstimate, std::string>;

    /// @brief 设置下一次配准的初始位姿猜测
    /// @brief 离线工具和重定位场景需要外部提供初始猜测
    void set_initial_pose(const Eigen::Isometry3d& pose) { prev_pose_ = pose; }

    /// @brief 重置为单位位姿（重新开始跟踪）
    void reset() {
        prev_pose_ = Eigen::Isometry3d::Identity();
        locked_    = false;
        accumulator_.clear();
    }

    /// @brief 是否已锁定（use_lock_strategy 启用时）
    [[nodiscard]] auto is_locked() const -> bool { return locked_; }

private:
    auto preprocess(const types::Frame& scan) -> types::PointCloud;

    /// @brief 锁定后 watchdog：用当前 scan 在锁定位姿下对地图的最近邻残差
    /// 检测雷达站是否被移动/碰撞。超标连续帧达到阈值 → 解锁并返回 true。
    auto watchdog_check(const types::Frame& scan) -> bool;

    /// @brief coarse 重定位：yaw + 平移多起点搜索，inlier 选优后精配。
    /// 成功返回 true 并更新 prev_pose_（重新锁定）。
    auto coarse_relocalize(const types::Frame& scan) -> bool;

    /// @brief 对齐评分：scan 变换到 map 后，inlier 比 + RMSE
    struct AlignmentScore {
        double inlier_ratio = 0.0;
        double rmse         = 0.0;
    };
    auto score_alignment(const types::PointCloud& scan, const Eigen::Isometry3d& T) const
        -> AlignmentScore;

    std::shared_ptr<const map_data::MapData> map_;
    config::LocalizationConfig cfg_;
    Eigen::Isometry3d prev_pose_;
    std::vector<Eigen::Vector3d> target_points_; // 缓存的地图点，构造时一次提取

    spherical_grid::SphericalGrid spherical_grid_;
    frame_accumulator::FrameAccumulator accumulator_;
    bool locked_ = false;

    // watchdog 状态
    std::size_t watchdog_frame_count_ = 0;
    int watchdog_high_residual_frames_ = 0;
};

} // namespace radar_lidar::localization
