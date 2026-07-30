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

enum class RegistrationState { INITIALIZING, REGISTERING, LOCKED, FAILED };

class LocalizationStageTestPeer;

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
    void set_initial_pose(const Eigen::Isometry3d& pose) {
        prev_pose_ = pose;
        locked_    = false;
        state_     = target_points_.empty() ? RegistrationState::INITIALIZING
                                            : RegistrationState::REGISTERING;
        failure_reason_.clear();
    }

    /// @brief 重置为单位位姿（重新开始跟踪）
    void reset() {
        prev_pose_ = Eigen::Isometry3d::Identity();
        locked_    = false;
        state_     = target_points_.empty() ? RegistrationState::INITIALIZING
                                            : RegistrationState::REGISTERING;
        failure_reason_.clear();
        accumulator_.clear();
    }

    /// @brief 是否已锁定（use_lock_strategy 启用时）
    [[nodiscard]] auto is_locked() const -> bool { return locked_; }
    [[nodiscard]] auto state() const noexcept -> RegistrationState { return state_; }
    [[nodiscard]] auto failure_reason() const -> const std::string& { return failure_reason_; }

private:
    friend class LocalizationStageTestPeer;

    auto preprocess(const types::Frame& scan) -> types::PointCloud;
    void apply_registration_result(const types::PoseEstimate& result);

    std::shared_ptr<const map_data::MapData> map_;
    config::LocalizationConfig cfg_;
    Eigen::Isometry3d prev_pose_;
    std::vector<Eigen::Vector3d> target_points_; // 缓存的地图点，构造时一次提取

    spherical_grid::SphericalGrid spherical_grid_;
    frame_accumulator::FrameAccumulator accumulator_;
    bool locked_             = false;
    RegistrationState state_ = RegistrationState::INITIALIZING;
    std::string failure_reason_;
};

} // namespace radar_lidar::localization
