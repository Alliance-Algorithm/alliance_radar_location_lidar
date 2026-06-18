#pragma once

#include <expected>
#include <memory>
#include <string>

#include <Eigen/Geometry>

#include "radar_lidar/config.hpp"
#include "radar_lidar/types.hpp"

namespace radar {

class MapData;

/// @brief Stage 1: 点云 → 位姿 (GICP scan-to-map)
class LocalizationStage {
public:
    LocalizationStage(std::shared_ptr<const MapData> map, config::LocalizationConfig cfg);

    /// @brief 对单帧点云执行 GICP 配准
    /// @param scan 当前帧点云
    /// @return 位姿估计 或 错误信息
    auto process(const types::Frame& scan) -> std::expected<types::PoseEstimate, std::string>;

private:
    std::shared_ptr<const MapData>   map_;
    config::LocalizationConfig       cfg_;
    Eigen::Isometry3d                prev_pose_;
    std::vector<Eigen::Vector3d>     target_points_;  // 缓存的地图点，构造时一次提取
};

}  // namespace radar
