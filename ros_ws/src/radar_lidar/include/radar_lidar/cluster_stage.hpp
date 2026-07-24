#pragma once

#include <expected>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "radar_lidar/data_format.hpp"

namespace radar_lidar::cluster {

/// @brief 欧氏聚类 + 质心 + AABB 边界框
/// 输出质心 + AABB 边界框
class ClusterStage {
public:
    explicit ClusterStage(config::ClusterConfig cfg);

    /// @brief 对动态点云执行聚类
    /// @param dynamic_points 动态点（来自 DynamicCloudStage）
    /// @return 聚类结果列表（每个聚类一个质心 + AABB）
    auto process(const types::PointCloud& dynamic_points)
        -> std::expected<std::vector<ClusterResult>, std::string>;

private:
    config::ClusterConfig cfg_;
};

} // namespace radar_lidar::cluster
