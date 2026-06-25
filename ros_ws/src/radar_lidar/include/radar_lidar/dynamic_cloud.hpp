#pragma once

#include <deque>
#include <expected>
#include <memory>
#include <string>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "radar_lidar/types.hpp"

namespace radar {

struct DynamicCloudConfig {
    float distance_threshold = 0.1f; // KdTree 最近邻距离²阈值，大于则为动态点
    int num_threads          = 12;
    int accumulate_frames    = 3; // 滑动窗口帧数
    bool use_roi             = true;
    float roi_x_min = 0, roi_x_max = 30;
    float roi_y_min = -15, roi_y_max = 15;
    float roi_z_min = 0, roi_z_max = 1.4f;
};

/// @brief 动态点提取
/// 用 KdTree K=1 最近邻把扫描和地图对比，距离 > 阈值的点 = 动态点（障碍物）
/// 支持 ROI 裁剪 + 多帧累积，可独立于 ROS 使用
class DynamicCloudStage {
public:
    explicit DynamicCloudStage(DynamicCloudConfig cfg);

    /// @brief 设置地图点云（用于 KdTree 构建）
    void set_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_cloud);

    /// @brief 处理一帧扫描，返回动态点
    /// @param scan 已变换到地图坐标系的扫描点云
    auto process(const types::PointCloud& scan) -> std::expected<types::PointCloud, std::string>;

    /// @brief 当前累积的动态点（合并所有帧）
    [[nodiscard]] auto accumulated() const -> types::PointCloud;

    /// @brief 清空累积
    void clear() { frames_.clear(); }

private:
    DynamicCloudConfig cfg_;
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_;
    bool map_set_ = false;

    std::deque<types::PointCloud> frames_;
};

} // namespace radar
