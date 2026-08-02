#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "radar_lidar/data_format.hpp"

namespace radar_lidar::odin_tune {

/// @brief 静态地图背景差分
/// 构造时对地图点云建一次 KdTree；每帧查询，距地图最近邻距离严格大于
/// 阈值判为目标点。地图差分与帧差的关键区别：背景是固定静态地图，
/// 因此静止目标（不在地图中）也能被检出。
class MapDifferencer {
public:
    /// @param map_points 地图点云（地图坐标系）
    /// @param distance_threshold 距离阈值 (m)，>0
    MapDifferencer(const types::PointCloud& map_points, double distance_threshold)
        : distance_threshold_(distance_threshold) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        map_cloud->reserve(map_points.size());
        for (const auto& p : map_points) {
            map_cloud->emplace_back(
                static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
        }
        map_cloud->width    = map_cloud->size();
        map_cloud->height   = 1;
        map_cloud->is_dense = true;
        kd_tree_.setInputCloud(map_cloud);
    }

    /// @brief 返回 scan 中距地图最近邻距离严格大于阈值的点（目标点）
    /// @param scan 扫描点云（须已变换到地图坐标系）
    auto differ(const types::PointCloud& scan) const -> types::PointCloud {
        if (kd_tree_.getInputCloud() == nullptr || kd_tree_.getInputCloud()->empty()) {
            return scan;
        }
        const float thresh_sq = static_cast<float>(distance_threshold_ * distance_threshold_);
        types::PointCloud result;
        result.reserve(scan.size());
        std::vector<int> idx(1);
        std::vector<float> dist_sq(1);
        for (const auto& p : scan) {
            const pcl::PointXYZ query(static_cast<float>(p.x()),
                static_cast<float>(p.y()), static_cast<float>(p.z()));
            if (kd_tree_.nearestKSearch(query, 1, idx, dist_sq) > 0 && dist_sq[0] > thresh_sq) {
                result.push_back(p);
            }
        }
        return result;
    }

    void set_distance_threshold(double t) {
        distance_threshold_ = t;
    }

private:
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree_;
    double distance_threshold_;
};

} // namespace radar_lidar::odin_tune
