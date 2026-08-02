#pragma once

#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "radar_lidar/data_format.hpp"

namespace radar_lidar::odin_tune {

/// @brief 当前帧 vs 背景模型 KdTree 最近邻差分
/// 背景模型须已对齐到当前帧坐标系；距离严格大于阈值判为动态点。
class FrameDifferencer {
public:
    explicit FrameDifferencer(double distance_threshold)
        : distance_threshold_(distance_threshold) { }

    auto differ(const types::PointCloud& current, const types::PointCloud& background) const
        -> types::PointCloud {
        if (background.empty()) {
            return { };
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr bg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        bg_cloud->reserve(background.size());
        for (const auto& p : background) {
            bg_cloud->emplace_back(
                static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
        }
        bg_cloud->width    = bg_cloud->size();
        bg_cloud->height   = 1;
        bg_cloud->is_dense = true;

        pcl::KdTreeFLANN<pcl::PointXYZ> tree;
        tree.setInputCloud(bg_cloud);

        const float thresh_sq = static_cast<float>(distance_threshold_ * distance_threshold_);

        types::PointCloud result;
        result.reserve(current.size());
        std::vector<int> idx(1);
        std::vector<float> dist_sq(1);
        for (const auto& p : current) {
            const pcl::PointXYZ query(
                static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
            if (tree.nearestKSearch(query, 1, idx, dist_sq) > 0 && dist_sq[0] > thresh_sq) {
                result.push_back(p);
            }
        }
        return result;
    }

    void set_distance_threshold(double t) { distance_threshold_ = t; }

private:
    double distance_threshold_;
};

} // namespace radar_lidar::odin_tune
