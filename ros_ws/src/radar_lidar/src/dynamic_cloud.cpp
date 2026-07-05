#include "radar_lidar/dynamic_cloud.hpp"

#include <algorithm>
#include <omp.h>
#include <ranges>

#include <pcl/filters/voxel_grid.h>

#include "radar_lidar/geometry_utils.hpp"

namespace radar {

DynamicCloudStage::DynamicCloudStage(DynamicCloudConfig cfg)
    : cfg_(std::move(cfg))
    , thread_count_(std::max(1, cfg_.num_threads))
    , thread_clouds_(static_cast<std::size_t>(thread_count_))
    , thread_indices_(static_cast<std::size_t>(thread_count_), std::vector<int>(1))
    , thread_dist_sq_(static_cast<std::size_t>(thread_count_), std::vector<float>(1)) { }

void DynamicCloudStage::set_map(const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_cloud) {
    // 下采样地图用于 KdTree（0.1m voxel）
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.setInputCloud(map_cloud);
    vg.filter(*filtered);
    kd_tree_.setInputCloud(filtered);
    map_set_ = true;
}

auto DynamicCloudStage::process(const types::PointCloud& scan)
    -> std::expected<types::PointCloud, std::string> {
    if (!map_set_) {
        return std::unexpected("Map not set for DynamicCloudStage");
    }
    if (scan.empty()) {
        return std::unexpected("Empty scan for DynamicCloudStage");
    }

    // ROI 裁剪
    auto roi_points = geom::clip_roi_aabb(scan, cfg_.roi);

    if (roi_points.empty()) {
        return types::PointCloud { };
    }

    // KdTree K=1 最近邻，多线程并行（缓冲区在构造时预分配，此处仅清空复用）
    const auto reserve_per_thread =
        std::max<std::size_t>(1, roi_points.size() / static_cast<std::size_t>(thread_count_));
    for (auto& thread_cloud : thread_clouds_) {
        thread_cloud.clear();
        thread_cloud.reserve(reserve_per_thread);
    }

#pragma omp parallel for num_threads(thread_count_) schedule(static)
    for (size_t i = 0; i < roi_points.size(); ++i) {
        int tid = omp_get_thread_num();
        pcl::PointXYZ query;
        query.x = static_cast<float>(roi_points[i].x());
        query.y = static_cast<float>(roi_points[i].y());
        query.z = static_cast<float>(roi_points[i].z());

        auto& idx     = thread_indices_[tid];
        auto& dist_sq = thread_dist_sq_[tid];
        if (kd_tree_.nearestKSearch(query, 1, idx, dist_sq) > 0) {
            if (dist_sq[0] > cfg_.distance_threshold * cfg_.distance_threshold) {
                thread_clouds_[tid].push_back(roi_points[i]);
            }
        }
    }

    // 合并线程结果
    auto dynamic_points = thread_clouds_ | std::views::join | std::ranges::to<types::PointCloud>();

    // 帧累积
    if (cfg_.accumulate_frames > 0) {
        frames_.push_back(std::move(dynamic_points));
        while (frames_.size() > static_cast<size_t>(cfg_.accumulate_frames)) {
            frames_.pop_front();
        }
    } else {
        frames_.clear();
        frames_.push_back(dynamic_points);
    }

    return accumulated();
}

auto DynamicCloudStage::accumulated() const -> types::PointCloud {
    return frames_ | std::views::join | std::ranges::to<types::PointCloud>();
}

} // namespace radar
