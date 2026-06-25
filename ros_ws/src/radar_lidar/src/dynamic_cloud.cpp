#include "radar_lidar/dynamic_cloud.hpp"

#include <algorithm>
#include <omp.h>

#include <pcl/filters/voxel_grid.h>

namespace radar {

DynamicCloudStage::DynamicCloudStage(DynamicCloudConfig cfg)
    : cfg_(std::move(cfg)) { }

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
    types::PointCloud roi_points;
    if (cfg_.use_roi) {
        roi_points.reserve(scan.size());
        for (const auto& p : scan) {
            if (p.x() >= cfg_.roi_x_min && p.x() <= cfg_.roi_x_max && p.y() >= cfg_.roi_y_min
                && p.y() <= cfg_.roi_y_max && p.z() >= cfg_.roi_z_min && p.z() <= cfg_.roi_z_max) {
                roi_points.push_back(p);
            }
        }
    } else {
        roi_points = scan;
    }

    if (roi_points.empty()) {
        return types::PointCloud { };
    }

    // KdTree K=1 最近邻，多线程并行
    const int thread_count = std::max(1, cfg_.num_threads);
    std::vector<types::PointCloud> thread_clouds(static_cast<std::size_t>(thread_count));
    std::vector<std::vector<int>> thread_indices(
        static_cast<std::size_t>(thread_count), std::vector<int>(1));
    std::vector<std::vector<float>> thread_dist_sq(
        static_cast<std::size_t>(thread_count), std::vector<float>(1));
    const auto reserve_per_thread =
        std::max<std::size_t>(1, roi_points.size() / static_cast<std::size_t>(thread_count));
    for (auto& thread_cloud : thread_clouds) {
        thread_cloud.reserve(reserve_per_thread);
    }

#pragma omp parallel for num_threads(thread_count) schedule(static)
    for (size_t i = 0; i < roi_points.size(); ++i) {
        int tid = omp_get_thread_num();
        pcl::PointXYZ query;
        query.x = static_cast<float>(roi_points[i].x());
        query.y = static_cast<float>(roi_points[i].y());
        query.z = static_cast<float>(roi_points[i].z());

        auto& idx     = thread_indices[tid];
        auto& dist_sq = thread_dist_sq[tid];
        if (kd_tree_.nearestKSearch(query, 1, idx, dist_sq) > 0) {
            if (dist_sq[0] > cfg_.distance_threshold * cfg_.distance_threshold) {
                thread_clouds[tid].push_back(roi_points[i]);
            }
        }
    }

    // 合并线程结果
    types::PointCloud dynamic_points;
    for (const auto& tc : thread_clouds) {
        dynamic_points.insert(dynamic_points.end(), tc.begin(), tc.end());
    }

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
    types::PointCloud result;
    size_t total = 0;
    for (const auto& f : frames_)
        total += f.size();
    result.reserve(total);
    for (const auto& f : frames_) {
        result.insert(result.end(), f.begin(), f.end());
    }
    return result;
}

} // namespace radar
