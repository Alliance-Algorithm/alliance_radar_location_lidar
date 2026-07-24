#include "radar_lidar/offline_pipeline.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <format>
#include <print>

#include "radar_lidar/geometry_utils.hpp"
#include "radar_lidar/offline_registration.hpp"
#include "radar_lidar/offline_visualization.hpp"

namespace radar_lidar::offline_pipeline {

namespace reg = radar_lidar::registration;

template <typename PointT>
auto to_rosmsg(const pcl::PointCloud<PointT>& cloud, const std::string& frame_id)
    -> sensor_msgs::msg::PointCloud2 {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame_id;
    return msg;
}

template auto to_rosmsg<pcl::PointXYZ>(const pcl::PointCloud<pcl::PointXYZ>&, const std::string&)
    -> sensor_msgs::msg::PointCloud2;
template auto to_rosmsg<pcl::PointXYZRGB>(
    const pcl::PointCloud<pcl::PointXYZRGB>&, const std::string&) -> sensor_msgs::msg::PointCloud2;

auto load_map_and_scan(rclcpp::Node* node, const std::string& output_frame,
    const std::string& scan_frame, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub_map,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub_raw)
    -> std::expected<LoadResult, std::string> {
    std::string map_path, scan_path;
    node->get_parameter("map_path", map_path);
    node->get_parameter("scan_path", scan_path);
    if (map_path.empty() || scan_path.empty())
        return std::unexpected("map_path and scan_path required");

    double voxel_leaf = 0.1, scan_voxel = 0.1;
    bool map_y_up = false;
    node->get_parameter("voxel_leaf", voxel_leaf);
    node->get_parameter("scan_voxel", scan_voxel);
    node->get_parameter("map_y_up", map_y_up);

    if (map_y_up) {
        std::println(stderr,
            "[offline] WARNING: map_y_up=true is deprecated, use model_to_map --y-up for Z-up "
            "map.");
        auto raw = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *raw) == -1)
            return std::unexpected("Failed to load map PCD for rotation");
        for (auto& pt : raw->points) {
            const float y = pt.y, z = pt.z;
            pt.y = -z;
            pt.z = y;
        }
        map_path = "/tmp/map_z_up.pcd";
        pcl::io::savePCDFileBinary(map_path, *raw);
        std::println("[offline] Map rotated Y-up->Z-up (deprecated), saved to {}", map_path);
    }

    auto map_result = map_data::MapData::load(map_path, voxel_leaf);
    if (!map_result) return std::unexpected(std::format("Map load failed: {}", map_result.error()));
    auto map = *map_result;
    std::println("[offline] Map: {} points", map->size());
    pub_map.publish(to_rosmsg(
        offline::make_colored_cloud(map->pcl_cloud(), offline::kMapColorBgr), output_frame));

    auto scan_pcl = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_path, *scan_pcl) == -1)
        return std::unexpected(std::format("Failed to load scan: {}", scan_path));
    if (scan_voxel > 0.0) {
        auto downsampled = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(scan_voxel, scan_voxel, scan_voxel);
        vg.setInputCloud(scan_pcl);
        vg.filter(*downsampled);
        std::println("[offline] Scan downsampled: {} -> {} points (voxel={})", scan_pcl->size(),
            downsampled->size(), scan_voxel);
        scan_pcl = downsampled;
    }

    auto frame = geom::filter_valid_points(*scan_pcl);
    std::println("[offline] Scan: {} valid points", frame.points.size());
    if (frame.points.size() < 100) return std::unexpected("Too few points");

    pub_raw.publish(to_rosmsg(
        offline::make_colored_cloud(frame.points, offline::kRawScanColorBgr), scan_frame));
    return LoadResult { map, std::move(frame) };
}

auto apply_sector_roi(rclcpp::Node* node, types::Frame& frame, const std::string& scan_frame,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub_roi) -> std::expected<void, std::string> {
    bool use_roi = false;
    node->get_parameter("use_roi", use_roi);
    if (!use_roi) return { };

    double roi_half_fov_deg = 60.0, roi_min_range = 1.0, roi_max_range = 30.0;
    double roi_z_min = 0.0, roi_z_max = 7.0;
    double roi_origin_x = 0.0, roi_origin_y = 0.0, roi_origin_yaw_deg = 0.0;
    node->get_parameter("roi_half_fov_deg", roi_half_fov_deg);
    node->get_parameter("roi_min_range", roi_min_range);
    node->get_parameter("roi_max_range", roi_max_range);
    node->get_parameter("roi_z_min", roi_z_min);
    node->get_parameter("roi_z_max", roi_z_max);
    node->get_parameter("roi_origin_x", roi_origin_x);
    node->get_parameter("roi_origin_y", roi_origin_y);
    node->get_parameter("roi_origin_yaw_deg", roi_origin_yaw_deg);

    const double yaw_rad   = reg::deg_to_rad(roi_origin_yaw_deg);
    const double cos_yaw   = std::cos(yaw_rad);
    const double sin_yaw   = std::sin(yaw_rad);
    const double half_fov  = reg::deg_to_rad(roi_half_fov_deg);
    const double tan2_half = std::tan(half_fov) * std::tan(half_fov);
    const double min_r2    = roi_min_range * roi_min_range;
    const double max_r2    = roi_max_range * roi_max_range;

    auto clipped = types::PointCloud();
    clipped.reserve(frame.points.size());
    for (const auto& p : frame.points) {
        if (p.z() < roi_z_min || p.z() > roi_z_max) continue;
        const double dx = p.x() - roi_origin_x;
        const double dy = p.y() - roi_origin_y;
        const double r2 = dx * dx + dy * dy;
        if (r2 < min_r2 || r2 > max_r2) continue;
        const double fx = dx * cos_yaw + dy * sin_yaw;
        const double fy = -dx * sin_yaw + dy * cos_yaw;
        if (fx <= 0.0) continue;
        if (fy * fy > fx * fx * tan2_half) continue;
        clipped.push_back(p);
    }
    std::println("[offline] ROI sector: {} -> {} points (origin=({:.1f},{:.1f}) yaw={} FOV=+/-{} "
                 "deg)",
        frame.points.size(), clipped.size(), roi_origin_x, roi_origin_y, roi_origin_yaw_deg,
        roi_half_fov_deg);
    if (clipped.size() < 100) return std::unexpected("Too few points after ROI");

    pub_roi.publish(
        to_rosmsg(offline::make_colored_cloud(clipped, offline::kRoiColorBgr), scan_frame));
    frame.points = std::move(clipped);
    return { };
}

} // namespace radar_lidar::offline_pipeline
