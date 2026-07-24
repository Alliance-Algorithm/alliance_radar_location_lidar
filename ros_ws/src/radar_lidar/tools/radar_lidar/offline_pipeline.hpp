#pragma once

#include <expected>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Geometry>

#include "radar_lidar/data_format.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/offline_registration.hpp"

namespace radar_lidar::offline_pipeline {

template <typename PointT>
auto to_rosmsg(const pcl::PointCloud<PointT>& cloud, const std::string& frame_id)
    -> sensor_msgs::msg::PointCloud2;

struct LoadResult {
    std::shared_ptr<const map_data::MapData> map;
    types::Frame frame;
};

// 加载地图（含 Y-up 转 Z-up 兼容）与扫描降采样，发布彩色点云。
auto load_map_and_scan(rclcpp::Node* node, const std::string& output_frame,
    const std::string& scan_frame, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub_map,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub_raw)
    -> std::expected<LoadResult, std::string>;

// Sector ROI 扇形范围滤波。若未启用则无操作，否则裁剪 frame.points 并发布 ROI 彩色云。
auto apply_sector_roi(rclcpp::Node* node, types::Frame& frame, const std::string& scan_frame,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>& pub_roi) -> std::expected<void, std::string>;

} // namespace radar_lidar::offline_pipeline
