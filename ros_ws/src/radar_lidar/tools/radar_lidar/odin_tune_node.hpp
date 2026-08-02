#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "radar_lidar/cluster_stage.hpp"
#include "radar_lidar/data_format.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/odin_tune/background_model.hpp"
#include "radar_lidar/odin_tune/frame_differencer.hpp"
#include "radar_lidar/odin_tune/map_differencer.hpp"
#include "radar_lidar/odin_tune/pose_buffer.hpp"

namespace radar_lidar::node {

/// @brief Odin1 直出点云聚类调参节点（无地图帧差 / 有地图静态背景两种模式）
/// 订阅 /odin1/cloud_raw + odometry，发布 /odin_tune/*；参数实时可调。
/// 地图模式（map_path 非空）：雷达系点云经固定安装位姿变换到地图系，
/// 与静态地图 KdTree 差分，动静目标均可检出。
class OdinTuneNode : public rclcpp::Node {
public:
    OdinTuneNode();

private:
    struct Params {
        std::string scan_topic { "/odin1/cloud_raw" };
        std::string odom_topic { "/odin1/odometry" };
        std::string output_frame { "odom" };
        double conf_threshold { 35.0 };
        double voxel_leaf { 0.05 };
        config::RoiBounds roi { .use_roi = true,
            .x_min                       = -11.0,
            .x_max                       = 14.0,
            .y_min                       = -7.5,
            .y_max                       = 7.5,
            .z_min                       = 0.0,
            .z_max                       = 1.4 };
        int bg_num_frames { 10 };
        double diff_threshold { 0.3 };
        config::ClusterConfig cluster { };
        // 地图模式（静态背景）：map_path 非空时启用
        std::string map_path { };
        double initial_tx { 0.0 };
        double initial_ty { 0.0 };
        double initial_tz { 0.0 };
        double initial_roll { 0.0 };
        double initial_pitch { 0.0 };
        double initial_yaw { 0.0 };
    };

    void init();
    void declare_and_load_params();
    void rebuild_stages();
    void rebuild_differencer_and_cluster();
    void rebuild_background();
    void init_map_background();

    void on_scan(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr& msg);

    void publish_dynamic(const types::PointCloud& pts, types::Timestamp stamp);
    void publish_background(const types::PointCloud& pts, types::Timestamp stamp);
    void publish_clusters(
        const std::vector<cluster::ClusterResult>& clusters, types::Timestamp stamp);
    void publish_diag(std::size_t dynamic_count, std::size_t cluster_count, double elapsed_ms,
        types::Timestamp stamp);

    Params params_;
    odin_tune::PoseBuffer pose_buffer_;
    std::optional<odin_tune::BackgroundModel> bg_model_;
    std::optional<odin_tune::FrameDifferencer> differencer_;
    std::optional<odin_tune::MapDifferencer> map_differencer_;
    std::optional<cluster::ClusterStage> cluster_stage_;
    Eigen::Isometry3d t_map_lidar_ { Eigen::Isometry3d::Identity() };
    std::uint64_t frame_count_ { 0 };
    std::uint64_t skipped_no_odom_ { 0 };
    std::uint64_t skipped_warmup_ { 0 };
    std::uint64_t skipped_parse_error_ { 0 };

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_background_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clusters_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cluster_viz_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_;
};

} // namespace radar_lidar::node
