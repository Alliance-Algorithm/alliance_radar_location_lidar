#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "radar_fusion/kalman_tracker.hpp"

namespace radar::fusion {

struct FusionConfig {
    double gate_distance     = 1.0; // 数据关联门限（米）
    double track_timeout_sec = 1.5; // 轨迹超时删除
    int min_hits_to_confirm  = 3;   // 确认轨迹所需命中次数
    int max_tracks           = 20;  // 最大轨迹数
};

class FusionNode : public rclcpp::Node {
public:
    FusionNode();

private:
    void on_lidar_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void on_cluster(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void publish_tracks(const std::vector<KalmanTracker>& tracks, const rclcpp::Time& stamp);

    FusionConfig cfg_;
    std::vector<KalmanTracker> tracks_;
    int next_track_id_ = 0;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_lidar_pose_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cluster_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_tracks_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
};

} // namespace radar::fusion
