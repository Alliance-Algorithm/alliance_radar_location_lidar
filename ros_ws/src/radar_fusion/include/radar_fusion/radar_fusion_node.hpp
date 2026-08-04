#pragma once

#include "radar_interfaces/msg/camera_detection_pose.hpp"
#include "radar_interfaces/msg/lidar_location.hpp"
#include <chrono>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "radar_fusion/data_format.hpp"
#include "radar_fusion/default_positions.hpp"
#include "radar_fusion/default_slots.hpp"
#include "radar_fusion/kalman_tracker.hpp"
#include "radar_fusion/match_timer.hpp"
#include "radar_interfaces/msg/game_state.hpp"

namespace radar_fusion::node {

class RadarFusionNode final : public rclcpp::Node {
public:
    explicit RadarFusionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions { });

private:
    void on_lidar_pose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void on_camera_detection(radar_interfaces::msg::CameraDetectionPose::SharedPtr msg);
    void on_game_state(radar_interfaces::msg::GameState::SharedPtr msg);

    void on_cluster(sensor_msgs::msg::PointCloud2::SharedPtr msg);

    // 独立 lidar 聚类 track 池（与 camera 池解耦，脏数据互不干扰）
    void process_lidar_clusters(const std::vector<Eigen::Vector2d>& measurements, int64_t now_ns);

    void publish_tracks(const std::vector<radar_fusion::kalman_tracker::KalmanTracker>& tracks,
        const rclcpp::Time& stamp);
    void publish_fused_tracks(
        const std::vector<radar_fusion::kalman_tracker::KalmanTracker>& tracks,
        const rclcpp::Time& stamp);
    void publish_lidar_tracks(
        const std::vector<radar_fusion::kalman_tracker::KalmanTracker>& tracks,
        const rclcpp::Time& stamp);
    void publish_lidar_location(
        const std::vector<radar_fusion::kalman_tracker::KalmanTracker>& tracks,
        const std::vector<radar_fusion::kalman_tracker::KalmanTracker>& lidar_tracks);
    void fill_default_positions(radar_interfaces::msg::LidarLocation& msg, int64_t now_ns);
    void publish_localization_pose(const geometry_msgs::msg::PoseWithCovarianceStamped& pose);
    void publish_status(const rclcpp::Time& stamp) const;
    void update_fusion_mode(int64_t reference_stamp_ns);

    static int64_t steady_now_ns();

    void process_measurements(const std::vector<Eigen::Vector2d>& measurements, int64_t now_ns,
        bool mark_unmatched_tracks, const std::vector<int>& classes = { });

    radar_fusion::fusion_config::FusionConfig cfg_;
    radar_fusion::fusion_config::FusionMode fusion_mode_ =
        radar_fusion::fusion_config::FusionMode::RADAR_ONLY;
    radar_fusion::match_timer::MatchTimer match_timer_;
    rclcpp::TimerBase::SharedPtr location_timer_; // 10Hz 统一输出
    std::string default_positions_path_;
    std::string enemy_color_  = "blue";
    std::string camera_topic_ = "/radar_camera/robot_pose";
    std::vector<radar_fusion::kalman_tracker::KalmanTracker> tracks_;
    std::vector<radar_fusion::kalman_tracker::KalmanTracker> lidar_tracks_;
    std::vector<radar_fusion::camera_observation::CameraObservation> latest_camera_observations_;
    int64_t latest_camera_stamp_ns_ = 0;
    int next_track_id_              = 0;
    int next_lidar_track_id_        = 0;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_lidar_pose_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cluster_;
    rclcpp::Subscription<radar_interfaces::msg::CameraDetectionPose>::SharedPtr
        sub_camera_detection_;
    rclcpp::Subscription<radar_interfaces::msg::GameState>::SharedPtr sub_game_state_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_tracks_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_fused_tracks_;
    rclcpp::Publisher<radar_interfaces::msg::LidarLocation>::SharedPtr pub_lidar_location_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_status_;
};

} // namespace radar_fusion::node
