#pragma once

#include <memory>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <radar_interfaces/msg/registration_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "radar_lidar/cluster_stage.hpp"
#include "radar_lidar/data_format.hpp"
#include "radar_lidar/dynamic_cloud_stage.hpp"
#include "radar_lidar/localization_stage.hpp"
#include "radar_lidar/map_data.hpp"

namespace radar_lidar::node {

class RadarLidarNode final : public rclcpp::Node {
public:
    RadarLidarNode();
    explicit RadarLidarNode(const rclcpp::NodeOptions& options);
    [[nodiscard]] auto failure_requested() const noexcept -> bool { return failure_requested_; }

private:
    void on_scan(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void publish_pose(const types::PoseEstimate& lidar_pose,
        const Eigen::Isometry3d& t_map_radar_base, types::Timestamp stamp);
    void publish_static_tf(const Eigen::Isometry3d& t_map_radar_base, types::Timestamp stamp);
    void publish_registration_status();
    void on_registration_timeout();
    void publish_diagnostics(const types::PoseEstimate& pose, double elapsed_ms, uint64_t frame);
    void publish_dynamic(const types::PointCloud& dynamic_points, types::Timestamp stamp);
    void publish_clusters(
        const std::vector<cluster::ClusterResult>& clusters, types::Timestamp stamp);
    auto radar_base_pose(const Eigen::Isometry3d& t_map_lidar) -> std::optional<Eigen::Isometry3d>;

    void transform_scan_to_map(const types::PointCloud& scan, const types::PoseEstimate& pose,
        types::PointCloud& transformed);

    /// @brief Read an optional Odin map -> scan estimate without replacing required GICP.
    auto try_odin_relocalization_pose(const std::string& source_frame, const rclcpp::Time& stamp)
        -> std::optional<types::PoseEstimate>;

    std::shared_ptr<const map_data::MapData> map_;
    localization::LocalizationStage localization_;
    dynamic_cloud::DynamicCloudStage dynamic_stage_;
    cluster::ClusterStage cluster_stage_;

    std::string scan_topic_   = "/livox/lidar";
    std::string hardware_id_  = "livox_mid70";
    std::string output_frame_ = "map";
    std::string lidar_frame_  = "lidar_link";
    bool detection_enabled_   = true;
    double registration_timeout_sec_ = 30.0;

    // Odin1 relocalization is an optional estimate; only accepted GICP can lock registration.
    bool use_odin_relocalization_tf_ = false;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clusters_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cluster_viz_;
    rclcpp::Publisher<radar_interfaces::msg::RegistrationStatus>::SharedPtr
        pub_registration_status_;
    rclcpp::TimerBase::SharedPtr registration_timeout_timer_;
    rclcpp::TimerBase::SharedPtr failure_shutdown_timer_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    uint64_t frame_count_ { 0 };
    bool static_tf_published_ { false };
    bool was_odin_relocalized_ { false };
    bool failure_requested_ { false };
    std::string last_rejection_reason_ { "No usable scan received" };
    std::string registration_failure_reason_;
    std::optional<Eigen::Isometry3d> t_radar_base_lidar_;
};

} // namespace radar_lidar::node
