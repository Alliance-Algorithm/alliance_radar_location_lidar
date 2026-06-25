#pragma once

#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "radar_lidar/cluster.hpp"
#include "radar_lidar/config.hpp"
#include "radar_lidar/dynamic_cloud.hpp"
#include "radar_lidar/localization.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/types.hpp"

namespace radar {

class LidarPipeline : public rclcpp::Node {
public:
    LidarPipeline();

private:
    void on_scan(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void publish_pose(const types::PoseEstimate& pose, types::Timestamp stamp);
    void publish_diagnostics(const types::PoseEstimate& pose, double elapsed_ms, uint64_t frame);
    void publish_dynamic(const types::PointCloud& dynamic_points, types::Timestamp stamp);
    void publish_clusters(const std::vector<ClusterResult>& clusters, types::Timestamp stamp);

    void transform_scan_to_map(const types::PointCloud& scan, const types::PoseEstimate& pose,
        types::PointCloud& transformed);

    std::shared_ptr<const MapData> map_;
    LocalizationStage localization_;
    DynamicCloudStage dynamic_stage_;
    ClusterStage cluster_stage_;
    bool detection_enabled_ = true;

    std::string scan_topic_;
    std::string hardware_id_;
    std::string output_frame_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clusters_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cluster_viz_;

    uint64_t frame_count_ { 0 };
};

} // namespace radar
