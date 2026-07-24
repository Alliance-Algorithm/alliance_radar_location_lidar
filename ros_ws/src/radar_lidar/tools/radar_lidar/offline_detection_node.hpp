#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "radar_lidar/data_format.hpp"
#include "radar_lidar/offline_registration.hpp"

namespace radar_lidar::map_data {
class MapData;
}
namespace radar_lidar::cluster {
class ClusterStage;
}
namespace radar_lidar::dynamic_cloud {
class DynamicCloudStage;
}

namespace radar_lidar::node {

class OfflineDetectionNode final : public rclcpp::Node {
public:
    OfflineDetectionNode();

private:
    void init();
    void load_params();
    auto load_map_and_publish() -> std::shared_ptr<const map_data::MapData>;
    auto load_scan_and_publish() -> types::Frame;
    auto run_registration(std::shared_ptr<const map_data::MapData> map, const types::Frame& frame)
        -> registration::PoseResult;
    void run_detection_and_clustering(std::shared_ptr<const map_data::MapData> map,
        const Eigen::Isometry3d& t_map_lidar, const types::PointCloud& scan_in_map);
    void publish_diag_summary(
        const registration::RegistrationScore& score, size_t dynamic_count, size_t cluster_count);

    std::string output_frame_ { "map" };

    // Parameters
    std::string map_path_, scan_path_;
    double voxel_leaf_ = 0.1;
    double scan_voxel_ = 0.1;
    registration::OfflinePoseParams reg_params_;
    double dynamic_distance_threshold_ = 0.1;
    int dynamic_num_threads_           = 4;
    double cluster_tolerance_          = 0.25;
    int min_cluster_size_              = 5;
    int max_cluster_size_              = 1000;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_, pub_raw_, pub_aligned_,
        pub_dynamic_, pub_clusters_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_;
};

} // namespace radar_lidar::node
