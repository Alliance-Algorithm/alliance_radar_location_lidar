#pragma once

#include <memory>
#include <string>

#include <Eigen/Geometry>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "radar_lidar/data_format.hpp"
#include "radar_lidar/offline_registration.hpp"

namespace radar_lidar::map_data {
class MapData;
}
namespace radar_lidar::localization {
class LocalizationStage;
}

namespace radar_lidar::node {

class OfflineTestNode final : public rclcpp::Node {
public:
    OfflineTestNode();

private:
    void init();
    void load_params();
    auto load_map_and_publish() -> std::shared_ptr<const map_data::MapData>;
    auto load_scan_and_publish() -> types::Frame;
    void apply_sector_roi(types::Frame& frame);
    auto run_registration(std::shared_ptr<const map_data::MapData> map, const types::Frame& frame)
        -> registration::PoseResult;
    void publish_results(const registration::PoseResult& result,
        std::shared_ptr<const map_data::MapData> map, const types::PointCloud& scan_in_map);
    void write_pose_json(const std::string& path, const Eigen::Isometry3d& T,
        const registration::RegistrationScore& score, bool converged);

    std::string output_frame_ { "map" };
    std::string scan_frame_ { "scan" };

    // Parameters
    std::string map_path_, scan_path_, pose_out_;
    double voxel_leaf_ = 0.1, scan_voxel_ = 0.1;
    bool map_y_up_ = false;
    registration::OfflinePoseParams reg_params_;

    // Sector ROI
    bool use_roi_            = false;
    double roi_half_fov_deg_ = 60.0, roi_min_range_ = 1.0, roi_max_range_ = 30.0;
    double roi_z_min_ = 0.0, roi_z_max_ = 7.0;
    double roi_origin_x_ = 0.0, roi_origin_y_ = 0.0, roi_origin_yaw_deg_ = 0.0;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_, pub_raw_, pub_roi_,
        pub_aligned_, pub_overlay_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_;
};

} // namespace radar_lidar::node
