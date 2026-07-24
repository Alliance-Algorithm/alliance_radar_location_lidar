#include "radar_lidar/offline_test_node.hpp"

#include <cmath>
#include <format>
#include <fstream>
#include <print>
#include <ranges>
#include <stdexcept>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "radar_lidar/geometry_utils.hpp"
#include "radar_lidar/localization_stage.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/offline_visualization.hpp"

namespace radar_lidar::node {

namespace {
    template <typename PointT>
    auto to_rosmsg(const pcl::PointCloud<PointT>& cloud, const std::string& frame_id)
        -> sensor_msgs::msg::PointCloud2 {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = frame_id;
        return msg;
    }
} // namespace

OfflineTestNode::OfflineTestNode()
    : Node("offline_test_node",
          rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true)) {
    init();
}

void OfflineTestNode::init() {
    load_params();
    auto map   = load_map_and_publish();
    auto frame = load_scan_and_publish();
    apply_sector_roi(frame);

    const auto result    = run_registration(map, frame);
    const auto& T        = result.t_map_lidar;
    const auto& final_sc = result.score;
    const bool converged = result.converged;

    // Transform scan to map, publish aligned and overlay
    types::PointCloud scan_in_map;
    scan_in_map.reserve(frame.points.size());
    for (const auto& pt : frame.points) {
        if (offline::is_valid_xyz(pt)) {
            scan_in_map.push_back(T * pt);
        }
    }

    pub_aligned_->publish(to_rosmsg(
        offline::make_colored_cloud(scan_in_map, offline::kAlignedScanColorBgr), output_frame_));
    pub_overlay_->publish(to_rosmsg(offline::make_overlay_cloud(map->pcl_cloud(), scan_in_map,
                                        offline::kMapColorBgr, offline::kAlignedScanColorBgr),
        output_frame_));

    // Pose
    const auto trans = T.translation();
    const Eigen::Quaterniond q(T.rotation());
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp            = now();
    pose_msg.header.frame_id         = output_frame_;
    pose_msg.pose.pose.position.x    = trans.x();
    pose_msg.pose.pose.position.y    = trans.y();
    pose_msg.pose.pose.position.z    = trans.z();
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();
    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(pose_msg.pose.covariance.data()) =
        result.covariance;
    pub_pose_->publish(pose_msg);

    // Diagnostics
    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name    = "offline_registration";
    diag.level   = converged ? diagnostic_msgs::msg::DiagnosticStatus::OK
                             : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag.message = std::format(
        "inlier={:.3f} rmse={:.4f} converged={}", final_sc.inlier_ratio, final_sc.rmse, converged);
    pub_diag_->publish(diag);

    if (!pose_out_.empty()) write_pose_json(pose_out_, T, final_sc, converged);

    std::println("[offline] Result: inlier={:.3f} rmse={:.4f} pos=({:.3f},{:.3f},{:.3f})",
        final_sc.inlier_ratio, final_sc.rmse, trans.x(), trans.y(), trans.z());
    std::println("[offline] All topics published. Spinning.");
}

void OfflineTestNode::load_params() {
    get_parameter("output_frame", output_frame_);
    get_parameter("scan_frame", scan_frame_);
    get_parameter("map_path", map_path_);
    get_parameter("scan_path", scan_path_);
    get_parameter("pose_out", pose_out_);
    get_parameter("voxel_leaf", voxel_leaf_);
    get_parameter("scan_voxel", scan_voxel_);
    get_parameter("map_y_up", map_y_up_);
    get_parameter("inlier_threshold", reg_params_.inlier_threshold);
    get_parameter("yaw_search_range_deg", reg_params_.yaw_search_range_deg);
    get_parameter("yaw_search_step_deg", reg_params_.yaw_search_step_deg);
    get_parameter("use_roi", use_roi_);
    get_parameter("roi_half_fov_deg", roi_half_fov_deg_);
    get_parameter("roi_min_range", roi_min_range_);
    get_parameter("roi_max_range", roi_max_range_);
    get_parameter("roi_z_min", roi_z_min_);
    get_parameter("roi_z_max", roi_z_max_);
    get_parameter("roi_origin_x", roi_origin_x_);
    get_parameter("roi_origin_y", roi_origin_y_);
    get_parameter("roi_origin_yaw_deg", roi_origin_yaw_deg_);
}

auto OfflineTestNode::load_map_and_publish() -> std::shared_ptr<const map_data::MapData> {
    if (map_path_.empty()) throw std::runtime_error("map_path required");

    if (map_y_up_) {
        std::println(
            stderr, "[offline] WARNING: map_y_up=true deprecated, use model_to_map instead");
        auto raw = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *raw) == -1)
            throw std::runtime_error("Failed to load map PCD for rotation");
        for (auto& pt : raw->points) {
            const float y = pt.y, z = pt.z;
            pt.y = -z;
            pt.z = y;
        }
        map_path_ = "/tmp/map_z_up.pcd";
        pcl::io::savePCDFileBinary(map_path_, *raw);
        std::println("[offline] Map rotated Y-up→Z-up, saved to {}", map_path_);
    }

    auto result = map_data::MapData::load(map_path_, voxel_leaf_);
    if (!result) throw std::runtime_error("Map load failed: " + result.error());
    auto map = *result;
    std::println("[offline] Map: {} points", map->size());

    const auto qos = rclcpp::QoS(1).transient_local();
    pub_map_       = create_publisher<sensor_msgs::msg::PointCloud2>("/offline/map", qos);
    pub_raw_       = create_publisher<sensor_msgs::msg::PointCloud2>("/offline/scan_raw", qos);
    pub_roi_       = create_publisher<sensor_msgs::msg::PointCloud2>("/offline/scan_roi", qos);
    pub_aligned_   = create_publisher<sensor_msgs::msg::PointCloud2>("/offline/scan_aligned", qos);
    pub_overlay_   = create_publisher<sensor_msgs::msg::PointCloud2>("/offline/overlay", qos);
    pub_pose_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/offline/pose", qos);
    pub_diag_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/offline/diagnostics", qos);

    pub_map_->publish(to_rosmsg(
        offline::make_colored_cloud(map->pcl_cloud(), offline::kMapColorBgr), output_frame_));
    return map;
}

auto OfflineTestNode::load_scan_and_publish() -> types::Frame {
    if (scan_path_.empty()) throw std::runtime_error("scan_path required");
    auto scan_pcl = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_path_, *scan_pcl) == -1)
        throw std::runtime_error("Failed to load scan: " + scan_path_);

    if (scan_voxel_ > 0.0) {
        auto ds = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(scan_voxel_, scan_voxel_, scan_voxel_);
        vg.setInputCloud(scan_pcl);
        vg.filter(*ds);
        std::println("[offline] Scan downsampled: {} -> {} points", scan_pcl->size(), ds->size());
        scan_pcl = ds;
    }
    auto frame = geom::filter_valid_points(*scan_pcl);
    std::println("[offline] Scan: {} valid points", frame.points.size());
    if (frame.points.size() < 100) throw std::runtime_error("Too few points in scan");

    pub_raw_->publish(to_rosmsg(
        offline::make_colored_cloud(frame.points, offline::kRawScanColorBgr), scan_frame_));
    return frame;
}

void OfflineTestNode::apply_sector_roi(types::Frame& frame) {
    if (!use_roi_) return;

    const double yaw_rad   = registration::deg_to_rad(roi_origin_yaw_deg_);
    const double cos_yaw   = std::cos(yaw_rad);
    const double sin_yaw   = std::sin(yaw_rad);
    const double half_fov  = registration::deg_to_rad(roi_half_fov_deg_);
    const double tan2_half = std::tan(half_fov) * std::tan(half_fov);
    const double min_r2    = roi_min_range_ * roi_min_range_;
    const double max_r2    = roi_max_range_ * roi_max_range_;

    types::PointCloud clipped;
    clipped.reserve(frame.points.size());
    for (const auto& p : frame.points) {
        if (p.z() < roi_z_min_ || p.z() > roi_z_max_) continue;
        const double dx = p.x() - roi_origin_x_;
        const double dy = p.y() - roi_origin_y_;
        const double r2 = dx * dx + dy * dy;
        if (r2 < min_r2 || r2 > max_r2) continue;
        const double fx = dx * cos_yaw + dy * sin_yaw;
        const double fy = -dx * sin_yaw + dy * cos_yaw;
        if (fx <= 0.0) continue;
        if (fy * fy > fx * fx * tan2_half) continue;
        clipped.push_back(p);
    }
    std::println("[offline] ROI sector: {} -> {} points (origin=({:.1f},{:.1f}) yaw={}° FOV=±{}°)",
        frame.points.size(), clipped.size(), roi_origin_x_, roi_origin_y_, roi_origin_yaw_deg_,
        roi_half_fov_deg_);

    if (clipped.size() < 100) throw std::runtime_error("Too few points after ROI");

    pub_roi_->publish(
        to_rosmsg(offline::make_colored_cloud(clipped, offline::kRoiColorBgr), scan_frame_));
    frame.points = std::move(clipped);
}

auto OfflineTestNode::run_registration(std::shared_ptr<const map_data::MapData> map,
    const types::Frame& frame) -> registration::PoseResult {
    auto reg_result = run_offline_registration(map, frame, reg_params_);
    if (!reg_result) throw std::runtime_error("Registration failed: " + reg_result.error());
    auto result = std::move(*reg_result);
    for (const auto& detail : result.details)
        std::println("[offline] {}", detail);
    return result;
}

void OfflineTestNode::write_pose_json(const std::string& path, const Eigen::Isometry3d& T,
    const registration::RegistrationScore& score, bool converged) {
    std::ofstream ofs(path);
    if (!ofs) {
        std::println(stderr, "[offline] ERROR: cannot open {}", path);
        return;
    }
    const Eigen::Vector3d t = T.translation();
    const Eigen::Quaterniond q(T.rotation());
    const Eigen::Matrix4d m = T.matrix();
    ofs << std::format("{{\n");
    ofs << std::format("  \"frame\": \"work_center_origin_zup_meters\",\n");
    ofs << std::format("  \"direction\": \"T_map_lidar (lidar point -> map)\",\n");
    ofs << std::format("  \"translation\": [{:.6f}, {:.6f}, {:.6f}],\n", t.x(), t.y(), t.z());
    ofs << std::format(
        "  \"quaternion_xyzw\": [{:.6f}, {:.6f}, {:.6f}, {:.6f}],\n", q.x(), q.y(), q.z(), q.w());
    ofs << std::format("  \"matrix\": [\n");
    for (int r = 0; r < 4; ++r)
        ofs << std::format("    [{:.6f}, {:.6f}, {:.6f}, {:.6f}]{}\n", m(r, 0), m(r, 1), m(r, 2),
            m(r, 3), r < 3 ? "," : "");
    ofs << std::format("  ],\n");
    ofs << std::format("  \"inlier_ratio\": {:.6f},\n", score.inlier_ratio);
    ofs << std::format("  \"rmse\": {:.6f},\n", score.rmse);
    ofs << std::format("  \"converged\": {}\n", converged);
    ofs << std::format("}}\n");
    std::println("[offline] Pose written: {}", path);
}

} // namespace radar_lidar::node
