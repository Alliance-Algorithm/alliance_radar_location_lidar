#include "radar_lidar/offline_detection_node.hpp"

#include <format>
#include <print>
#include <ranges>
#include <stdexcept>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/static_transform_broadcaster.h>

#include "radar_lidar/cluster_stage.hpp"
#include "radar_lidar/dynamic_cloud_stage.hpp"
#include "radar_lidar/geometry_utils.hpp"
#include "radar_lidar/localization_stage.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/offline_visualization.hpp"

namespace radar_lidar::node {

namespace {
    inline constexpr offline::BgrColor kDynamicColorBgr { 0, 0, 255 };

    template <typename PointT>
    auto to_rosmsg(const pcl::PointCloud<PointT>& cloud, const std::string& frame_id)
        -> sensor_msgs::msg::PointCloud2 {
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = frame_id;
        return msg;
    }
} // namespace

OfflineDetectionNode::OfflineDetectionNode()
    : Node("offline_detection_node",
          rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true)) {
    init();
}

void OfflineDetectionNode::init() {
    load_params();
    auto map   = load_map_and_publish();
    auto frame = load_scan_and_publish();

    const auto result = run_registration(map, frame);
    const auto& T     = result.t_map_lidar;

    {
        auto tf_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp            = now();
        tf_msg.header.frame_id         = output_frame_;
        tf_msg.child_frame_id          = "scan";
        tf_msg.transform.translation.x = T.translation().x();
        tf_msg.transform.translation.y = T.translation().y();
        tf_msg.transform.translation.z = T.translation().z();
        const Eigen::Quaterniond q(T.rotation());
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();
        tf_br->sendTransform(tf_msg);
    }

    types::PointCloud scan_in_map;
    scan_in_map.reserve(frame.points.size());
    for (const auto& pt : frame.points) {
        if (offline::is_valid_xyz(pt)) {
            scan_in_map.push_back(T * pt);
        }
    }

    pub_aligned_->publish(to_rosmsg(
        offline::make_colored_cloud(scan_in_map, offline::kAlignedScanColorBgr), output_frame_));
    std::println("[detection] Scan aligned: {} points in map frame", scan_in_map.size());

    run_detection_and_clustering(map, T, scan_in_map);
    std::println("[detection] Done. Spinning for Foxglove.");
}

void OfflineDetectionNode::load_params() {
    get_parameter("output_frame", output_frame_);
    get_parameter("map_path", map_path_);
    get_parameter("scan_path", scan_path_);
    get_parameter("voxel_leaf", voxel_leaf_);
    get_parameter("scan_voxel", scan_voxel_);
    get_parameter("inlier_threshold", reg_params_.inlier_threshold);
    get_parameter("yaw_search_range_deg", reg_params_.yaw_search_range_deg);
    get_parameter("yaw_search_step_deg", reg_params_.yaw_search_step_deg);
    get_parameter("dynamic_distance_threshold", dynamic_distance_threshold_);
    get_parameter("dynamic_num_threads", dynamic_num_threads_);
    get_parameter("cluster_tolerance", cluster_tolerance_);
    get_parameter("min_cluster_size", min_cluster_size_);
    get_parameter("max_cluster_size", max_cluster_size_);
}

auto OfflineDetectionNode::load_map_and_publish() -> std::shared_ptr<const map_data::MapData> {
    if (map_path_.empty()) throw std::runtime_error("map_path required");
    auto result = map_data::MapData::load(map_path_, voxel_leaf_);
    if (!result) throw std::runtime_error("Map load failed: " + result.error());
    auto map = *result;
    std::println("[detection] Map: {} points", map->size());

    const auto qos = rclcpp::QoS(1).transient_local();
    pub_map_       = create_publisher<sensor_msgs::msg::PointCloud2>("/offline_detection/map", qos);
    pub_raw_ = create_publisher<sensor_msgs::msg::PointCloud2>("/offline_detection/scan_raw", qos);
    pub_aligned_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/offline_detection/scan_aligned", qos);
    pub_dynamic_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/offline_detection/dynamic", qos);
    pub_clusters_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/offline_detection/clusters", qos);
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
        "/offline_detection/diagnostics", qos);

    pub_map_->publish(to_rosmsg(
        offline::make_colored_cloud(map->pcl_cloud(), offline::kMapColorBgr), output_frame_));
    return map;
}

auto OfflineDetectionNode::load_scan_and_publish() -> types::Frame {
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
        std::println("[detection] Scan downsampled: {} -> {} points", scan_pcl->size(), ds->size());
        scan_pcl = ds;
    }
    auto frame = geom::filter_valid_points(*scan_pcl);
    std::println("[detection] Scan: {} valid points", frame.points.size());
    if (frame.points.size() < 100) throw std::runtime_error("Too few points in scan");

    pub_raw_->publish(
        to_rosmsg(offline::make_colored_cloud(frame.points, offline::kRawScanColorBgr), "scan"));
    return frame;
}

auto OfflineDetectionNode::run_registration(std::shared_ptr<const map_data::MapData> map,
    const types::Frame& frame) -> registration::PoseResult {
    auto reg_result = run_offline_registration(map, frame, reg_params_);
    if (!reg_result) throw std::runtime_error("Registration failed: " + reg_result.error());
    auto result = std::move(*reg_result);
    for (const auto& detail : result.details)
        std::println("[detection] {}", detail);
    return result;
}

void OfflineDetectionNode::run_detection_and_clustering(
    std::shared_ptr<const map_data::MapData> map, const Eigen::Isometry3d& t_map_lidar,
    const types::PointCloud& scan_in_map) {
    config::DynamicCloudConfig dyn_cfg;
    dyn_cfg.distance_threshold = dynamic_distance_threshold_;
    dyn_cfg.accumulate_frames  = 0;
    dyn_cfg.num_threads        = dynamic_num_threads_;

    dynamic_cloud::DynamicCloudStage dyn_stage(dyn_cfg);
    dyn_stage.set_map(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map->pcl_cloud()));
    auto dyn_res = dyn_stage.process(scan_in_map);
    if (!dyn_res) throw std::runtime_error("Dynamic extraction failed: " + dyn_res.error());

    const auto& dyn_pts = *dyn_res;
    std::println("[detection] Dynamic points: {}", dyn_pts.size());
    if (!dyn_pts.empty())
        pub_dynamic_->publish(
            to_rosmsg(offline::make_colored_cloud(dyn_pts, kDynamicColorBgr), output_frame_));

    config::ClusterConfig cl_cfg;
    cl_cfg.cluster_tolerance = cluster_tolerance_;
    cl_cfg.min_cluster_size  = min_cluster_size_;
    cl_cfg.max_cluster_size  = max_cluster_size_;
    cluster::ClusterStage cl_stage(cl_cfg);
    auto cl_res = cl_stage.process(dyn_pts);
    if (!cl_res) throw std::runtime_error("Clustering failed: " + cl_res.error());

    const auto& clusters = *cl_res;
    std::println("[detection] Clusters: {}", clusters.size());
    if (!clusters.empty()) {
        pcl::PointCloud<pcl::PointXYZ> centroids;
        centroids.reserve(clusters.size());
        for (const auto& c : clusters)
            centroids.emplace_back(static_cast<float>(c.centroid.x()),
                static_cast<float>(c.centroid.y()), static_cast<float>(c.centroid.z()));
        centroids.width    = centroids.size();
        centroids.height   = 1;
        centroids.is_dense = true;
        pub_clusters_->publish(to_rosmsg(centroids, output_frame_));

        for (size_t i = 0; i < clusters.size(); ++i) {
            const auto& c = clusters[i];
            const auto sz = c.max_bound - c.min_bound;
            std::println("[detection]   cluster[{}]: centroid=({:.2f},{:.2f},{:.2f}) "
                         "pts={} size=({:.2f},{:.2f},{:.2f})",
                i, c.centroid.x(), c.centroid.y(), c.centroid.z(), c.point_count, sz.x(), sz.y(),
                sz.z());
        }
    }
    publish_diag_summary(registration::RegistrationScore { }, dyn_pts.size(), clusters.size());
}

void OfflineDetectionNode::publish_diag_summary(
    const registration::RegistrationScore& score, size_t dynamic_count, size_t cluster_count) {
    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name    = "offline_detection";
    diag.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag.message = std::format("dynamic={} clusters={} inlier={:.3f} rmse={:.4f}", dynamic_count,
        cluster_count, score.inlier_ratio, score.rmse);
    pub_diag_->publish(diag);
}

} // namespace radar_lidar::node
