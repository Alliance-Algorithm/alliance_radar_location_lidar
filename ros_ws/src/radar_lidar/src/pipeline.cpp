#include "radar_lidar/pipeline.hpp"

#include <cfloat>
#include <chrono>
#include <ranges>

#include <pcl_conversions/pcl_conversions.h>

namespace radar {

LidarPipeline::LidarPipeline()
    : Node("radar_lidar_node",
          rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true))
    , map_(nullptr)
    , localization_(nullptr, { })
    , dynamic_stage_({ })
    , cluster_stage_({ }) {

    // ── node params ────────────────────────────────────────────────
    get_parameter("scan_topic", scan_topic_);
    get_parameter("hardware_id", hardware_id_);
    get_parameter("output_frame", output_frame_);
    get_parameter("detection_enabled", detection_enabled_);

    // ── map ────────────────────────────────────────────────────────
    std::string map_path;
    get_parameter("map_path", map_path);
    if (map_path.empty()) {
        RCLCPP_FATAL(get_logger(), "map_path parameter is required!");
        return;
    }
    auto map_result = MapData::Load(map_path, 0.1);
    if (!map_result) {
        RCLCPP_FATAL(get_logger(), "Failed to load map: %s", map_result.error().c_str());
        return;
    }
    map_ = *map_result;
    RCLCPP_INFO(get_logger(), "Map loaded: %zu points", map_->size());

    // ── GICP config ────────────────────────────────────────────────
    config::LocalizationConfig loc_cfg;
    get_parameter("gicp_num_threads", loc_cfg.num_threads);
    get_parameter("gicp_max_corr_distance", loc_cfg.max_corr_distance);
    get_parameter("gicp_max_iterations", loc_cfg.max_iterations);
    get_parameter("gicp_use_spherical_grid", loc_cfg.use_spherical_grid);
    get_parameter("gicp_spherical_grid_deg", loc_cfg.spherical_grid_deg);
    get_parameter("gicp_accumulate_frames", loc_cfg.accumulate_frames);
    get_parameter("gicp_use_lock_strategy", loc_cfg.use_lock_strategy);
    get_parameter("gicp_lock_fitness", loc_cfg.lock_fitness);
    get_parameter("gicp_use_roi", loc_cfg.use_roi);
    get_parameter("gicp_roi_x_min", loc_cfg.roi_x_min);
    get_parameter("gicp_roi_x_max", loc_cfg.roi_x_max);
    get_parameter("gicp_roi_y_min", loc_cfg.roi_y_min);
    get_parameter("gicp_roi_y_max", loc_cfg.roi_y_max);
    get_parameter("gicp_roi_z_min", loc_cfg.roi_z_min);
    get_parameter("gicp_roi_z_max", loc_cfg.roi_z_max);
    get_parameter("gicp_voxel_leaf_size", loc_cfg.voxel_leaf_size);
    get_parameter("gicp_rotation_eps", loc_cfg.rotation_eps);
    get_parameter("gicp_translation_eps", loc_cfg.translation_eps);
    get_parameter("gicp_verbose", loc_cfg.verbose);

    localization_ = LocalizationStage(map_, loc_cfg);

    // ── detection config ───────────────────────────────────────────
    DynamicCloudConfig dyn_cfg;
    get_parameter("dynamic_distance_threshold", dyn_cfg.distance_threshold);
    get_parameter("dynamic_num_threads", dyn_cfg.num_threads);
    get_parameter("dynamic_accumulate_frames", dyn_cfg.accumulate_frames);
    get_parameter("dynamic_use_roi", dyn_cfg.use_roi);
    get_parameter("dynamic_roi_x_min", dyn_cfg.roi_x_min);
    get_parameter("dynamic_roi_x_max", dyn_cfg.roi_x_max);
    get_parameter("dynamic_roi_y_min", dyn_cfg.roi_y_min);
    get_parameter("dynamic_roi_y_max", dyn_cfg.roi_y_max);
    get_parameter("dynamic_roi_z_min", dyn_cfg.roi_z_min);
    get_parameter("dynamic_roi_z_max", dyn_cfg.roi_z_max);

    dynamic_stage_ = DynamicCloudStage(dyn_cfg);
    dynamic_stage_.set_map(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_->pcl_cloud()));

    // ── cluster config ─────────────────────────────────────────────
    ClusterConfig cl_cfg;
    get_parameter("cluster_tolerance", cl_cfg.cluster_tolerance);
    get_parameter("cluster_min_size", cl_cfg.min_cluster_size);
    get_parameter("cluster_max_size", cl_cfg.max_cluster_size);
    cluster_stage_ = ClusterStage(cl_cfg);

    // ── subscription ───────────────────────────────────────────────
    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(scan_topic_,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_scan(msg); });

    pub_pose_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/lidar/pose", 10);
    pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/diagnostics", 10);
    pub_dynamic_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/dynamic", 10);
    pub_clusters_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/cluster", 10);
    pub_cluster_viz_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar/cluster_viz", 10);

    RCLCPP_INFO(get_logger(), "radar_lidar ready. Listening on %s (detection=%s)",
        scan_topic_.c_str(), detection_enabled_ ? "ON" : "OFF");
}

void LidarPipeline::on_scan(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    ++frame_count_;
    const auto t0 = std::chrono::steady_clock::now();

    types::Frame frame;
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);
        frame.points = cloud->points | std::views::filter([](const auto& pt) {
            return std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)
                && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > DBL_EPSILON;
        }) | std::views::transform([](const auto& pt) { return Eigen::Vector3d(pt.x, pt.y, pt.z); })
            | std::ranges::to<types::PointCloud>();
        frame.stamp    = rclcpp::Time(msg->header.stamp).nanoseconds();
        frame.frame_id = msg->header.frame_id;
    }

    if (frame.points.size() < 100) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Too few points: %zu", frame.points.size());
        return;
    }

    auto pose = localization_.process(frame);
    if (!pose) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Localization failed: %s", pose.error().c_str());
        const auto t1           = std::chrono::steady_clock::now();
        const double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        publish_diagnostics(pose.value_or(types::PoseEstimate { }), elapsed_ms, frame_count_);
        return;
    }

    publish_pose(*pose, frame.stamp);

    // 检测：把扫描变换到地图坐标系后提取动态点
    if (detection_enabled_) {
        types::PointCloud scan_in_map;
        transform_scan_to_map(frame.points, *pose, scan_in_map);

        auto dynamic_result = dynamic_stage_.process(scan_in_map);
        if (dynamic_result && !dynamic_result->empty()) {
            publish_dynamic(*dynamic_result, frame.stamp);

            auto cluster_result = cluster_stage_.process(*dynamic_result);
            if (cluster_result && !cluster_result->empty()) {
                publish_clusters(*cluster_result, frame.stamp);
            }
        }
    }

    const auto t1           = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    publish_diagnostics(*pose, elapsed_ms, frame_count_);
}

void LidarPipeline::transform_scan_to_map(const types::PointCloud& scan,
    const types::PoseEstimate& pose, types::PointCloud& transformed) {
    transformed = scan | std::views::transform([&pose](const auto& p) { return pose.T * p; })
        | std::ranges::to<types::PointCloud>();
}

void LidarPipeline::publish_pose(const types::PoseEstimate& pose, types::Timestamp stamp) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp    = rclcpp::Time(stamp);
    msg.header.frame_id = output_frame_;

    const auto& T            = pose.T;
    msg.pose.pose.position.x = T.translation().x();
    msg.pose.pose.position.y = T.translation().y();
    msg.pose.pose.position.z = T.translation().z();
    Eigen::Quaterniond q(T.rotation());
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(msg.pose.covariance.data()) =
        pose.covariance;

    pub_pose_->publish(msg);
}

void LidarPipeline::publish_diagnostics(
    const types::PoseEstimate& pose, double elapsed_ms, uint64_t frame) {
    auto diag        = diagnostic_msgs::msg::DiagnosticStatus();
    diag.level       = pose.converged ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                      : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diag.name        = "radar_lidar/localization";
    diag.hardware_id = hardware_id_;
    diag.message     = pose.converged ? "TRACKING" : "INIT";

    auto add_kv = [&](const char* k, const std::string& v) {
        auto kv  = diagnostic_msgs::msg::KeyValue();
        kv.key   = k;
        kv.value = v;
        diag.values.push_back(kv);
    };
    add_kv("fitness", std::to_string(pose.fitness_score));
    add_kv("time_ms", std::to_string(elapsed_ms));
    add_kv("frame", std::to_string(frame));
    add_kv("converged", pose.converged ? "true" : "false");

    pub_diag_->publish(diag);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Frame %lu | score=%.4f | time=%.1fms | %s", frame, pose.fitness_score, elapsed_ms,
        diag.message.c_str());
}

void LidarPipeline::publish_dynamic(
    const types::PointCloud& dynamic_points, types::Timestamp stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(dynamic_points.size());
    for (const auto& p : dynamic_points) {
        cloud->emplace_back(
            static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    }
    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp    = rclcpp::Time(stamp);
    msg.header.frame_id = output_frame_;
    pub_dynamic_->publish(msg);
}

void LidarPipeline::publish_clusters(
    const std::vector<ClusterResult>& clusters, types::Timestamp stamp) {
    // Centroid PointCloud2 for downstream consumption (fusion_node)
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroids(new pcl::PointCloud<pcl::PointXYZ>());
    centroids->reserve(clusters.size());
    for (const auto& c : clusters) {
        centroids->emplace_back(static_cast<float>(c.centroid.x()),
            static_cast<float>(c.centroid.y()), static_cast<float>(c.centroid.z()));
    }
    centroids->width    = centroids->size();
    centroids->height   = 1;
    centroids->is_dense = true;

    sensor_msgs::msg::PointCloud2 centroid_msg;
    pcl::toROSMsg(*centroids, centroid_msg);
    centroid_msg.header.stamp    = rclcpp::Time(stamp);
    centroid_msg.header.frame_id = output_frame_;
    pub_clusters_->publish(centroid_msg);

    // MarkerArray visualization (AABB boxes + centroid spheres)
    visualization_msgs::msg::MarkerArray markers;

    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& c = clusters[i];

        visualization_msgs::msg::Marker box;
        box.header.stamp    = rclcpp::Time(stamp);
        box.header.frame_id = output_frame_;
        box.ns              = "clusters";
        box.id              = static_cast<int>(i);
        box.type            = visualization_msgs::msg::Marker::CUBE;
        box.action          = visualization_msgs::msg::Marker::ADD;

        box.pose.position.x    = (c.min_bound.x() + c.max_bound.x()) / 2.0;
        box.pose.position.y    = (c.min_bound.y() + c.max_bound.y()) / 2.0;
        box.pose.position.z    = (c.min_bound.z() + c.max_bound.z()) / 2.0;
        box.pose.orientation.w = 1.0;

        box.scale.x = c.max_bound.x() - c.min_bound.x();
        box.scale.y = c.max_bound.y() - c.min_bound.y();
        box.scale.z = c.max_bound.z() - c.min_bound.z();

        if (box.scale.x < 0.01) box.scale.x = 0.01;
        if (box.scale.y < 0.01) box.scale.y = 0.01;
        if (box.scale.z < 0.01) box.scale.z = 0.01;

        box.color.r  = 0.0f;
        box.color.g  = 1.0f;
        box.color.b  = 0.0f;
        box.color.a  = 0.3f;
        box.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(box);

        visualization_msgs::msg::Marker centroid;
        centroid.header.stamp    = rclcpp::Time(stamp);
        centroid.header.frame_id = output_frame_;
        centroid.ns              = "centroids";
        centroid.id              = static_cast<int>(i);
        centroid.type            = visualization_msgs::msg::Marker::SPHERE;
        centroid.action          = visualization_msgs::msg::Marker::ADD;

        centroid.pose.position.x    = c.centroid.x();
        centroid.pose.position.y    = c.centroid.y();
        centroid.pose.position.z    = c.centroid.z();
        centroid.pose.orientation.w = 1.0;

        centroid.scale.x = 0.15;
        centroid.scale.y = 0.15;
        centroid.scale.z = 0.15;

        centroid.color.r  = 1.0f;
        centroid.color.g  = 0.0f;
        centroid.color.b  = 0.0f;
        centroid.color.a  = 1.0f;
        centroid.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(centroid);
    }

    pub_cluster_viz_->publish(markers);
}

} // namespace radar
