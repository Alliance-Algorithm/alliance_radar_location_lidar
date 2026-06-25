#include "radar_lidar/pipeline.hpp"

#include <cfloat>
#include <chrono>

#include <pcl_conversions/pcl_conversions.h>

namespace radar {

LidarPipeline::LidarPipeline()
    : Node("radar_lidar_node")
    , map_(nullptr)
    , localization_(nullptr, { })
    , dynamic_stage_({ })
    , cluster_stage_({ }) {

    // ── node params ────────────────────────────────────────────────
    this->declare_parameter("map_path", "");
    this->declare_parameter("scan_topic", "/livox/lidar");
    this->declare_parameter("hardware_id", "livox_mid70");
    this->declare_parameter("output_frame", "map");
    this->declare_parameter("detection_enabled", true);

    scan_topic_        = this->get_parameter("scan_topic").as_string();
    hardware_id_       = this->get_parameter("hardware_id").as_string();
    output_frame_      = this->get_parameter("output_frame").as_string();
    detection_enabled_ = this->get_parameter("detection_enabled").as_bool();

    // ── map ────────────────────────────────────────────────────────
    const auto map_path = this->get_parameter("map_path").as_string();
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
    this->declare_parameter("gicp.num_threads", 4);
    this->declare_parameter("gicp.max_corr_distance", 1.0);
    this->declare_parameter("gicp.max_iterations", 48);

    // 预处理参数
    this->declare_parameter("gicp.use_spherical_grid", true);
    this->declare_parameter("gicp.spherical_grid_deg", 0.1);
    this->declare_parameter("gicp.accumulate_frames", 20);
    this->declare_parameter("gicp.use_lock_strategy", false);
    this->declare_parameter("gicp.lock_fitness", 0.2);

    // ROI
    this->declare_parameter("gicp.use_roi", false);
    this->declare_parameter("gicp.roi_x_min", 0.0);
    this->declare_parameter("gicp.roi_x_max", 30.0);
    this->declare_parameter("gicp.roi_y_min", -15.0);
    this->declare_parameter("gicp.roi_y_max", 15.0);
    this->declare_parameter("gicp.roi_z_min", 0.0);
    this->declare_parameter("gicp.roi_z_max", 7.0);

    config::LocalizationConfig loc_cfg;
    loc_cfg.num_threads        = this->get_parameter("gicp.num_threads").as_int();
    loc_cfg.max_corr_distance  = this->get_parameter("gicp.max_corr_distance").as_double();
    loc_cfg.max_iterations     = this->get_parameter("gicp.max_iterations").as_int();
    loc_cfg.use_spherical_grid = this->get_parameter("gicp.use_spherical_grid").as_bool();
    loc_cfg.spherical_grid_deg = this->get_parameter("gicp.spherical_grid_deg").as_double();
    loc_cfg.accumulate_frames  = this->get_parameter("gicp.accumulate_frames").as_int();
    loc_cfg.use_lock_strategy  = this->get_parameter("gicp.use_lock_strategy").as_bool();
    loc_cfg.lock_fitness       = this->get_parameter("gicp.lock_fitness").as_double();
    loc_cfg.use_roi            = this->get_parameter("gicp.use_roi").as_bool();
    loc_cfg.roi_x_min          = this->get_parameter("gicp.roi_x_min").as_double();
    loc_cfg.roi_x_max          = this->get_parameter("gicp.roi_x_max").as_double();
    loc_cfg.roi_y_min          = this->get_parameter("gicp.roi_y_min").as_double();
    loc_cfg.roi_y_max          = this->get_parameter("gicp.roi_y_max").as_double();
    loc_cfg.roi_z_min          = this->get_parameter("gicp.roi_z_min").as_double();
    loc_cfg.roi_z_max          = this->get_parameter("gicp.roi_z_max").as_double();

    localization_ = LocalizationStage(map_, loc_cfg);

    // ── detection config ───────────────────────────────────────────
    this->declare_parameter("dynamic.distance_threshold", 0.1);
    this->declare_parameter("dynamic.num_threads", 12);
    this->declare_parameter("dynamic.accumulate_frames", 3);
    this->declare_parameter("dynamic.use_roi", true);
    this->declare_parameter("dynamic.roi_x_min", 0.0);
    this->declare_parameter("dynamic.roi_x_max", 30.0);
    this->declare_parameter("dynamic.roi_y_min", -15.0);
    this->declare_parameter("dynamic.roi_y_max", 15.0);
    this->declare_parameter("dynamic.roi_z_min", 0.0);
    this->declare_parameter("dynamic.roi_z_max", 1.4);

    DynamicCloudConfig dyn_cfg;
    dyn_cfg.distance_threshold =
        static_cast<float>(this->get_parameter("dynamic.distance_threshold").as_double());
    dyn_cfg.num_threads       = this->get_parameter("dynamic.num_threads").as_int();
    dyn_cfg.accumulate_frames = this->get_parameter("dynamic.accumulate_frames").as_int();
    dyn_cfg.use_roi           = this->get_parameter("dynamic.use_roi").as_bool();
    dyn_cfg.roi_x_min = static_cast<float>(this->get_parameter("dynamic.roi_x_min").as_double());
    dyn_cfg.roi_x_max = static_cast<float>(this->get_parameter("dynamic.roi_x_max").as_double());
    dyn_cfg.roi_y_min = static_cast<float>(this->get_parameter("dynamic.roi_y_min").as_double());
    dyn_cfg.roi_y_max = static_cast<float>(this->get_parameter("dynamic.roi_y_max").as_double());
    dyn_cfg.roi_z_min = static_cast<float>(this->get_parameter("dynamic.roi_z_min").as_double());
    dyn_cfg.roi_z_max = static_cast<float>(this->get_parameter("dynamic.roi_z_max").as_double());

    dynamic_stage_ = DynamicCloudStage(dyn_cfg);
    dynamic_stage_.set_map(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map_->pcl_cloud()));

    this->declare_parameter("cluster.tolerance", 0.25);
    this->declare_parameter("cluster.min_size", 5);
    this->declare_parameter("cluster.max_size", 1000);
    ClusterConfig cl_cfg;
    cl_cfg.cluster_tolerance =
        static_cast<float>(this->get_parameter("cluster.tolerance").as_double());
    cl_cfg.min_cluster_size = this->get_parameter("cluster.min_size").as_int();
    cl_cfg.max_cluster_size = this->get_parameter("cluster.max_size").as_int();
    cluster_stage_          = ClusterStage(cl_cfg);

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
        frame.points.reserve(cloud->size());
        for (const auto& pt : cloud->points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)
                && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > DBL_EPSILON)
                frame.points.emplace_back(pt.x, pt.y, pt.z);
        }
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
    transformed.clear();
    transformed.reserve(scan.size());
    for (const auto& p : scan) {
        transformed.push_back(pose.T * p);
    }
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

    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
            msg.pose.covariance[i * 6 + j] = pose.covariance(i, j);

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
