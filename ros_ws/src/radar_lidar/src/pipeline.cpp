#include "radar_lidar/pipeline.hpp"

#include <chrono>

#include <pcl_conversions/pcl_conversions.h>

namespace radar {

LidarPipeline::LidarPipeline()
    : Node("radar_lidar_node")
    , map_(nullptr)
    , localization_(nullptr, { }) {

    this->declare_parameter("map_path", "");
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

    this->declare_parameter("gicp.num_threads", 4);
    this->declare_parameter("gicp.max_corr_distance", 1.0);
    this->declare_parameter("gicp.max_iterations", 48);

    config::LocalizationConfig loc_cfg;
    loc_cfg.num_threads       = this->get_parameter("gicp.num_threads").as_int();
    loc_cfg.max_corr_distance = this->get_parameter("gicp.max_corr_distance").as_double();
    loc_cfg.max_iterations    = this->get_parameter("gicp.max_iterations").as_int();

    localization_ = LocalizationStage(map_, loc_cfg);

    sub_scan_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar",
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_scan(msg); });

    pub_pose_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/lidar/pose", 10);
    pub_diag_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/diagnostics", 10);

    RCLCPP_INFO(get_logger(), "radar_lidar ready. Listening on /livox/lidar");
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
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
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
    if (pose) publish_pose(*pose, frame.stamp);

    const auto t1           = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    publish_diagnostics(pose.value_or(types::PoseEstimate { }), elapsed_ms, frame_count_);
}

void LidarPipeline::publish_pose(const types::PoseEstimate& pose, types::Timestamp stamp) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp    = rclcpp::Time(stamp);
    msg.header.frame_id = "map";

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
    diag.hardware_id = "livox_mid70";
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

} // namespace radar
