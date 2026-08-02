#include "radar_lidar/odin_tune_node.hpp"

#include <algorithm>
#include <exception>
#include <format>
#include <print>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "radar_lidar/geometry_utils.hpp"
#include "radar_lidar/odin_tune/cloud_point.hpp"

namespace radar_lidar::node {

namespace {
    constexpr std::int64_t kMaxOdomDelayNs = 500'000'000LL; // 0.5s
} // namespace

OdinTuneNode::OdinTuneNode()
    : Node("odin_tune_node",
          rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true))
    , pose_buffer_(kMaxOdomDelayNs) {
    init();
}

void OdinTuneNode::init() {
    declare_and_load_params();
    init_map_background();
    rebuild_stages();

    sub_scan_ = create_subscription<sensor_msgs::msg::PointCloud2>(params_.scan_topic,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_scan(msg); });
    sub_odom_ =
        create_subscription<nav_msgs::msg::Odometry>(params_.odom_topic, rclcpp::SensorDataQoS(),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });

    pub_dynamic_    = create_publisher<sensor_msgs::msg::PointCloud2>("/odin_tune/dynamic", 10);
    pub_background_ = create_publisher<sensor_msgs::msg::PointCloud2>("/odin_tune/background", 10);
    pub_clusters_   = create_publisher<sensor_msgs::msg::PointCloud2>("/odin_tune/clusters", 10);
    pub_cluster_viz_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/odin_tune/cluster_viz", 10);
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/odin_tune/diag", 10);

    RCLCPP_INFO(get_logger(),
        "odin_tune ready. mode=%s scan=%s odom=%s bg_frames=%d diff=%.3f cluster_tol=%.3f",
        map_differencer_ ? "map-static-background" : "frame-difference", params_.scan_topic.c_str(),
        params_.odom_topic.c_str(), params_.bg_num_frames, params_.diff_threshold,
        params_.cluster.cluster_tolerance);
}

void OdinTuneNode::init_map_background() {
    if (params_.map_path.empty()) {
        return;
    }
    // 固定安装位姿（地图系）：与主链路 initial_pose 同约定（Rz*Ry*Rx）
    t_map_lidar_ = Eigen::Isometry3d::Identity();
    t_map_lidar_.translation() =
        Eigen::Vector3d(params_.initial_tx, params_.initial_ty, params_.initial_tz);
    t_map_lidar_.linear() = (Eigen::AngleAxisd(params_.initial_yaw, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(params_.initial_pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(params_.initial_roll, Eigen::Vector3d::UnitX()))
                                .toRotationMatrix();

    auto map_result = map_data::MapData::load(params_.map_path, 0.1);
    if (!map_result) {
        RCLCPP_FATAL(get_logger(), "Failed to load map %s: %s", params_.map_path.c_str(),
            map_result.error().c_str());
        rclcpp::shutdown();
        return;
    }
    const auto& map_cloud = (*map_result)->pcl_cloud();
    types::PointCloud map_pts;
    map_pts.reserve(map_cloud.size());
    for (const auto& p : map_cloud.points) {
        map_pts.emplace_back(p.x, p.y, p.z);
    }
    map_differencer_ = odin_tune::MapDifferencer(map_pts, params_.diff_threshold);
    RCLCPP_INFO(get_logger(), "Map background loaded: %zu points from %s", map_pts.size(),
        params_.map_path.c_str());
}

void OdinTuneNode::declare_and_load_params() {
    auto declare = [this](const std::string& name, const auto& def) {
        if (!has_parameter(name)) {
            declare_parameter(name, rclcpp::ParameterValue(def));
        }
    };
    declare("scan_topic", params_.scan_topic);
    declare("odom_topic", params_.odom_topic);
    declare("output_frame", params_.output_frame);
    declare("conf_threshold", params_.conf_threshold);
    declare("voxel_leaf", params_.voxel_leaf);
    declare("roi_enabled", params_.roi.use_roi);
    declare("roi_x_min", params_.roi.x_min);
    declare("roi_x_max", params_.roi.x_max);
    declare("roi_y_min", params_.roi.y_min);
    declare("roi_y_max", params_.roi.y_max);
    declare("roi_z_min", params_.roi.z_min);
    declare("roi_z_max", params_.roi.z_max);
    declare("bg_num_frames", params_.bg_num_frames);
    declare("diff_threshold", params_.diff_threshold);
    declare("cluster_tolerance", params_.cluster.cluster_tolerance);
    declare("min_cluster_size", params_.cluster.min_cluster_size);
    declare("max_cluster_size", params_.cluster.max_cluster_size);
    declare("map_path", params_.map_path);
    declare("initial_tx", params_.initial_tx);
    declare("initial_ty", params_.initial_ty);
    declare("initial_tz", params_.initial_tz);
    declare("initial_roll", params_.initial_roll);
    declare("initial_pitch", params_.initial_pitch);
    declare("initial_yaw", params_.initial_yaw);

    auto get = [this](const std::string& name, auto& dst) {
        dst = get_parameter(name).get_value<std::decay_t<decltype(dst)>>();
    };
    get("scan_topic", params_.scan_topic);
    get("odom_topic", params_.odom_topic);
    get("output_frame", params_.output_frame);
    get("conf_threshold", params_.conf_threshold);
    get("voxel_leaf", params_.voxel_leaf);
    get("roi_enabled", params_.roi.use_roi);
    get("roi_x_min", params_.roi.x_min);
    get("roi_x_max", params_.roi.x_max);
    get("roi_y_min", params_.roi.y_min);
    get("roi_y_max", params_.roi.y_max);
    get("roi_z_min", params_.roi.z_min);
    get("roi_z_max", params_.roi.z_max);
    get("bg_num_frames", params_.bg_num_frames);
    get("diff_threshold", params_.diff_threshold);
    get("cluster_tolerance", params_.cluster.cluster_tolerance);
    get("min_cluster_size", params_.cluster.min_cluster_size);
    get("max_cluster_size", params_.cluster.max_cluster_size);
    get("map_path", params_.map_path);
    get("initial_tx", params_.initial_tx);
    get("initial_ty", params_.initial_ty);
    get("initial_tz", params_.initial_tz);
    get("initial_roll", params_.initial_roll);
    get("initial_pitch", params_.initial_pitch);
    get("initial_yaw", params_.initial_yaw);

    add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        for (const auto& p : params) {
            const auto& n = p.get_name();
            if (n == "scan_topic" || n == "odom_topic" || n == "output_frame") {
                continue; // topic/frame 变更需重启
            }
            const std::string allowed[] = { "conf_threshold", "voxel_leaf", "roi_enabled",
                "roi_x_min", "roi_x_max", "roi_y_min", "roi_y_max", "roi_z_min", "roi_z_max",
                "bg_num_frames", "diff_threshold", "cluster_tolerance", "min_cluster_size",
                "max_cluster_size" };
            if (std::find(std::begin(allowed), std::end(allowed), n) == std::end(allowed)) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = false;
                result.reason     = "unknown parameter: " + n;
                return result;
            }
        }
        const int prev_bg_num_frames = params_.bg_num_frames;
        for (const auto& p : params) {
            const auto& n = p.get_name();
            if (n == "conf_threshold") params_.conf_threshold = p.as_double();
            else if (n == "voxel_leaf") params_.voxel_leaf = p.as_double();
            else if (n == "roi_enabled") params_.roi.use_roi = p.as_bool();
            else if (n == "roi_x_min") params_.roi.x_min = p.as_double();
            else if (n == "roi_x_max") params_.roi.x_max = p.as_double();
            else if (n == "roi_y_min") params_.roi.y_min = p.as_double();
            else if (n == "roi_y_max") params_.roi.y_max = p.as_double();
            else if (n == "roi_z_min") params_.roi.z_min = p.as_double();
            else if (n == "roi_z_max") params_.roi.z_max = p.as_double();
            else if (n == "bg_num_frames") params_.bg_num_frames = p.as_int();
            else if (n == "diff_threshold") params_.diff_threshold = p.as_double();
            else if (n == "cluster_tolerance") params_.cluster.cluster_tolerance = p.as_double();
            else if (n == "min_cluster_size") params_.cluster.min_cluster_size = p.as_int();
            else if (n == "max_cluster_size") params_.cluster.max_cluster_size = p.as_int();
        }
        if (params_.bg_num_frames != prev_bg_num_frames) {
            rebuild_background();
        }
        rebuild_differencer_and_cluster();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason     = "";
        return result;
    });
}

void OdinTuneNode::rebuild_stages() {
    rebuild_background();
    rebuild_differencer_and_cluster();
}

void OdinTuneNode::rebuild_differencer_and_cluster() {
    differencer_   = odin_tune::FrameDifferencer(params_.diff_threshold);
    cluster_stage_ = cluster::ClusterStage(params_.cluster);
}

void OdinTuneNode::rebuild_background() {
    bg_model_ = odin_tune::BackgroundModel(params_.bg_num_frames);
}

void OdinTuneNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr& msg) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    const auto& p          = msg->pose.pose.position;
    const auto& o          = msg->pose.pose.orientation;
    pose.translation()     = Eigen::Vector3d(p.x, p.y, p.z);
    pose.linear()          = Eigen::Quaterniond(o.w, o.x, o.y, o.z).toRotationMatrix();
    pose_buffer_.add(rclcpp::Time(msg->header.stamp).nanoseconds(), pose);
}

void OdinTuneNode::on_scan(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    ++frame_count_;
    const auto t0 = std::chrono::steady_clock::now();

    pcl::PointCloud<odin_tune::OdinPoint> raw;
    try {
        pcl::fromROSMsg(*msg, raw);
    } catch (const std::exception& e) {
        ++skipped_parse_error_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "cloud_raw parse failed, skipping frame (skipped=%lu): %s", skipped_parse_error_,
            e.what());
        return;
    }

    const types::Timestamp stamp = rclcpp::Time(msg->header.stamp).nanoseconds();

    // 发布时间戳用主机时间轴（Odin 设备时间为内部时间轴，与主机差约 56 年，
    // Foxglove 等可视化会按消息时间戳过滤/同步，设备时间会导致不渲染）
    const types::Timestamp pub_stamp = now().nanoseconds();

    // 1. confidence 过滤 + 无效点过滤 → 雷达系点云
    types::PointCloud frame_pts;
    frame_pts.reserve(raw.size());
    for (const auto& pt : raw.points) {
        if (pt.confidence < static_cast<std::uint16_t>(params_.conf_threshold)) continue;
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        frame_pts.emplace_back(pt.x, pt.y, pt.z);
    }
    if (frame_pts.empty()) return;

    // 2. 体素下采样（可选）
    if (params_.voxel_leaf > 0.0) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>);
        in->reserve(frame_pts.size());
        for (const auto& p : frame_pts) {
            in->emplace_back(
                static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
        }
        in->width    = in->size();
        in->height   = 1;
        in->is_dense = true;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(static_cast<float>(params_.voxel_leaf),
            static_cast<float>(params_.voxel_leaf), static_cast<float>(params_.voxel_leaf));
        vg.setInputCloud(in);
        vg.filter(*ds);
        frame_pts.clear();
        frame_pts.reserve(ds->size());
        for (const auto& p : ds->points) {
            frame_pts.emplace_back(p.x, p.y, p.z);
        }
    }

    // 地图模式：静态地图背景差分
    if (map_differencer_) {
        // 雷达系 → 地图系（固定安装位姿）
        types::PointCloud scan_in_map;
        scan_in_map.reserve(frame_pts.size());
        for (const auto& p : frame_pts) {
            scan_in_map.push_back(t_map_lidar_ * p);
        }
        auto dynamic_pts = map_differencer_->differ(scan_in_map);
        publish_background(scan_in_map, pub_stamp);

        if (params_.roi.use_roi) {
            dynamic_pts = geom::clip_roi_aabb(dynamic_pts, params_.roi);
        }

        std::vector<cluster::ClusterResult> clusters;
        if (!dynamic_pts.empty()) {
            auto result = cluster_stage_->process(dynamic_pts);
            if (result) {
                clusters = *result;
            } else {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000, "ClusterStage: %s", result.error().c_str());
            }
        }

        publish_dynamic(dynamic_pts, pub_stamp);
        publish_clusters(clusters, pub_stamp);

        const auto t1           = std::chrono::steady_clock::now();
        const double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        publish_diag(dynamic_pts.size(), clusters.size(), elapsed_ms, pub_stamp);
        return;
    }

    // 3. 当前帧位姿
    const auto pose = pose_buffer_.lookup(stamp);
    if (!pose) {
        ++skipped_no_odom_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "No odometry for stamp %ld (skipped=%lu)", stamp, skipped_no_odom_);
        return;
    }

    // 传感器系 → odom 系（差分在传感器系完成，ROI/发布在场地系）
    const auto to_odom = [&pose](const types::PointCloud& pts) {
        types::PointCloud result;
        result.reserve(pts.size());
        for (const auto& p : pts) {
            result.push_back(*pose * p);
        }
        return result;
    };

    // 4. 背景差分（背景不含当前帧：先 differ 后 add）
    const auto background = bg_model_->align_to(*pose);
    if (bg_model_->frame_count() < params_.bg_num_frames) {
        ++skipped_warmup_;
        bg_model_->add(frame_pts, *pose);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "Background warmup %d/%d (skipped=%lu)", bg_model_->frame_count(),
            params_.bg_num_frames, skipped_warmup_);
        publish_background(to_odom(background), pub_stamp);
        return;
    }
    bg_model_->add(frame_pts, *pose);

    auto dynamic_pts = differencer_->differ(frame_pts, background);
    publish_background(to_odom(background), pub_stamp);

    // 5. 动态点变换到 odom 系（车辆移动时传感器系点与场地系不一致）
    dynamic_pts = to_odom(dynamic_pts);

    // 6. 可选 ROI 裁剪（odom/场地系）
    if (params_.roi.use_roi) {
        dynamic_pts = geom::clip_roi_aabb(dynamic_pts, params_.roi);
    }

    // 7. 聚类
    std::vector<cluster::ClusterResult> clusters;
    if (!dynamic_pts.empty()) {
        auto result = cluster_stage_->process(dynamic_pts);
        if (result) {
            clusters = *result;
        } else {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "ClusterStage: %s", result.error().c_str());
        }
    }

    publish_dynamic(dynamic_pts, pub_stamp);
    publish_clusters(clusters, pub_stamp);

    const auto t1           = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    publish_diag(dynamic_pts.size(), clusters.size(), elapsed_ms, pub_stamp);
}

void OdinTuneNode::publish_dynamic(const types::PointCloud& pts, types::Timestamp stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(pts.size());
    for (const auto& p : pts) {
        cloud->emplace_back(
            static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    }
    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp    = rclcpp::Time(stamp);
    msg.header.frame_id = params_.output_frame;
    pub_dynamic_->publish(msg);
}

void OdinTuneNode::publish_background(const types::PointCloud& pts, types::Timestamp stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(pts.size());
    for (const auto& p : pts) {
        cloud->emplace_back(
            static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    }
    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp    = rclcpp::Time(stamp);
    msg.header.frame_id = params_.output_frame;
    pub_background_->publish(msg);
}

void OdinTuneNode::publish_clusters(
    const std::vector<cluster::ClusterResult>& clusters, types::Timestamp stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroids(new pcl::PointCloud<pcl::PointXYZ>);
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
    centroid_msg.header.frame_id = params_.output_frame;
    pub_clusters_->publish(centroid_msg);

    visualization_msgs::msg::MarkerArray markers;
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& c = clusters[i];

        visualization_msgs::msg::Marker box;
        box.header.stamp       = rclcpp::Time(stamp);
        box.header.frame_id    = params_.output_frame;
        box.ns                 = "clusters";
        box.id                 = static_cast<int>(i);
        box.type               = visualization_msgs::msg::Marker::CUBE;
        box.action             = visualization_msgs::msg::Marker::ADD;
        box.pose.position.x    = (c.min_bound.x() + c.max_bound.x()) / 2.0;
        box.pose.position.y    = (c.min_bound.y() + c.max_bound.y()) / 2.0;
        box.pose.position.z    = (c.min_bound.z() + c.max_bound.z()) / 2.0;
        box.pose.orientation.w = 1.0;
        box.scale.x            = std::max(0.01, c.max_bound.x() - c.min_bound.x());
        box.scale.y            = std::max(0.01, c.max_bound.y() - c.min_bound.y());
        box.scale.z            = std::max(0.01, c.max_bound.z() - c.min_bound.z());
        box.color.r            = 0.0f;
        box.color.g            = 1.0f;
        box.color.b            = 0.0f;
        box.color.a            = 0.3f;
        box.lifetime           = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(box);

        visualization_msgs::msg::Marker centroid;
        centroid.header.stamp       = rclcpp::Time(stamp);
        centroid.header.frame_id    = params_.output_frame;
        centroid.ns                 = "centroids";
        centroid.id                 = static_cast<int>(i);
        centroid.type               = visualization_msgs::msg::Marker::SPHERE;
        centroid.action             = visualization_msgs::msg::Marker::ADD;
        centroid.pose.position.x    = c.centroid.x();
        centroid.pose.position.y    = c.centroid.y();
        centroid.pose.position.z    = c.centroid.z();
        centroid.pose.orientation.w = 1.0;
        centroid.scale.x            = 0.15;
        centroid.scale.y            = 0.15;
        centroid.scale.z            = 0.15;
        centroid.color.r            = 1.0f;
        centroid.color.g            = 0.0f;
        centroid.color.b            = 0.0f;
        centroid.color.a            = 1.0f;
        centroid.lifetime           = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(centroid);
    }
    pub_cluster_viz_->publish(markers);
}

void OdinTuneNode::publish_diag(std::size_t dynamic_count, std::size_t cluster_count,
    double elapsed_ms, types::Timestamp stamp) {
    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name    = "odin_tune/detection";
    diag.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag.message = std::format(
        "dynamic={} clusters={} time_ms={:.2f}", dynamic_count, cluster_count, elapsed_ms);
    diag.hardware_id = "odin1";
    pub_diag_->publish(diag);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s", diag.message.c_str());
}

} // namespace radar_lidar::node
