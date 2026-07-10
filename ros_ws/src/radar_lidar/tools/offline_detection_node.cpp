// 离线检测节点: map + scan → 定位 → 动态点提取 → 聚类 → Foxglove 可视化
//
// 发布话题 (transient_local QoS):
//   /offline_detection/map          PointCloud2     地图 (青色)
//   /offline_detection/scan_raw     PointCloud2     原始扫描 (橙色)
//   /offline_detection/scan_aligned PointCloud2     变换到地图系的扫描 (绿色)
//   /offline_detection/dynamic      PointCloud2     动态点 (红色)
//   /offline_detection/clusters     PointCloud2     聚类质心云 (供下游消费)
//   /offline_detection/diagnostics  DiagnosticStatus 统计信息
//
// TF: 广播 output_frame -> "scan" 静态变换 (用最终配准位姿 t_map_lidar)，
// 使 scan_raw 在 Foxglove 里正确显示在雷达站实际位置, 而非地图原点。

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <format>
#include <numbers>
#include <print>
#include <ranges>
#include <string>
#include <vector>

#include "radar_lidar/cluster.hpp"
#include "radar_lidar/dynamic_cloud.hpp"
#include "radar_lidar/geometry_utils.hpp"
#include "radar_lidar/localization.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/offline_visualization.hpp"
#include "radar_lidar/types.hpp"

namespace {

constexpr auto deg_to_rad(double deg) -> double { return deg * std::numbers::pi / 180.0; }
constexpr auto rad_to_deg(double rad) -> double { return rad * 180.0 / std::numbers::pi; }

// 动态点颜色: 红色 (BGR)
inline constexpr radar::lidar::offline::BgrColor kDynamicColorBgr { 0, 0, 255 };

template <typename PointT>
auto to_rosmsg(const pcl::PointCloud<PointT>& cloud, const std::string& frame_id)
    -> sensor_msgs::msg::PointCloud2 {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame_id;
    return msg;
}

// 配准评分: 内点率 (dist < threshold) + 内点 RMSE, 归一化可跨起点比较。
struct RegistrationScore {
    double inlier_ratio = 0.0;
    double rmse         = std::numeric_limits<double>::max();
};

auto is_better_score(const RegistrationScore& a, const RegistrationScore& b) -> bool {
    if (std::abs(a.inlier_ratio - b.inlier_ratio) > 1e-6) return a.inlier_ratio > b.inlier_ratio;
    return a.rmse < b.rmse;
}

auto score_alignment(const pcl::KdTreeFLANN<pcl::PointXYZ>& map_tree,
    const radar::lidar::types::PointCloud& source, const Eigen::Isometry3d& T,
    double inlier_threshold) -> RegistrationScore {
    const double th2 = inlier_threshold * inlier_threshold;
    std::vector<int> idx(1);
    std::vector<float> sq_dist(1);
    size_t inliers     = 0;
    double sq_dist_sum = 0.0;
    for (const auto& p : source) {
        const Eigen::Vector3d tp = T * p;
        pcl::PointXYZ query(
            static_cast<float>(tp.x()), static_cast<float>(tp.y()), static_cast<float>(tp.z()));
        if (map_tree.nearestKSearch(query, 1, idx, sq_dist) > 0 && sq_dist[0] <= th2) {
            ++inliers;
            sq_dist_sum += sq_dist[0];
        }
    }
    RegistrationScore s;
    if (!source.empty())
        s.inlier_ratio = static_cast<double>(inliers) / static_cast<double>(source.size());
    if (inliers > 0) s.rmse = std::sqrt(sq_dist_sum / static_cast<double>(inliers));
    return s;
}

} // namespace

class OfflineDetectionNode : public rclcpp::Node {
public:
    OfflineDetectionNode()
        : Node("offline_detection_node",
              rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true)) {
        init();
    }

private:
    void init() {
        std::string map_path, scan_path;
        get_parameter("output_frame", output_frame_);
        get_parameter("map_path", map_path);
        get_parameter("scan_path", scan_path);

        if (map_path.empty() || scan_path.empty()) {
            std::println(stderr, "[detection] ERROR: map_path and scan_path required");
            rclcpp::shutdown();
            return;
        }

        // ── 发布者 ─────────────────────────────────────────────────────────
        const auto qos = rclcpp::QoS(1).transient_local();
        pub_map_ = create_publisher<sensor_msgs::msg::PointCloud2>("/offline_detection/map", qos);
        pub_raw_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("/offline_detection/scan_raw", qos);
        pub_aligned_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("/offline_detection/scan_aligned", qos);
        pub_dynamic_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("/offline_detection/dynamic", qos);
        pub_clusters_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("/offline_detection/clusters", qos);
        pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
            "/offline_detection/diagnostics", qos);

        // ── 加载地图 ────────────────────────────────────────────────────────
        double voxel_leaf = 0.1;
        get_parameter("voxel_leaf", voxel_leaf);
        auto map_result = radar::lidar::MapData::load(map_path, voxel_leaf);
        if (!map_result) {
            std::println(stderr, "[detection] ERROR: Map load failed: {}", map_result.error());
            rclcpp::shutdown();
            return;
        }
        auto map = *map_result;
        std::println("[detection] Map: {} points", map->size());
        pub_map_->publish(to_rosmsg(radar::lidar::offline::make_colored_cloud(
                                        map->pcl_cloud(), radar::lidar::offline::kMapColorBgr),
            output_frame_));

        // ── 加载扫描 ────────────────────────────────────────────────────────
        double scan_voxel = 0.1;
        get_parameter("scan_voxel", scan_voxel);
        auto scan_pcl = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_path, *scan_pcl) == -1) {
            std::println(stderr, "[detection] ERROR: Failed to load scan: {}", scan_path);
            rclcpp::shutdown();
            return;
        }
        if (scan_voxel > 0.0) {
            auto downsampled = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setLeafSize(scan_voxel, scan_voxel, scan_voxel);
            vg.setInputCloud(scan_pcl);
            vg.filter(*downsampled);
            std::println("[detection] Scan downsampled: {} → {} points (voxel={})",
                scan_pcl->size(), downsampled->size(), scan_voxel);
            scan_pcl = downsampled;
        }
        auto frame = radar::lidar::geom::filter_valid_points(*scan_pcl);
        std::println("[detection] Scan: {} valid points", frame.points.size());
        if (frame.points.size() < 100) {
            std::println(stderr, "[detection] ERROR: Too few points in scan");
            rclcpp::shutdown();
            return;
        }
        pub_raw_->publish(to_rosmsg(radar::lidar::offline::make_colored_cloud(
                                        frame.points, radar::lidar::offline::kRawScanColorBgr),
            "scan"));

        // ── 定位: 粗配准 yaw 多起点搜索 + 精配准 ────────────────────────────
        double init_x = 0.0, init_y = 0.0, init_z = 0.0, init_yaw_deg = 0.0, init_pitch_deg = 0.0;
        get_parameter("initial_x", init_x);
        get_parameter("initial_y", init_y);
        get_parameter("initial_z", init_z);
        get_parameter("initial_yaw_deg", init_yaw_deg);
        get_parameter("initial_pitch_deg", init_pitch_deg);

        bool use_look_at = true;
        double look_at_x = 0.0, look_at_y = 0.0, look_at_z = 0.5;
        get_parameter("use_look_at", use_look_at);
        get_parameter("look_at_x", look_at_x);
        get_parameter("look_at_y", look_at_y);
        get_parameter("look_at_z", look_at_z);

        double yaw_search_range_deg = 30.0, yaw_search_step_deg = 10.0, inlier_threshold = 0.3;
        get_parameter("yaw_search_range_deg", yaw_search_range_deg);
        get_parameter("yaw_search_step_deg", yaw_search_step_deg);
        get_parameter("inlier_threshold", inlier_threshold);

        const Eigen::Vector3d eye(init_x, init_y, init_z);
        double base_yaw   = deg_to_rad(init_yaw_deg);
        double base_pitch = deg_to_rad(init_pitch_deg);
        if (use_look_at) {
            const auto [yaw, pitch] =
                radar::lidar::geom::look_at_yaw_pitch(eye, { look_at_x, look_at_y, look_at_z });
            base_yaw   = yaw;
            base_pitch = pitch;
        }
        std::println("[detection] Init pose: eye=({:.2f},{:.2f},{:.2f}) yaw={:.2f}° pitch={:.2f}°",
            init_x, init_y, init_z, rad_to_deg(base_yaw), rad_to_deg(base_pitch));

        radar::lidar::config::LocalizationConfig coarse_cfg;
        coarse_cfg.voxel_leaf_size   = 0.5;
        coarse_cfg.max_corr_distance = 30.0;
        coarse_cfg.max_iterations    = 50;
        coarse_cfg.roi.use_roi       = false;
        get_parameter("coarse_voxel", coarse_cfg.voxel_leaf_size);
        get_parameter("coarse_max_corr", coarse_cfg.max_corr_distance);
        get_parameter("coarse_max_iter", coarse_cfg.max_iterations);

        std::vector<double> yaw_offsets;
        if (yaw_search_step_deg > 0.0 && yaw_search_range_deg > 0.0) {
            for (double off = -yaw_search_range_deg; off <= yaw_search_range_deg + 1e-9;
                off += yaw_search_step_deg) {
                yaw_offsets.push_back(deg_to_rad(off));
            }
        } else {
            yaw_offsets.push_back(0.0);
        }

        struct Candidate {
            Eigen::Isometry3d t_map_lidar;
            RegistrationScore score;
            double yaw_offset_deg;
            bool converged;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(yaw_offsets.size());
        for (const double yaw_off : yaw_offsets) {
            auto init_pose =
                radar::lidar::geom::pose_from_yaw_pitch(eye, base_yaw + yaw_off, base_pitch);
            auto coarse_stage = radar::lidar::LocalizationStage(map, coarse_cfg);
            coarse_stage.set_initial_pose(init_pose);
            auto coarse_result             = coarse_stage.process(frame);
            const Eigen::Isometry3d cand_T = coarse_result ? coarse_result->t_map_lidar : init_pose;
            const auto score =
                score_alignment(map->pcl_tree(), frame.points, cand_T, inlier_threshold);
            candidates.push_back({ cand_T, score, rad_to_deg(yaw_off),
                coarse_result ? coarse_result->converged : false });
            std::println("[detection]   yaw_off={:+.1f}° → inlier={:.3f} rmse={:.4f}",
                rad_to_deg(yaw_off), score.inlier_ratio, score.rmse);
        }

        const auto best = std::ranges::max_element(candidates,
            [](const auto& a, const auto& b) { return is_better_score(b.score, a.score); });
        std::println("[detection] Best coarse: yaw_off={:+.1f}° inlier={:.3f} rmse={:.4f}",
            best->yaw_offset_deg, best->score.inlier_ratio, best->score.rmse);

        radar::lidar::config::LocalizationConfig fine_cfg;
        fine_cfg.voxel_leaf_size   = voxel_leaf;
        fine_cfg.max_corr_distance = 3.0;
        fine_cfg.roi.use_roi       = false;
        get_parameter("max_corr_distance", fine_cfg.max_corr_distance);
        get_parameter("max_iterations", fine_cfg.max_iterations);
        get_parameter("num_threads", fine_cfg.num_threads);

        auto fine_stage = radar::lidar::LocalizationStage(map, fine_cfg);
        fine_stage.set_initial_pose(best->t_map_lidar);
        auto fine_result = fine_stage.process(frame);

        Eigen::Isometry3d t_map_lidar = best->t_map_lidar;
        RegistrationScore reg_score   = best->score;
        if (fine_result) {
            const auto fine_sc = score_alignment(
                map->pcl_tree(), frame.points, fine_result->t_map_lidar, inlier_threshold);
            if (is_better_score(fine_sc, best->score)) {
                t_map_lidar = fine_result->t_map_lidar;
                reg_score   = fine_sc;
                std::println("[detection] Fine OK. inlier={:.3f} rmse={:.4f}", fine_sc.inlier_ratio,
                    fine_sc.rmse);
            } else {
                std::println("[detection] Fine worse, keeping coarse");
            }
        } else {
            std::println(stderr, "[detection] Fine registration failed, keeping coarse");
        }

        // ── TF: output_frame → "scan" ──────────────────────────────────────
        // 让 scan_raw (以 "scan" 帧发布) 在 Foxglove 里正确渲染在雷达站实际位置
        {
            auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp            = now();
            tf_msg.header.frame_id         = output_frame_;
            tf_msg.child_frame_id          = "scan";
            tf_msg.transform.translation.x = t_map_lidar.translation().x();
            tf_msg.transform.translation.y = t_map_lidar.translation().y();
            tf_msg.transform.translation.z = t_map_lidar.translation().z();
            const Eigen::Quaterniond q(t_map_lidar.rotation());
            tf_msg.transform.rotation.x = q.x();
            tf_msg.transform.rotation.y = q.y();
            tf_msg.transform.rotation.z = q.z();
            tf_msg.transform.rotation.w = q.w();
            tf_broadcaster->sendTransform(tf_msg);
        }

        // ── 扫描变换到地图系 ─────────────────────────────────────────────────
        auto scan_in_map = frame.points | std::views::filter([](const auto& pt) {
            return radar::lidar::offline::is_valid_xyz(pt);
        }) | std::views::transform([&t_map_lidar](const auto& pt) { return t_map_lidar * pt; })
            | std::ranges::to<radar::lidar::types::PointCloud>();
        pub_aligned_->publish(to_rosmsg(radar::lidar::offline::make_colored_cloud(scan_in_map,
                                            radar::lidar::offline::kAlignedScanColorBgr),
            output_frame_));
        std::println("[detection] Scan aligned: {} points in map frame", scan_in_map.size());

        // ── 动态点提取 ───────────────────────────────────────────────────────
        radar::lidar::DynamicCloudConfig dynamic_cfg;
        get_parameter("dynamic_distance_threshold", dynamic_cfg.distance_threshold);
        dynamic_cfg.accumulate_frames = 0; // 单帧离线, 不做多帧累积
        int dynamic_threads           = 4;
        get_parameter("dynamic_num_threads", dynamic_threads);
        dynamic_cfg.num_threads = dynamic_threads;

        radar::lidar::DynamicCloudStage dynamic_stage(dynamic_cfg);
        dynamic_stage.set_map(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(map->pcl_cloud()));

        auto dynamic_result = dynamic_stage.process(scan_in_map);
        if (!dynamic_result) {
            std::println(
                stderr, "[detection] Dynamic extraction failed: {}", dynamic_result.error());
            rclcpp::shutdown();
            return;
        }
        const auto& dynamic_points = *dynamic_result;
        std::println("[detection] Dynamic points: {}", dynamic_points.size());

        if (!dynamic_points.empty()) {
            pub_dynamic_->publish(to_rosmsg(
                radar::lidar::offline::make_colored_cloud(dynamic_points, kDynamicColorBgr),
                output_frame_));
        }

        // ── 聚类 ──────────────────────────────────────────────────────────────
        radar::lidar::ClusterConfig cluster_cfg;
        get_parameter("cluster_tolerance", cluster_cfg.cluster_tolerance);
        get_parameter("min_cluster_size", cluster_cfg.min_cluster_size);
        get_parameter("max_cluster_size", cluster_cfg.max_cluster_size);

        radar::lidar::ClusterStage cluster_stage(cluster_cfg);
        auto cluster_result = cluster_stage.process(dynamic_points);
        if (!cluster_result) {
            std::println(stderr, "[detection] Clustering failed: {}", cluster_result.error());
            rclcpp::shutdown();
            return;
        }
        const auto& clusters = *cluster_result;
        std::println("[detection] Clusters: {}", clusters.size());

        // 质心 PointCloud2
        if (!clusters.empty()) {
            pcl::PointCloud<pcl::PointXYZ> centroids;
            centroids.reserve(clusters.size());
            for (const auto& c : clusters) {
                centroids.emplace_back(static_cast<float>(c.centroid.x()),
                    static_cast<float>(c.centroid.y()), static_cast<float>(c.centroid.z()));
            }
            centroids.width    = centroids.size();
            centroids.height   = 1;
            centroids.is_dense = true;
            pub_clusters_->publish(to_rosmsg(centroids, output_frame_));

            for (size_t i = 0; i < clusters.size(); ++i) {
                const auto& c = clusters[i];
                const auto sz = c.max_bound - c.min_bound;
                std::println("[detection]   cluster[{}]: centroid=({:.2f},{:.2f},{:.2f}) "
                             "pts={} size=({:.2f},{:.2f},{:.2f})",
                    i, c.centroid.x(), c.centroid.y(), c.centroid.z(), c.point_count, sz.x(),
                    sz.y(), sz.z());
            }
        }

        // ── 诊断状态 ─────────────────────────────────────────────────────────
        diagnostic_msgs::msg::DiagnosticStatus diag;
        diag.name    = "offline_detection";
        diag.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diag.message = std::format("dynamic={} clusters={} inlier={:.3f} rmse={:.4f}",
            dynamic_points.size(), clusters.size(), reg_score.inlier_ratio, reg_score.rmse);
        pub_diag_->publish(diag);

        std::println("[detection] Done. dynamic={} clusters={}  Spinning for Foxglove.",
            dynamic_points.size(), clusters.size());
    }

    std::string output_frame_ { "map" };

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_, pub_raw_, pub_aligned_,
        pub_dynamic_, pub_clusters_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OfflineDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
