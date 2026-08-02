#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace radar_lidar::types {

using Point      = Eigen::Vector3d;
using PointCloud = std::vector<Point>;
using Timestamp  = int64_t;

struct Frame {
    PointCloud points;
    Timestamp stamp { 0 };
    std::string frame_id;
};

struct PoseEstimate {
    Eigen::Isometry3d t_map_lidar          = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
    double fitness_score                   = 0.0;
    bool converged                         = false;
};

} // namespace radar_lidar::types

namespace radar_lidar::config {

struct RoiBounds {
    bool use_roi = false;
    double x_min = 0, x_max = 30;
    double y_min = -15, y_max = 15;
    double z_min = 0, z_max = 7;
};

struct ClusterConfig {
    double cluster_tolerance = 0.25;
    int min_cluster_size     = 5;
    int max_cluster_size     = 1000;
};

struct DynamicCloudConfig {
    double distance_threshold = 0.1;
    int num_threads           = 12;
    int accumulate_frames     = 3;
    RoiBounds roi { .use_roi = true,
        .x_min               = 0,
        .x_max               = 30,
        .y_min               = -15,
        .y_max               = 15,
        .z_min               = 0,
        .z_max               = 1.4 };
};

struct LocalizationConfig {
    double voxel_leaf_size   = 0.1;
    double max_corr_distance = 1.0;
    int max_iterations       = 48;
    double rotation_eps      = 0.03;
    double translation_eps   = 0.1;
    int num_threads          = 4;
    bool verbose             = false;

    bool use_spherical_grid   = true;
    double spherical_grid_deg = 0.1;
    int accumulate_frames     = 20;

    bool use_lock_strategy = true;
    double lock_fitness    = 0.2;

    // 锁定后 watchdog：低开销残差监测，检测雷达站被移动/碰撞
    // 残差 = 当前 scan 在锁定位姿下与地图的最近邻距离均值（每帧或抽帧）
    // 若残差连续 exceed_watchdog_frames 帧 > watchdog_fitness → 解锁重配准
    bool enable_watchdog        = false;
    double watchdog_fitness     = 0.5;
    int watchdog_check_interval = 10; // 锁定后每 N 帧检测一次
    int watchdog_unlock_frames  = 3;  // 连续超标帧数 → 解锁

    // 解锁后 coarse 重定位搜索（watchdog 触发时自动执行）
    // 在锁定位姿附近做 yaw + 平移多起点 coarse GICP，inlier 选优后精配
    bool enable_coarse_relocalize   = false;
    double coarse_yaw_range_deg     = 30.0; // yaw 搜索范围 ±
    double coarse_yaw_step_deg      = 5.0;  // yaw 搜索步长
    double coarse_translate_range_m = 3.0;  // 平移搜索范围 ± (x/y)
    double coarse_translate_step_m  = 1.0;  // 平移搜索步长
    double coarse_voxel             = 0.5;  // coarse 配准降采样
    double coarse_max_corr          = 5.0;  // coarse 对应距离
    int coarse_max_iter             = 20;   // coarse 迭代上限
    double coarse_inlier_threshold  = 0.3;  // inlier 判定阈值 (m)
    double coarse_min_inlier        = 0.3;  // 重定位成功所需最小 inlier 比

    bool has_initial_pose = false;
    double initial_tx = 0, initial_ty = 0, initial_tz = 0;
    double initial_roll = 0, initial_pitch = 0, initial_yaw = 0;

    // 定位 ROI（map 系，场地覆盖范围）：默认全场地
    RoiBounds roi { .use_roi = true,
        .x_min               = -15,
        .x_max               = 30,
        .y_min               = -15,
        .y_max               = 15,
        .z_min               = -2,
        .z_max               = 7 };
};

} // namespace radar_lidar::config

namespace radar_lidar::cluster {

struct ClusterResult {
    Eigen::Vector3d centroid { 0, 0, 0 };
    Eigen::Vector3d min_bound { 0, 0, 0 };
    Eigen::Vector3d max_bound { 0, 0, 0 };
    int point_count = 0;
};

} // namespace radar_lidar::cluster
