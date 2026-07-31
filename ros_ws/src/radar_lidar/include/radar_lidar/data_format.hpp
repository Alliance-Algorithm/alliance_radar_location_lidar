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

    bool has_initial_pose = false;
    double initial_tx = 0, initial_ty = 0, initial_tz = 0;
    double initial_roll = 0, initial_pitch = 0, initial_yaw = 0;

    RoiBounds roi;
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
