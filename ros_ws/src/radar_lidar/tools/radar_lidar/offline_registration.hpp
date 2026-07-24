#pragma once

#include <cmath>
#include <expected>
#include <limits>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include "radar_lidar/data_format.hpp"

namespace radar_lidar::map_data {
class MapData;
}

namespace radar_lidar::registration {

constexpr auto deg_to_rad(double deg) -> double { return deg * std::numbers::pi / 180.0; }
constexpr auto rad_to_deg(double rad) -> double { return rad * 180.0 / std::numbers::pi; }

struct RegistrationScore {
    double inlier_ratio = 0.0;
    double rmse         = std::numeric_limits<double>::max();
};

auto is_better_score(const RegistrationScore& a, const RegistrationScore& b) -> bool;

auto score_alignment(const pcl::KdTreeFLANN<pcl::PointXYZ>& map_tree,
    const types::PointCloud& source, const Eigen::Isometry3d& T, double inlier_threshold)
    -> RegistrationScore;

// 离线配准参数, 驱动 yaw 多起点搜索 + 粗/精两阶段 GICP。
struct OfflinePoseParams {
    double init_x = 0.0, init_y = 0.0, init_z = 0.0;
    double init_yaw_deg   = 0.0;
    double init_pitch_deg = 0.0;

    bool use_look_at = true;
    double look_at_x = 0.0, look_at_y = 0.0, look_at_z = 0.5;

    double yaw_search_range_deg = 30.0;
    double yaw_search_step_deg  = 10.0;
    double inlier_threshold     = 0.3;

    double coarse_voxel    = 0.5;
    double coarse_max_corr = 30.0;
    int coarse_max_iter    = 50;

    config::LocalizationConfig fine_cfg;
};

struct PoseResult {
    Eigen::Isometry3d t_map_lidar = Eigen::Isometry3d::Identity();
    RegistrationScore score;
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
    bool converged                         = false;
    std::vector<std::string> details; // progressive status messages for node logging
};

// 粗配准 yaw 多起点 + 精配准。ROS 无关, 仅依赖 MapData/Frame/参数结构体。
auto run_offline_registration(std::shared_ptr<const map_data::MapData> map,
    const types::Frame& frame, const OfflinePoseParams& params)
    -> std::expected<PoseResult, std::string>;

} // namespace radar_lidar::registration
