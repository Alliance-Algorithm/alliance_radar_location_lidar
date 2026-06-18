#include "radar_lidar/localization.hpp"

#include <cmath>

#include <small_gicp/registration/registration_helper.hpp>

#include "radar_lidar/map_data.hpp"

namespace radar {

LocalizationStage::LocalizationStage(std::shared_ptr<const MapData> map,
                                     config::LocalizationConfig cfg)
    : map_(std::move(map))
    , cfg_(cfg)
    , prev_pose_(Eigen::Isometry3d::Identity()) {}

auto LocalizationStage::process(const types::Frame& scan)
    -> std::expected<types::PoseEstimate, std::string> {
    if (scan.points.empty()) {
        return std::unexpected("Empty scan");
    }
    if (!map_ || map_->size() == 0) {
        return std::unexpected("Map not loaded");
    }

    // 提取地图点 (已在下采样后的 MapData 中)
    const auto& map_cloud = map_->sgicp_cloud();
    std::vector<Eigen::Vector3d> target_points(map_cloud.size());
    for (size_t i = 0; i < map_cloud.size(); ++i) {
        target_points[i] = map_cloud.point(i).head<3>();
    }

    // small_gicp 配置
    small_gicp::RegistrationSetting setting;
    setting.type                        = small_gicp::RegistrationSetting::GICP;
    setting.downsampling_resolution     = cfg_.voxel_leaf_size;
    setting.max_correspondence_distance = cfg_.max_corr_distance;
    setting.max_iterations              = cfg_.max_iterations;
    setting.rotation_eps                = cfg_.rotation_eps;
    setting.translation_eps             = cfg_.translation_eps;
    setting.num_threads                 = cfg_.num_threads;
    setting.verbose                     = cfg_.verbose;

    auto result = small_gicp::align(target_points, scan.points, prev_pose_, setting);

    // 更新状态
    prev_pose_ = result.T_target_source;

    // 构造输出
    types::PoseEstimate out;
    out.T             = result.T_target_source;
    out.fitness_score = result.error;
    out.converged     = result.converged;

    // 协方差 = Hessian 逆 (带正则化)
    Eigen::Matrix<double, 6, 6> H_reg =
        result.H + Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
    out.covariance = H_reg.inverse();

    return out;
}

}  // namespace radar
