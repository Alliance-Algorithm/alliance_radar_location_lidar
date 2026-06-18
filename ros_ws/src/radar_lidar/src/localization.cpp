#include "radar_lidar/localization.hpp"

#include <cmath>

#include <small_gicp/registration/registration_helper.hpp>

#include "radar_lidar/map_data.hpp"

namespace radar {

LocalizationStage::LocalizationStage(
    std::shared_ptr<const MapData> map, config::LocalizationConfig cfg)
    : map_(std::move(map))
    , cfg_(cfg)
    , prev_pose_(Eigen::Isometry3d::Identity()) {
    // 构造时一次提取地图点，避免每帧重复拷贝
    if (map_ && map_->size() > 0) {
        const auto& mc = map_->sgicp_cloud();
        target_points_.resize(mc.size());
        for (size_t i = 0; i < mc.size(); ++i) {
            target_points_[i] = mc.point(i).head<3>();
        }
    }
}

auto LocalizationStage::process(const types::Frame& scan)
    -> std::expected<types::PoseEstimate, std::string> {
    if (scan.points.empty()) {
        return std::unexpected("Empty scan");
    }
    if (target_points_.empty()) {
        return std::unexpected("Map not loaded");
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

    auto result = small_gicp::align(target_points_, scan.points, prev_pose_, setting);

    // 只有收敛时才更新状态，避免漂移传播
    if (!result.converged) {
        return std::unexpected(
            "GICP did not converge (score=" + std::to_string(result.error) + ")");
    }
    prev_pose_ = result.T_target_source;

    // 构造输出
    types::PoseEstimate out;
    out.T             = result.T_target_source;
    out.fitness_score = result.error;
    out.converged     = true;

    // 协方差 = Hessian 逆 (带正则化)
    Eigen::Matrix<double, 6, 6> H_reg = result.H + Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
    out.covariance                    = H_reg.inverse();

    return out;
}

} // namespace radar
