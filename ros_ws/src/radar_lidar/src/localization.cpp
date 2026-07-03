#include "radar_lidar/localization.hpp"

#include <cmath>
#include <print>

#include <Eigen/Cholesky>
#include <small_gicp/registration/registration_helper.hpp>

#include "radar_lidar/map_data.hpp"

namespace radar {

LocalizationStage::LocalizationStage(
    std::shared_ptr<const MapData> map, config::LocalizationConfig cfg)
    : map_(std::move(map))
    , cfg_(cfg)
    , prev_pose_(Eigen::Isometry3d::Identity())
    , spherical_grid_(cfg.spherical_grid_deg)
    , accumulator_(cfg.accumulate_frames > 0 ? static_cast<size_t>(cfg.accumulate_frames) : 1) {
    if (map_ && map_->size() > 0) {
        const auto& mc = map_->sgicp_cloud();
        target_points_.reserve(mc.size());
        for (size_t i = 0; i < mc.size(); ++i) {
            target_points_.emplace_back(mc.point(i).head<3>());
        }
    }
}

auto LocalizationStage::preprocess(const types::Frame& scan) -> types::PointCloud {
    // ROI 裁剪
    types::PointCloud roi_points;
    if (cfg_.use_roi) {
        roi_points.reserve(scan.points.size());
        for (const auto& p : scan.points) {
            if (p.x() >= cfg_.roi_x_min && p.x() <= cfg_.roi_x_max && p.y() >= cfg_.roi_y_min
                && p.y() <= cfg_.roi_y_max && p.z() >= cfg_.roi_z_min && p.z() <= cfg_.roi_z_max) {
                roi_points.push_back(p);
            }
        }
    } else {
        roi_points = scan.points;
    }

    if (!cfg_.use_spherical_grid && cfg_.accumulate_frames == 0) {
        return roi_points;
    }

    // 帧累积
    if (cfg_.accumulate_frames > 0) {
        accumulator_.push(roi_points);
        auto accumulated = accumulator_.all_points();

        // 球面网格预处理
        if (cfg_.use_spherical_grid) {
            spherical_grid_.clear();
            spherical_grid_.add(accumulated);
            return spherical_grid_.extract();
        }
        return accumulated;
    }

    // 只球面网格，不累积
    if (cfg_.use_spherical_grid) {
        spherical_grid_.clear();
        spherical_grid_.add(roi_points);
        return spherical_grid_.extract();
    }

    return roi_points;
}

auto LocalizationStage::process(const types::Frame& scan)
    -> std::expected<types::PoseEstimate, std::string> {
    if (scan.points.empty()) {
        return std::unexpected("Empty scan");
    }
    if (target_points_.empty()) {
        return std::unexpected("Map not loaded");
    }

    // 锁定策略：已锁定则直接返回上一次位姿
    if (cfg_.use_lock_strategy && locked_) {
        types::PoseEstimate out;
        out.T             = prev_pose_;
        out.fitness_score = 0.0;
        out.converged     = true;
        out.covariance    = Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
        return out;
    }

    // 预处理（球面网格 + 帧累积 + ROI）
    auto source_points = preprocess(scan);
    if (source_points.size() < 50) {
        return std::unexpected(
            "Too few points after preprocessing: " + std::to_string(source_points.size()));
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

    auto result = small_gicp::align(target_points_, source_points, prev_pose_, setting);

    prev_pose_ = result.T_target_source;
    if (!result.converged) {
        std::println("[localization] GICP did not converge (score={:.4f}, iter={}) -- using best available transform",
            result.error, result.iterations);
    }

    // 锁定策略：fitness 足够好且收敛则锁定
    if (cfg_.use_lock_strategy && result.converged && result.error < cfg_.lock_fitness) {
        locked_ = true;
    }

    types::PoseEstimate out;
    out.T             = result.T_target_source;
    out.fitness_score = result.error;
    out.converged     = result.converged;

    Eigen::Matrix<double, 6, 6> H_reg = result.H + Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
    out.covariance                    = H_reg.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

    return out;
}

} // namespace radar
