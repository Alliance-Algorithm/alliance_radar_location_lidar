#include "radar_lidar/localization_stage.hpp"

#include <cmath>
#include <cstdio>

#include <Eigen/Cholesky>
#include <small_gicp/registration/registration_helper.hpp>

#include "radar_lidar/geometry_utils.hpp"
#include "radar_lidar/map_data.hpp"

namespace radar_lidar::localization {

namespace {

    [[nodiscard]] auto in_localization_roi(
        const Eigen::Vector3d& point, const config::RoiBounds& roi) -> bool {
        return point.x() > roi.x_min && point.x() < roi.x_max && point.y() > roi.y_min
            && point.y() < roi.y_max && point.z() > roi.z_min && point.z() < roi.z_max;
    }

    [[nodiscard]] auto filter_localization_roi(
        const types::PointCloud& points, const config::RoiBounds& roi) -> types::PointCloud {
        types::PointCloud result;
        result.reserve(points.size());
        for (const auto& point : points) {
            if (in_localization_roi(point, roi)) {
                result.push_back(point);
            }
        }
        return result;
    }

} // namespace

LocalizationStage::LocalizationStage(
    std::shared_ptr<const map_data::MapData> map, config::LocalizationConfig cfg)
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

    if (cfg_.has_initial_pose) {
        prev_pose_ = Eigen::Isometry3d::Identity();
        prev_pose_.translation() =
            Eigen::Vector3d(cfg_.initial_tx, cfg_.initial_ty, cfg_.initial_tz);
        prev_pose_.linear() = (Eigen::AngleAxisd(cfg_.initial_yaw, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(cfg_.initial_pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(cfg_.initial_roll, Eigen::Vector3d::UnitX()))
                                  .toRotationMatrix();
        locked_             = true;
    }
}

auto LocalizationStage::preprocess(const types::Frame& scan) -> types::PointCloud {
    if (!cfg_.use_spherical_grid && cfg_.accumulate_frames == 0) {
        return filter_localization_roi(scan.points, cfg_.roi);
    }

    // 帧累积
    if (cfg_.accumulate_frames > 0) {
        accumulator_.push(scan.points);
        auto accumulated = accumulator_.all_points();

        // 球面网格预处理
        if (cfg_.use_spherical_grid) {
            spherical_grid_.clear();
            spherical_grid_.add(accumulated);
            return filter_localization_roi(spherical_grid_.extract(), cfg_.roi);
        }
        return filter_localization_roi(accumulated, cfg_.roi);
    }

    // 只球面网格，不累积
    if (cfg_.use_spherical_grid) {
        spherical_grid_.clear();
        spherical_grid_.add(scan.points);
        return filter_localization_roi(spherical_grid_.extract(), cfg_.roi);
    }

    return filter_localization_roi(scan.points, cfg_.roi);
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
        // watchdog：低开销残差监测，检测雷达站被移动/碰撞。
        // 超标连续帧达阈值 → 解锁，走完整 GICP 重新配准。
        if (cfg_.enable_watchdog && watchdog_check(scan)) {
            locked_ = false;
            std::printf("[localization_stage] watchdog: residual exceeded threshold, "
                        "unlocking for re-registration\n");
            // 解锁后尝试 coarse 重定位（若启用）
            if (cfg_.enable_coarse_relocalize) {
                if (coarse_relocalize(scan)) {
                    std::printf("[localization_stage] coarse relocalization succeeded, "
                                "re-locked\n");
                }
            }
        }
        types::PoseEstimate out;
        out.t_map_lidar   = prev_pose_;
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

    // 锁定策略：fitness 足够好且收敛则锁定
    if (cfg_.use_lock_strategy && result.converged && result.error < cfg_.lock_fitness) {
        locked_ = true;
    }

    types::PoseEstimate out;
    out.t_map_lidar   = result.T_target_source;
    out.fitness_score = result.error;
    out.converged     = result.converged;

    Eigen::Matrix<double, 6, 6> H_reg = result.H + Eigen::Matrix<double, 6, 6>::Identity() * 1e-6;
    out.covariance                    = H_reg.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

    return out;
}

auto LocalizationStage::watchdog_check(const types::Frame& scan) -> bool {
    const int interval = std::max(1, cfg_.watchdog_check_interval);
    if (++watchdog_frame_count_ % static_cast<std::size_t>(interval) != 0) {
        return false;
    }

    const auto& tree = map_->pcl_tree();
    if (tree.getInputCloud() == nullptr) return false;

    // 用锁定位姿把 scan 变换到 map 系，对降采样后的点查最近邻距离
    const auto& pose       = prev_pose_;
    double sum             = 0.0;
    int count              = 0;
    const std::size_t step = std::max<std::size_t>(1, scan.points.size() / 200); // 采样 ≤200 点
    for (std::size_t i = 0; i < scan.points.size(); i += step) {
        const Eigen::Vector3d p_map = pose * scan.points[i];
        pcl::PointXYZ query(static_cast<float>(p_map.x()), static_cast<float>(p_map.y()),
            static_cast<float>(p_map.z()));
        std::vector<int> idx(1);
        std::vector<float> dist_sq(1);
        if (tree.nearestKSearch(query, 1, idx, dist_sq) == 0) continue;
        sum += std::sqrt(dist_sq[0]);
        ++count;
    }
    if (count == 0) return false;

    const double mean_residual = sum / count;
    if (mean_residual > cfg_.watchdog_fitness) {
        ++watchdog_high_residual_frames_;
    } else {
        watchdog_high_residual_frames_ = 0;
    }
    std::printf("[localization_stage] watchdog residual=%.3f (thr=%.3f, streak=%d)\n",
        mean_residual, cfg_.watchdog_fitness, watchdog_high_residual_frames_);
    return watchdog_high_residual_frames_ >= cfg_.watchdog_unlock_frames;
}

auto LocalizationStage::score_alignment(
    const types::PointCloud& scan, const Eigen::Isometry3d& T) const -> AlignmentScore {
    const auto& tree = map_->pcl_tree();
    if (tree.getInputCloud() == nullptr) return { };

    int inliers   = 0;
    double sum_sq = 0.0;
    int count     = 0;
    for (const auto& p : scan) {
        const Eigen::Vector3d p_map = T * p;
        pcl::PointXYZ query(static_cast<float>(p_map.x()), static_cast<float>(p_map.y()),
            static_cast<float>(p_map.z()));
        std::vector<int> idx(1);
        std::vector<float> dist_sq(1);
        if (tree.nearestKSearch(query, 1, idx, dist_sq) == 0) continue;
        const double d = std::sqrt(dist_sq[0]);
        if (d < cfg_.coarse_inlier_threshold) ++inliers;
        sum_sq += dist_sq[0];
        ++count;
    }
    if (count == 0) return { };
    return { static_cast<double>(inliers) / count, std::sqrt(sum_sq / count) };
}

auto LocalizationStage::coarse_relocalize(const types::Frame& scan) -> bool {
    if (!map_ || scan.points.empty()) return false;

    // 预处理（复用主流程的球面网格/ROI，但不累积）
    auto source_points = preprocess(scan);
    if (source_points.size() < 50) return false;

    // coarse 配置
    auto coarse_cfg              = cfg_;
    coarse_cfg.voxel_leaf_size   = cfg_.coarse_voxel;
    coarse_cfg.max_corr_distance = cfg_.coarse_max_corr;
    coarse_cfg.max_iterations    = cfg_.coarse_max_iter;
    coarse_cfg.roi.use_roi       = false;

    // 以锁定位姿为基准生成 yaw + 平移多起点
    const Eigen::Vector3d base_t = prev_pose_.translation();
    const Eigen::Matrix3d base_R = prev_pose_.rotation();
    const double base_yaw        = std::atan2(base_R(1, 0), base_R(0, 0));

    std::vector<double> yaw_offsets;
    for (double off = -cfg_.coarse_yaw_range_deg; off <= cfg_.coarse_yaw_range_deg + 1e-9;
        off += cfg_.coarse_yaw_step_deg) {
        yaw_offsets.push_back(off * M_PI / 180.0);
    }
    std::vector<double> tx_offsets, ty_offsets;
    for (double off = -cfg_.coarse_translate_range_m; off <= cfg_.coarse_translate_range_m + 1e-9;
        off += cfg_.coarse_translate_step_m) {
        tx_offsets.push_back(off);
        ty_offsets.push_back(off);
    }

    struct Candidate {
        Eigen::Isometry3d t_map_lidar;
        AlignmentScore score;
        double yaw_off_deg;
        double tx_off, ty_off;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(yaw_offsets.size() * tx_offsets.size() * ty_offsets.size());

    for (const double yaw_off : yaw_offsets) {
        for (const double tx_off : tx_offsets) {
            for (const double ty_off : ty_offsets) {
                Eigen::Isometry3d init = Eigen::Isometry3d::Identity();
                init.translation()     = base_t + Eigen::Vector3d(tx_off, ty_off, 0.0);
                init.linear() = (Eigen::AngleAxisd(base_yaw + yaw_off, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()))
                                    .toRotationMatrix();

                // coarse GICP（直接用当前 stage 的 target 点）
                small_gicp::RegistrationSetting setting;
                setting.type                        = small_gicp::RegistrationSetting::GICP;
                setting.downsampling_resolution     = coarse_cfg.voxel_leaf_size;
                setting.max_correspondence_distance = coarse_cfg.max_corr_distance;
                setting.max_iterations              = coarse_cfg.max_iterations;
                setting.rotation_eps                = coarse_cfg.rotation_eps;
                setting.translation_eps             = coarse_cfg.translation_eps;
                setting.num_threads                 = coarse_cfg.num_threads;
                setting.verbose                     = false;

                auto result = small_gicp::align(target_points_, source_points, init, setting);
                const Eigen::Isometry3d cand_T = result.T_target_source;
                const auto score               = score_alignment(scan.points, cand_T);
                candidates.push_back({ cand_T, score, yaw_off * 180.0 / M_PI, tx_off, ty_off });
            }
        }
    }

    // 选 inlier 最高者
    const auto best = std::max_element(
        candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
            return a.score.inlier_ratio < b.score.inlier_ratio;
        });
    if (best == candidates.end() || best->score.inlier_ratio < cfg_.coarse_min_inlier) {
        std::printf("[localization_stage] coarse relocalize FAILED (best inlier=%.3f)\n",
            best == candidates.end() ? 0.0 : best->score.inlier_ratio);
        return false;
    }

    std::printf("[localization_stage] coarse relocalize best: yaw_off=%.1f° t=(%.2f,%.2f) "
                "inlier=%.3f rmse=%.3f\n",
        best->yaw_off_deg, best->tx_off, best->ty_off, best->score.inlier_ratio, best->score.rmse);

    // 精配（用当前完整配置，从 best 出发）
    prev_pose_       = best->t_map_lidar;
    auto source_fine = preprocess(scan);
    small_gicp::RegistrationSetting fine_setting;
    fine_setting.type                        = small_gicp::RegistrationSetting::GICP;
    fine_setting.downsampling_resolution     = cfg_.voxel_leaf_size;
    fine_setting.max_correspondence_distance = cfg_.max_corr_distance;
    fine_setting.max_iterations              = cfg_.max_iterations;
    fine_setting.rotation_eps                = cfg_.rotation_eps;
    fine_setting.translation_eps             = cfg_.translation_eps;
    fine_setting.num_threads                 = cfg_.num_threads;
    fine_setting.verbose                     = cfg_.verbose;

    auto fine  = small_gicp::align(target_points_, source_fine, prev_pose_, fine_setting);
    prev_pose_ = fine.T_target_source;

    // 精配后验证
    const auto final_score = score_alignment(scan.points, prev_pose_);
    if (final_score.inlier_ratio < cfg_.coarse_min_inlier) {
        std::printf("[localization_stage] coarse relocalize fine FAILED (inlier=%.3f)\n",
            final_score.inlier_ratio);
        return false;
    }
    locked_ = true;
    std::printf("[localization_stage] coarse relocalize success: inlier=%.3f rmse=%.3f\n",
        final_score.inlier_ratio, final_score.rmse);
    return true;
}

} // namespace radar_lidar::localization
