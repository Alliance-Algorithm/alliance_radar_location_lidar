#include "radar_lidar/offline_registration.hpp"

#include <cmath>
#include <format>
#include <ranges>
#include <vector>

#include "radar_lidar/geometry_utils.hpp"
#include "radar_lidar/localization_stage.hpp"
#include "radar_lidar/map_data.hpp"

namespace radar_lidar::registration {

auto is_better_score(const RegistrationScore& a, const RegistrationScore& b) -> bool {
    if (std::abs(a.inlier_ratio - b.inlier_ratio) > 1e-6) return a.inlier_ratio > b.inlier_ratio;
    return a.rmse < b.rmse;
}

auto score_alignment(const pcl::KdTreeFLANN<pcl::PointXYZ>& map_tree,
    const types::PointCloud& source, const Eigen::Isometry3d& T, double inlier_threshold)
    -> RegistrationScore {
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

auto run_offline_registration(std::shared_ptr<const map_data::MapData> map,
    const types::Frame& frame, const OfflinePoseParams& p)
    -> std::expected<PoseResult, std::string> {
    PoseResult result;
    const Eigen::Vector3d eye(p.init_x, p.init_y, p.init_z);
    double base_yaw   = deg_to_rad(p.init_yaw_deg);
    double base_pitch = deg_to_rad(p.init_pitch_deg);
    if (p.use_look_at) {
        const auto [yaw, pitch] =
            geom::look_at_yaw_pitch(eye, { p.look_at_x, p.look_at_y, p.look_at_z });
        base_yaw   = yaw;
        base_pitch = pitch;
    }
    result.details.push_back(std::format("Init pose: eye=({:.2f},{:.2f},{:.2f}) yaw={:.2f}° "
                                         "pitch={:.2f}°",
        p.init_x, p.init_y, p.init_z, rad_to_deg(base_yaw), rad_to_deg(base_pitch)));

    // Coarse registration: yaw multi-start search
    auto coarse_cfg              = p.fine_cfg;
    coarse_cfg.voxel_leaf_size   = p.coarse_voxel;
    coarse_cfg.max_corr_distance = p.coarse_max_corr;
    coarse_cfg.max_iterations    = p.coarse_max_iter;
    coarse_cfg.roi.use_roi       = false;

    std::vector<double> yaw_offsets;
    if (p.yaw_search_step_deg > 0.0 && p.yaw_search_range_deg > 0.0) {
        for (double off = -p.yaw_search_range_deg; off <= p.yaw_search_range_deg + 1e-9;
            off += p.yaw_search_step_deg)
            yaw_offsets.push_back(deg_to_rad(off));
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
        auto init_pose    = geom::pose_from_yaw_pitch(eye, base_yaw + yaw_off, base_pitch);
        auto coarse_stage = localization::LocalizationStage(map, coarse_cfg);
        coarse_stage.set_initial_pose(init_pose);
        auto coarse_result             = coarse_stage.process(frame);
        const Eigen::Isometry3d cand_T = coarse_result ? coarse_result->t_map_lidar : init_pose;
        const auto score =
            score_alignment(map->pcl_tree(), frame.points, cand_T, p.inlier_threshold);
        candidates.push_back({ cand_T, score, rad_to_deg(yaw_off),
            coarse_result ? coarse_result->converged : false });
        result.details.push_back(std::format("  yaw_off={:+.1f}° -> inlier={:.3f} rmse={:.4f}",
            rad_to_deg(yaw_off), score.inlier_ratio, score.rmse));
    }

    const auto best = std::ranges::max_element(
        candidates, [](const auto& a, const auto& b) { return is_better_score(b.score, a.score); });
    result.details.push_back(std::format("Best coarse: yaw_off={:+.1f}° inlier={:.3f} rmse={:.4f}",
        best->yaw_offset_deg, best->score.inlier_ratio, best->score.rmse));

    // Fine registration
    auto fine_stage = localization::LocalizationStage(map, p.fine_cfg);
    fine_stage.set_initial_pose(best->t_map_lidar);
    auto fine_result = fine_stage.process(frame);

    result.t_map_lidar = best->t_map_lidar;
    result.score       = best->score;
    result.converged   = best->converged;

    if (fine_result) {
        const auto fine_sc = score_alignment(
            map->pcl_tree(), frame.points, fine_result->t_map_lidar, p.inlier_threshold);
        if (is_better_score(fine_sc, best->score)) {
            result.t_map_lidar = fine_result->t_map_lidar;
            result.covariance  = fine_result->covariance;
            result.score       = fine_sc;
            result.converged   = fine_result->converged;
            result.details.push_back(std::format(
                "Fine OK. inlier={:.3f} rmse={:.4f}", fine_sc.inlier_ratio, fine_sc.rmse));
        } else {
            result.details.push_back(std::format("Fine worse (inlier={:.3f} < coarse={:.3f}), "
                                                 "keeping coarse",
                fine_sc.inlier_ratio, best->score.inlier_ratio));
        }
    } else {
        result.details.push_back("Fine registration failed, keeping coarse");
    }
    return result;
}

} // namespace radar_lidar::registration
