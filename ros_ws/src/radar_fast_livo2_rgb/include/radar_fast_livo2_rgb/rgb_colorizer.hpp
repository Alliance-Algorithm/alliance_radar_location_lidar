// rgb_colorizer.hpp — World-point → RGB colour projection orchestrator
//
// Task 5 of the FAST-LIVO2 RGB map/replay plan.
//
// Pure function with zero ROS dependencies. Takes a set of world-frame
// LiDAR points, an odometry pose, a BGR camera image, and a calibrated
// lidar→camera model, then returns a ColorVoxelMap populated with the
// best-quality visible colour observations.
//
// Algorithm:
//   1. Inverse-transform world points to lidar frame via odometry pose.
//   2. Project every lidar point into camera pixels (depth buffer pass).
//   3. Discard occluded points (visibility pass).
//   4. Compute BGR colour + Sobel gradient → quality score.
//   5. Insert into ColorVoxelMap with best-observation semantics.

#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "radar_fast_livo2_rgb/color_voxel_map.hpp"
#include "radar_fast_livo2_rgb/rgb_projection.hpp"

namespace radar::fast_livo2::rgb {

/// Compute the normalised Sobel gradient magnitude at pixel (col, row)
/// in a BGR8 image.  Extracts a small patch, converts to grayscale,
/// applies 3×3 Sobel in X and Y, and normalises by the theoretical
/// maximum of a 3×3 Sobel on uint8 data (1020 * sqrt(2) ≈ 1442.5).
///
/// Returns a value in approximately [0, 1] — higher means stronger edge.
inline auto compute_normalized_gradient(
    const cv::Mat& color, int col, int row, ColorFormat fmt) -> double
{
    constexpr int kHalfPatch = 2;   // 5×5 patch
    constexpr double kMaxSobelMag = 1020.0 * 1.4142135623730951; // ≈ 1442.5

    const int width = color.cols;
    const int height = color.rows;

    int x0 = std::max(0, col - kHalfPatch);
    int y0 = std::max(0, row - kHalfPatch);
    int x1 = std::min(width, col + kHalfPatch + 1);
    int y1 = std::min(height, row + kHalfPatch + 1);

    if (x1 <= x0 || y1 <= y0) return 0.0;

    cv::Rect roi(x0, y0, x1 - x0, y1 - y0);
    cv::Mat patch_color = color(roi);
    cv::Mat patch_gray;
    int code = (fmt == ColorFormat::RGB) ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY;
    cv::cvtColor(patch_color, patch_gray, code);

    cv::Mat grad_x, grad_y;
    cv::Sobel(patch_gray, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(patch_gray, grad_y, CV_32F, 0, 1, 3);

    int cx = col - x0;
    int cy = row - y0;
    float dx = grad_x.at<float>(cy, cx);
    float dy = grad_y.at<float>(cy, cx);
    float mag = std::sqrt(dx * dx + dy * dy);

    return static_cast<double>(mag) / kMaxSobelMag;
}

/// Project a batch of world-frame LiDAR points through an odometry pose
/// and a calibrated camera model, extract BGR colours for visible points,
/// compute a quality score for each observation, and insert them into a
/// fresh ColorVoxelMap.
///
/// \param world_points   World-frame point positions (from FAST-LIVO2).
/// \param color          Camera image (BGR or RGB depending on source).
/// \param odom_pose      T_world_lidar (rigid body pose of lidar in world).
/// \param calibration    Camera intrinsics + lidar→camera extrinsics.
/// \param quality_weights Quality scoring weights.
/// \param voxel_size     Colour voxel map resolution in metres (default 0.10).
/// \param fmt            Byte order of `color` (BGR for ROS replay, RGB for live SHM).
///
/// \return A ColorVoxelMap containing the best-quality visible observations.
///         Points behind the camera, outside the image, or occluded are
///         silently skipped.
inline auto color_world_points(
    const std::vector<Eigen::Vector3d>& world_points,
    const cv::Mat& color,
    const Eigen::Isometry3d& odom_pose,
    const Calibration& calibration,
    const QualityWeights& quality_weights,
    double voxel_size = 0.10,
    ColorFormat fmt = ColorFormat::BGR)
    -> ColorVoxelMap
{
    ColorVoxelMap map(voxel_size);

    if (world_points.empty() || color.empty()) return map;

    // Set quality weights into a copy of calibration for the score call.
    Calibration cal_with_weights = calibration;
    cal_with_weights.quality_weights = quality_weights;

    // Inverse odometry: world point → lidar point.
    const Eigen::Isometry3d lidar_pose = odom_pose.inverse();

    const int width = color.cols;
    const int height = color.rows;

    // ── Pass 1: depth buffer ──────────────────────────────────────────
    auto depth_buffer = build_depth_buffer(width, height);

    struct Candidate {
        Eigen::Vector3d world;
        Eigen::Vector3d lidar;
        Projection proj;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(world_points.size());

    for (const auto& p_world : world_points) {
        Eigen::Vector3d p_lidar = lidar_pose * p_world;
        auto proj = project_lidar_point(p_lidar, calibration);
        if (!proj.has_value()) continue;

        int col = static_cast<int>(std::round(proj->u));
        int row = static_cast<int>(std::round(proj->v));

        if (col < 0 || col >= width || row < 0 || row >= height) continue;

        update_nearest_depth(depth_buffer, col, row,
                             static_cast<float>(proj->depth));
        candidates.push_back({p_world, p_lidar, *proj});
    }

    // ── Pass 2: visibility → colour → quality → insert ────────────────
    for (const auto& c : candidates) {
        int col = static_cast<int>(std::round(c.proj.u));
        int row = static_cast<int>(std::round(c.proj.v));

        if (!is_visible(depth_buffer, col, row,
                        static_cast<float>(c.proj.depth))) {
            continue;
        }

        // Sample colour and compute packed RGB.
        cv::Vec3b pixel = color.at<cv::Vec3b>(row, col);
        uint32_t packed = (fmt == ColorFormat::RGB)
            ? pack_rgb_from_rgb_order(pixel)
            : pack_rgb(pixel);

        // Compute local gradient for the quality score.
        double gradient = compute_normalized_gradient(color, col, row, fmt);

        double score = quality_score(
            c.lidar, c.proj, cal_with_weights,
            width, height, gradient);

        map.insert_if_better(c.world, packed, score);
    }

    return map;
}

} // namespace radar::fast_livo2::rgb
