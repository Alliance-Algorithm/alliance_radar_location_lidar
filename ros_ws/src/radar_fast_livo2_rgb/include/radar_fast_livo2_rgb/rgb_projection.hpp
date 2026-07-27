// rgb_projection.hpp — RGB projection primitives for LiDAR point colorization
//
// Task 1 of the FAST-LIVO2 RGB map/replay plan.
// Pure geometric/color utilities with no ROS or hardware dependencies:
//   - Calibration validation
//   - Perspective projection of a single lidar point into camera pixel
//   - Per-pixel nearest-depth update / visibility test
//   - BGR-to-packed-0xRRGGBB conversion
//   - Deterministic quality score for point selection

#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

namespace radar::fast_livo2::rgb {

/// Byte-order of the source camera image.
enum class ColorFormat { BGR, RGB };

/// Quality weights for point selection scoring.
struct QualityWeights {
    double axis_alignment = 1.0;
    double inverse_distance = 1.0;
    double image_center = 0.5;
    double gradient = 0.25;
};

/// Camera intrinsic + extrinsic calibration for lidar->camera projection.
struct Calibration {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    Eigen::Matrix3d rotation_lidar_camera = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation_lidar_camera = Eigen::Vector3d::Zero();
    QualityWeights quality_weights;
};

/// Result of projecting a lidar point into a camera pixel.
struct Projection {
    double u;      ///< Column (pixel x)
    double v;      ///< Row (pixel y)
    double depth;  ///< Depth along camera optical axis (z_camera)
};

/// Calibration with identity extrinsics and 100x100 intrinsics centered at (50,40).
inline auto make_identity_calibration() -> Calibration
{
    return Calibration {
        .fx = 100.0, .fy = 100.0, .cx = 50.0, .cy = 40.0,
        .rotation_lidar_camera = Eigen::Matrix3d::Identity(),
        .translation_lidar_camera = Eigen::Vector3d::Zero()
    };
}

/// Validate that calibration parameters are physically plausible.
/// All intrinsics (fx/fy/cx/cy) must be finite and strictly positive —
/// zero or negative values are placeholders that must be rejected.
inline auto validate_calibration(const Calibration& cal) -> bool
{
    if (!std::isfinite(cal.fx) || cal.fx <= 0.0)
        return false;
    if (!std::isfinite(cal.fy) || cal.fy <= 0.0)
        return false;
    if (!std::isfinite(cal.cx) || cal.cx <= 0.0)
        return false;
    if (!std::isfinite(cal.cy) || cal.cy <= 0.0)
        return false;
    if (!cal.rotation_lidar_camera.allFinite())
        return false;
    if (!cal.translation_lidar_camera.allFinite())
        return false;
    return true;
}

/// Project a single lidar point into camera pixel coordinates.
///
/// Returns std::nullopt if calibration is invalid, the point is behind
/// the camera (z_camera <= 0), or projection falls outside finite range.
inline auto project_lidar_point(
    const Eigen::Vector3d& point_lidar,
    const Calibration& cal) -> std::optional<Projection>
{
    if (!validate_calibration(cal))
        return std::nullopt;

    const Eigen::Vector3d p_cam =
        cal.rotation_lidar_camera * point_lidar + cal.translation_lidar_camera;

    if (p_cam.z() <= 0.0)
        return std::nullopt;

    double inv_z = 1.0 / p_cam.z();
    double u = cal.fx * p_cam.x() * inv_z + cal.cx;
    double v = cal.fy * p_cam.y() * inv_z + cal.cy;

    if (!std::isfinite(u) || !std::isfinite(v))
        return std::nullopt;

    return Projection { .u = u, .v = v, .depth = p_cam.z() };
}

/// Update the depth buffer at (col, row) if `depth` is nearer than stored.
inline void update_nearest_depth(
    cv::Mat& depth_buffer, int col, int row, float depth)
{
    float& stored = depth_buffer.at<float>(row, col);
    if (depth < stored) {
        stored = depth;
    }
}

/// Build a depth buffer initialized to infinity for given image dimensions.
inline auto build_depth_buffer(int width, int height) -> cv::Mat
{
    return cv::Mat(height, width, CV_32FC1,
                   cv::Scalar(std::numeric_limits<float>::infinity()));
}

/// Check whether a point at given depth is visible (not occluded) at (col, row).
inline auto is_visible(
    const cv::Mat& depth_buffer, int col, int row,
    float depth, float tolerance = 1e-6F) -> bool
{
    if (col < 0 || col >= depth_buffer.cols
        || row < 0 || row >= depth_buffer.rows) {
        return false;
    }
    return depth <= depth_buffer.at<float>(row, col) + tolerance;
}

/// Pack OpenCV BGR Vec3b into a 0xRRGGBB packed uint32.
inline auto pack_rgb(const cv::Vec3b& bgr) -> uint32_t
{
    return (static_cast<uint32_t>(bgr[2]) << 16)
         | (static_cast<uint32_t>(bgr[1]) << 8)
         | static_cast<uint32_t>(bgr[0]);
}

/// Pack raw B,G,R bytes into a 0xRRGGBB packed uint32.
inline auto pack_rgb(uint8_t b, uint8_t g, uint8_t r) -> uint32_t
{
    return (static_cast<uint32_t>(r) << 16)
         | (static_cast<uint32_t>(g) << 8)
         | static_cast<uint32_t>(b);
}

/// Pack OpenCV RGB-order Vec3b (R=ch0, G=ch1, B=ch2) into 0xRRGGBB.
/// Use when the cv::Mat holds PixelType_Gvsp_RGB8_Packed data (live SHM).
inline auto pack_rgb_from_rgb_order(const cv::Vec3b& rgb) -> uint32_t
{
    return (static_cast<uint32_t>(rgb[0]) << 16)
         | (static_cast<uint32_t>(rgb[1]) << 8)
         | static_cast<uint32_t>(rgb[2]);
}

/// Compute a deterministic quality score for a projected lidar point.
///
/// Combines four terms:
///   axis_alignment * (z_camera / distance)
///   + inverse_distance * (1.0 / distance)
///   + image_center * center_weight
///   + gradient * normalized_gradient
inline auto quality_score(
    const Eigen::Vector3d& point_lidar,
    const Projection& proj,
    const Calibration& cal,
    int image_width,
    int image_height,
    double normalized_gradient = 0.0) -> double
{
    double distance = point_lidar.norm();
    if (distance <= 0.0)
        return 0.0;

    auto const& w = cal.quality_weights;

    double axis_term = w.axis_alignment * (proj.depth / distance);
    double inv_dist_term = w.inverse_distance * (1.0 / distance);

    double dx = (proj.u - cal.cx) / static_cast<double>(image_width);
    double dy = (proj.v - cal.cy) / static_cast<double>(image_height);
    double center_dist = std::sqrt(dx * dx + dy * dy);
    double center_weight = 1.0 - std::min(center_dist, 1.0);
    double center_term = w.image_center * center_weight;

    double grad_term = w.gradient * normalized_gradient;

    return axis_term + inv_dist_term + center_term + grad_term;
}

} // namespace radar::fast_livo2::rgb
