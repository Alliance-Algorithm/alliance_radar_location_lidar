#pragma once
#include <expected>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "radar_camera/data_format.hpp"

namespace radar_camera::projection {

struct Triangle {
    Eigen::Vector3f v0, v1, v2;
};

struct Ray {
    Eigen::Vector3f origin;
    Eigen::Vector3f direction;
};

class Projector {
public:
    Projector()  = default;
    ~Projector() = default;
    auto proj_init_camera(const camera_config::CameraConfig& camera_cfg)
        -> std::expected<void, std::string>;

    auto proj_init_map(const projection_config::ProjectionConfig& proj_cfg)
        -> std::expected<void, std::string>;

    auto proj_preprocess(const std::vector<detection::Detection>& detections)
        -> std::expected<std::vector<std::optional<cv::Point2d>>, std::string>;

    auto proj_pixel_to_ray(const cv::Point2d& pixel) -> std::expected<Ray, std::string>;

    auto proj_map_intersect(const Ray& ray) -> std::expected<cv::Point2d, std::string>;

    auto proj_postprocess(const std::vector<std::optional<cv::Point2d>>& projected,
        const std::vector<detection::Detection>& detections)
        -> std::expected<robot_pose::RobotPose, std::string>;

private:
    std::vector<cv::Point2f> undistort_src_;
    std::vector<cv::Point2f> undistort_dst_;

    camera_config::CameraConfig camera_cfg_;
    std::vector<Triangle> triangles_;
    Eigen::Isometry3d t_map_camera_ = Eigen::Isometry3d::Identity();
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool init_ = false;
};

} // namespace radar_camera::projection
