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

class Projector {
public:
    Projector()  = default;
    ~Projector() = default;
    auto proj_init(const camera_config::CameraConfig& camera_cfg,
                   const projection_config::ProjectionConfig& proj_cfg)
        -> std::expected<void, std::string>;

    auto proj_runtime(const cv::Point2d& pixel) -> std::expected<cv::Point2d, std::string>;

    auto proj_process(const std::vector<detection::Detection>& detections)
        -> std::expected<robot_pose::RobotPose, std::string>;

private:
    auto map_init(const std::string& mesh_path) -> std::expected<void, std::string>;
    auto map_intersect(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction) const
        -> std::optional<Eigen::Vector3f>;

    camera_config::CameraConfig camera_cfg_;
    std::vector<Triangle> triangles_;
    Eigen::Isometry3d t_map_camera_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat map1_, map2_;
    bool init_ = false;
};

} // namespace radar_camera::projection
