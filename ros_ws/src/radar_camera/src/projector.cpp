#include "radar_camera/projector.hpp"
#include <cmath>

namespace radar_camera::projection {

auto Projector::proj_init(const camera_config::CameraConfig& camera_cfg,
    const projection_config::ProjectionConfig& proj_cfg) -> std::expected<void, std::string> {
    if (camera_cfg.camera_matrix.size() != 9) {
        return std::unexpected("camera_matrix must have 9 elements (3x3 row-major)");
    }
    if (camera_cfg.distortion_coefficients.size() != 5) {
        return std::unexpected("distortion_coefficients must have 5 elements");
    }
    if (camera_cfg.rotation.size() != 3) {
        return std::unexpected("rotation must have 3 elements (roll, pitch, yaw)");
    }
    if (camera_cfg.translation.size() != 3) {
        return std::unexpected("translation must have 3 elements");
    }

    camera_matrix_ = cv::Mat_<double>(3, 3);
    for (int i = 0; i < 9; ++i) {
        camera_matrix_.at<double>(i / 3, i % 3) = camera_cfg.camera_matrix[i];
    }

    dist_coeffs_ = cv::Mat_<double>(1, 5);
    for (int i = 0; i < 5; ++i) {
        dist_coeffs_.at<double>(i) = camera_cfg.distortion_coefficients[i];
    }

    cv::Size image_size(static_cast<int>(camera_cfg.camera_matrix[0] * 2),
                        static_cast<int>(camera_cfg.camera_matrix[4] * 2));
    cv::initUndistortRectifyMap(camera_matrix_, dist_coeffs_, cv::Mat_<double>::eye(3, 3),
        camera_matrix_, image_size, CV_32FC1, map1_, map2_);

    double roll  = camera_cfg.rotation[0];
    double pitch = camera_cfg.rotation[1];
    double yaw   = camera_cfg.rotation[2];

    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();

    Eigen::Vector3d t(camera_cfg.translation[0],
                      camera_cfg.translation[1],
                      camera_cfg.translation[2]);

    t_map_camera_.linear() = R;
    t_map_camera_.translation() = t;

    if (!proj_cfg.mesh_path.empty()) {
        auto ret = map_.map_init(proj_cfg.mesh_path);
        if (!ret) {
            return std::unexpected(ret.error());
        }
    }

    init_ = true;
    return {};
}

auto Projector::proj_runtime(const cv::Point2d& pixel)
    -> std::expected<Eigen::Vector3d, std::string> {
    if (!init_) {
        return std::unexpected("Projector not initialized");
    }

    std::vector<cv::Point2f> src{ pixel };
    std::vector<cv::Point2f> dst;
    cv::undistortPoints(src, dst, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);

    if (dst.empty()) {
        return std::unexpected("undistortPoints failed");
    }

    double x_norm = dst[0].x;
    double y_norm = dst[0].y;

    Eigen::Vector3d dir_cam(x_norm, y_norm, 1.0);
    dir_cam.normalize();

    Eigen::Vector3d origin = t_map_camera_.translation();
    Eigen::Vector3d dir_world = t_map_camera_.rotation() * dir_cam;

    auto hit = map_.map_intersect(origin.cast<float>(), dir_world.cast<float>());
    if (!hit) {
        return std::unexpected("Ray did not hit any triangle");
    }

    return hit->cast<double>();
}

} // namespace radar_camera::projection
