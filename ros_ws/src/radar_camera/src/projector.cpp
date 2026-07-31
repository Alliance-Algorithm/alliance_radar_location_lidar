#include "radar_camera/projector.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <cmath>
#include <filesystem>

namespace radar_camera::projection {

auto Projector::proj_init_camera(const camera_config::CameraConfig& camera_cfg)
    -> std::expected<void, std::string> {
    camera_cfg_ = camera_cfg;
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

    double roll  = camera_cfg.rotation[0];
    double pitch = camera_cfg.rotation[1];
    double yaw   = camera_cfg.rotation[2];

    Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();

    Eigen::Vector3d t(
        camera_cfg.translation[0], camera_cfg.translation[1], camera_cfg.translation[2]);

    t_map_camera_.linear()      = R;
    t_map_camera_.translation() = t;

    init_ = true;
    return { };
}

auto Projector::proj_init_map(const projection_config::ProjectionConfig& proj_cfg)
    -> std::expected<void, std::string> {
    if (proj_cfg.mesh_path.empty()) {
        return { };
    }
    if (!std::filesystem::exists(proj_cfg.mesh_path)) {
        return std::unexpected("Mesh file not found: " + proj_cfg.mesh_path);
    }

    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(proj_cfg.mesh_path,
        aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_GenNormals
            | aiProcess_OptimizeMeshes);

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        return std::unexpected("Assimp error: " + std::string(importer.GetErrorString()));
    }

    triangles_.clear();

    std::function<void(aiNode*, const aiScene*)> collect;
    collect = [this, &collect](aiNode* node, const aiScene* scene) {
        for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
            aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
                const aiFace& face = mesh->mFaces[f];
                if (face.mNumIndices != 3) continue;

                Eigen::Vector3f v0(mesh->mVertices[face.mIndices[0]].x,
                    mesh->mVertices[face.mIndices[0]].y, mesh->mVertices[face.mIndices[0]].z);
                Eigen::Vector3f v1(mesh->mVertices[face.mIndices[1]].x,
                    mesh->mVertices[face.mIndices[1]].y, mesh->mVertices[face.mIndices[1]].z);
                Eigen::Vector3f v2(mesh->mVertices[face.mIndices[2]].x,
                    mesh->mVertices[face.mIndices[2]].y, mesh->mVertices[face.mIndices[2]].z);
                triangles_.push_back({ v0, v1, v2 });
            }
        }
        for (unsigned int i = 0; i < node->mNumChildren; ++i) {
            collect(node->mChildren[i], scene);
        }
    };

    collect(scene->mRootNode, scene);

    if (triangles_.empty()) {
        return std::unexpected("No triangles found in mesh");
    }
    return { };
}

auto Projector::proj_pixel_to_ray(const cv::Point2d& pixel) -> std::expected<Ray, std::string> {
    if (!init_) {
        return std::unexpected("Projector not initialized");
    }

    undistort_src_.clear();
    undistort_src_.push_back(cv::Point2f(static_cast<float>(pixel.x), static_cast<float>(pixel.y)));
    cv::undistortPoints(
        undistort_src_, undistort_dst_, camera_matrix_, dist_coeffs_, cv::noArray(), cv::noArray());

    if (undistort_dst_.empty()) {
        return std::unexpected("undistortPoints failed");
    }

    double x_norm = undistort_dst_[0].x;
    double y_norm = undistort_dst_[0].y;

    Eigen::Vector3d dir_cam(x_norm, y_norm, 1.0);
    dir_cam.normalize();

    Eigen::Vector3f origin    = t_map_camera_.translation().cast<float>();
    Eigen::Vector3f dir_world = (t_map_camera_.rotation() * dir_cam).cast<float>();

    return Ray { origin, dir_world };
}

auto Projector::proj_map_intersect(const Ray& ray) -> std::expected<cv::Point2d, std::string> {
    constexpr float kEpsilon = 1e-8f;
    float nearest            = std::numeric_limits<float>::max();
    std::optional<Eigen::Vector3f> hit_point;

    for (const auto& tri : triangles_) {
        Eigen::Vector3f e1 = tri.v1 - tri.v0;
        Eigen::Vector3f e2 = tri.v2 - tri.v0;

        Eigen::Vector3f pvec = ray.direction.cross(e2);
        float det            = e1.dot(pvec);
        if (std::fabs(det) < kEpsilon) continue;

        float inv_det        = 1.0f / det;
        Eigen::Vector3f tvec = ray.origin - tri.v0;
        float u              = tvec.dot(pvec) * inv_det;
        if (u < 0.0f || u > 1.0f) continue;

        Eigen::Vector3f qvec = tvec.cross(e1);
        float v              = ray.direction.dot(qvec) * inv_det;
        if (v < 0.0f || u + v > 1.0f) continue;

        float t = (tri.v2 - tri.v0).dot(qvec) * inv_det;
        if (t < kEpsilon || t >= nearest) continue;

        nearest   = t;
        hit_point = ray.origin + ray.direction * t;
    }

    if (!hit_point) {
        return std::unexpected("Ray did not hit any triangle");
    }
    return cv::Point2d(hit_point->x(), hit_point->y());
}

auto Projector::proj_preprocess(const std::vector<detection::Detection>& detections)
    -> std::expected<std::vector<std::optional<cv::Point2d>>, std::string> {
    std::vector<std::optional<cv::Point2d>> projected;
    projected.reserve(detections.size());
    for (const auto& det : detections) {
        auto ray = proj_pixel_to_ray(det.center);
        if (!ray) {
            projected.push_back(std::nullopt);
            continue;
        }
        auto pt = proj_map_intersect(*ray);
        if (!pt) {
            projected.push_back(std::nullopt);
            continue;
        }
        projected.push_back(*pt);
    }
    return projected;
}

auto Projector::proj_postprocess(const std::vector<std::optional<cv::Point2d>>& projected,
    const std::vector<detection::Detection>& detections)
    -> std::expected<robot_pose::RobotPose, std::string> {
    robot_pose::RobotPose pose { };
    for (size_t i = 0; i < detections.size(); ++i) {
        if (!projected[i]) continue;
        const auto& det = detections[i];
        if (det.id == camera_cfg_.hero_class_id) {
            pose.hero_position   = *projected[i];
            pose.hero_confidence = det.confidence;
        } else if (det.id == camera_cfg_.engine_class_id) {
            pose.engine_position   = *projected[i];
            pose.engine_confidence = det.confidence;
        } else if (det.id == camera_cfg_.infantry_3_class_id) {
            pose.infantry_3_position   = *projected[i];
            pose.infantry_3_confidence = det.confidence;
        } else if (det.id == camera_cfg_.infantry_4_class_id) {
            pose.infantry_4_position   = *projected[i];
            pose.infantry_4_confidence = det.confidence;
        } else if (det.id == camera_cfg_.sentry_class_id) {
            pose.sentry_position   = *projected[i];
            pose.sentry_confidence = det.confidence;
        } else if (det.id == camera_cfg_.drone_class_id) {
            pose.drone_position   = *projected[i];
            pose.drone_confidence = det.confidence;
        }
    }
    return pose;
}

} // namespace radar_camera::projection
