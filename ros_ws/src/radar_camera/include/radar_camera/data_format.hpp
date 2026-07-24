#pragma once
#include <cstdint>
#include <opencv2/core/types.hpp>
#include <string>
#include <vector>

namespace radar_camera::detection {

struct Detection {
    cv::Point2d center;
    int id;
    float confidence;
};

} // namespace radar_camera::detection

namespace radar_camera::camera_config {

struct CameraConfig {
    std::string enemy_color;
    int hero_class_id;
    int engine_class_id;
    int infantry_3_class_id;
    int infantry_4_class_id;
    int sentry_class_id;
    int drone_class_id;
    std::string pub_topic_name;
    std::string shm_name;
    int width  = 1920;
    int height = 1080;
    std::vector<double> camera_matrix;
    std::vector<double> distortion_coefficients;
    std::vector<double> rotation;
    std::vector<double> translation;
};

} // namespace radar_camera::camera_config

namespace radar_camera::inference_config {

struct InferenceConfig {
    std::string model_path;
    std::string device_name           = "CPU";
    int model_input_width             = 1280;
    int model_input_height            = 1280;
    int num_classes                   = 12;
    float conf_threshold              = 0.6f;
    float min_length_width_rate       = 0.5f;
    float max_length_width_rate       = 3.0f;
    float drone_min_length_width_rate = 2.0f;
    float drone_max_length_width_rate = 10.0f;
    std::vector<std::int64_t> drone_class_ids { 5, 11 };
    bool use_openvino = true;
};

} // namespace radar_camera::inference_config

namespace radar_camera::projection_config {

struct ProjectionConfig {
    std::string mesh_path;
    std::string device_name = "CPU";
};

} // namespace radar_camera::projection_config

namespace radar_camera::robot_pose {

struct RobotPose {
    cv::Point2d hero_position;
    double hero_confidence;
    cv::Point2d engine_position;
    double engine_confidence;
    cv::Point2d infantry_3_position;
    double infantry_3_confidence;
    cv::Point2d infantry_4_position;
    double infantry_4_confidence;
    cv::Point2d sentry_position;
    double sentry_confidence;
    cv::Point2d drone_position;
    double drone_confidence;
};

} // namespace radar_camera::robot_pose
