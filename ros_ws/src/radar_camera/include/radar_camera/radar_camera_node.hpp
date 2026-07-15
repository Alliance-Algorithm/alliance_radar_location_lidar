#pragma once
#include "radar_interfaces/msg/camera_detection_pose.hpp"
#include <chrono>
#include <expected>
#include <map>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <utility>
#include <vector>
namespace radar_camera {

typedef struct CameraConfig {
    std::string enemy_color;
    int hero_class_id;
    int engine_class_id;
    int infantry_3_class_id;
    int infantry_4_class_id;
    int sentry_class_id;
    std::string model_path;
    std::vector<double> camera_matrix;
    std::vector<double> distortion_coefficients;
    std::vector<double> rotation;
    std::vector<double> translation;
    std::string topic_name;
} camera_inference_config;
typedef struct RobotPose {
    cv::Point hero_position;
    double hero_confidence;
    cv::Point engine_position;
    double engine_confidence;
    cv::Point infantry_3_position;
    double infantry_3_confidence;
    cv::Point infantry_4_position;
    double infantry_4_confidence;
    cv::Point sentry_position;
    double sentry_confidence;
    cv::Point drone_position;
    double drone_confidence;
} robot_pose;
class RadarCameraNode final : public rclcpp::Node {
public:
    RadarCameraNode();
    ~RadarCameraNode() = default;
    auto ConfigsLoader(rclcpp::Node& node, camera_inference_config& config)
        -> std::expected<void, std::string>;
    auto ImageCallback(sensor_msgs::msg::Image::SharedPtr msg) -> void;
    auto PublishCallback(const robot_pose& robot_poses) -> void;
    cv::Mat ImageData;
    std::chrono::steady_clock::time_point capture_timestamp_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<radar_interfaces::msg::CameraDetectionPose>::SharedPtr pose_publisher_;
    camera_inference_config config_;
};

} // namespace radar_camera
