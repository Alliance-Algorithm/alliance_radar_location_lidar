#pragma once
#include "radar_interfaces/msg/camera_detection_pose.hpp"
#include <Eigen/Dense>
#include <chrono>
#include <expected>
#include <map>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <utility>
namespace radar_camera {

typedef struct CameraConfig {
    std::string enemy_color;
    int hero_class_id;
    int engine_class_id;
    int infantry_3_class_id;
    int infantry_4_class_id;
    int sentry_class_id;
    std::string model_path;
    Eigen::Matrix3d camera_matrix;
    Eigen::Vector<double, 5> distortion_coefficients;
    Eigen::Vector3d rotation;
    Eigen::Vector3d translation;
    std::string topic_name;
} camera_inference_config;

class RadarCameraNode final : public rclcpp::Node {
public:
    RadarCameraNode();
    ~RadarCameraNode() = default;
    auto ConfigsLoader(rclcpp::Node& node, camera_inference_config& config)
        -> std::expected<void, std::string>;
    auto ImageCallback(sensor_msgs::msg::Image::SharedPtr msg) -> void;
    auto PublishCallback(const std::vector<Eigen::Vector2d>& robot_poses) -> void;

private:
    cv::Mat ImageData;
    std::chrono::steady_clock::time_point capture_timestamp_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pose_publisher_;
    camera_inference_config config_;
};

} // namespace radar_camera
