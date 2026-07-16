#pragma once
#include <chrono>
#include <expected>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

#include "radar_camera/data_format.hpp"
#include "radar_camera/model_inference.hpp"
#include "radar_camera/projector.hpp"
#include "radar_interfaces/msg/camera_detection_pose.hpp"

namespace radar_camera::node {

auto ConfigsLoader(rclcpp::Node& node, camera_config::CameraConfig& camera,
    inference_config::InferenceConfig& inference, projection_config::ProjectionConfig& projection)
    -> std::expected<void, std::string>;

class RadarCameraNode final : public rclcpp::Node {
public:
    RadarCameraNode();
    ~RadarCameraNode() = default;

    auto ImageCallback(sensor_msgs::msg::Image::SharedPtr msg) -> void;
    auto PublishCallback(const robot_pose::RobotPose& robot_poses) -> void;

    cv::Mat ImageData;
    std::chrono::steady_clock::time_point capture_timestamp_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<radar_interfaces::msg::CameraDetectionPose>::SharedPtr pose_publisher_;

    camera_config::CameraConfig camera_config_;
    inference_config::InferenceConfig inference_config_;
    projection_config::ProjectionConfig projection_config_;
    robot_pose::RobotPose robot_poses_;
    std::unique_ptr<model_inference::ModelInference> model_inference_;
    projection::Projector projector_;
};

} // namespace radar_camera::node
