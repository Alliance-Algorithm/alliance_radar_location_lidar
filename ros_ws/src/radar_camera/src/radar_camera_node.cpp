#include "radar_camera/radar_camera_node.hpp"
#include <chrono>
namespace radar_camera {
RadarCameraNode::RadarCameraNode()
    : Node("radar_camera_node") {
    if (auto result = ConfigsLoader(*this, config_); !result) {
        RCLCPP_ERROR(get_logger(), "Failed to load configuration: %s", result.error().c_str());
    }
    pose_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(config_.topic_name, 10);
    image_subscription_ =
        this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10,

            [this](sensor_msgs::msg::Image::SharedPtr msg) { ImageCallback(msg); });
}
auto RadarCameraNode::ConfigsLoader(rclcpp::Node& node, camera_inference_config& config)
    -> std::expected<void, std::string> {
    try {
        node.get_parameter("enemy_color", config.enemy_color);
        node.get_parameter("hero_" + config.enemy_color, config.hero_class_id);
        node.get_parameter("engine_" + config.enemy_color, config.engine_class_id);
        node.get_parameter("infantry_3_" + config.enemy_color, config.infantry_3_class_id);
        node.get_parameter("infantry_4_" + config.enemy_color, config.infantry_4_class_id);
        node.get_parameter("sentry_" + config.enemy_color, config.sentry_class_id);
        node.get_parameter("model_path", config.model_path);
        node.get_parameter("camera_matrix", config.camera_matrix);
        node.get_parameter("distortion_coefficients", config.distortion_coefficients);
        node.get_parameter("rotation", config.rotation);
        node.get_parameter("translation", config.translation);
        node.get_parameter("topic_name", config.topic_name);
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Error loading configuration: ") + e.what());
    }
    return { };
}
auto RadarCameraNode::ImageCallback(sensor_msgs::msg::Image::SharedPtr msg) -> void {
    ImageData =
        cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(&msg->data[0])).clone();
    capture_timestamp_ =
        std::chrono::steady_clock::time_point(std::chrono::seconds(msg->header.stamp.sec)
            + std::chrono::nanoseconds(msg->header.stamp.nanosec));
}
auto RadarCameraNode::PublishCallback(const std::vector<Eigen::Vector2d>& robot_poses) -> void {

    pose_publisher_->publish(pose_msg);
}

} // namespace radar_camera