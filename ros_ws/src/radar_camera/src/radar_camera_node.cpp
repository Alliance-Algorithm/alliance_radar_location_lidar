#include "radar_camera/radar_camera_node.hpp"

namespace radar_camera::node {

RadarCameraNode::RadarCameraNode()
    : Node("radar_camera_node") {
    auto ret = ConfigsLoader(*this, camera_config_, inference_config_, projection_config_);
    if (!ret) {
        RCLCPP_ERROR(get_logger(), "Failed to load configuration: %s", ret.error().c_str());
    }

    model_inference_ = std::make_unique<model_inference::ModelInference>(inference_config_);

    auto proj_ret = projector_.proj_init(camera_config_, projection_config_);
    if (!proj_ret) {
        RCLCPP_WARN(get_logger(), "Projector init failed: %s", proj_ret.error().c_str());
    }

    pose_publisher_ = this->create_publisher<radar_interfaces::msg::CameraDetectionPose>(
        camera_config_.pub_topic_name, 10);
    image_subscription_ =
        this->create_subscription<sensor_msgs::msg::Image>(camera_config_.sub_topic_name, 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) { ImageCallback(msg); });
}

auto RadarCameraNode::ImageCallback(sensor_msgs::msg::Image::SharedPtr msg) -> void {
    ImageData =
        cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(&msg->data[0])).clone();
    capture_timestamp_ =
        std::chrono::steady_clock::time_point(std::chrono::seconds(msg->header.stamp.sec)
            + std::chrono::nanoseconds(msg->header.stamp.nanosec));

    auto async_ret = model_inference_->infer_runtime_async(ImageData);
    if (!async_ret) {
        RCLCPP_WARN(get_logger(), "Inference failed: %s", async_ret.error().c_str());
        return;
    }

    auto wait_ret = model_inference_->infer_runtime_wait();
    if (!wait_ret) {
        RCLCPP_WARN(get_logger(), "Inference failed: %s", wait_ret.error().c_str());
        return;
    }

    auto proj_ret = projector_.proj_process(*wait_ret, camera_config_);
    if (!proj_ret) {
        RCLCPP_WARN(get_logger(), "Projection failed: %s", proj_ret.error().c_str());
        return;
    }
    PublishCallback(*proj_ret);
}

auto RadarCameraNode::PublishCallback(const robot_pose::RobotPose& robot_poses) -> void {
    auto pose_msg                  = radar_interfaces::msg::CameraDetectionPose();
    pose_msg.header.stamp          = rclcpp::Time(capture_timestamp_.time_since_epoch().count());
    pose_msg.header.frame_id       = "camera_frame";
    pose_msg.hero_position.x       = robot_poses.hero_position.x;
    pose_msg.hero_position.y       = robot_poses.hero_position.y;
    pose_msg.engine_position.x     = robot_poses.engine_position.x;
    pose_msg.engine_position.y     = robot_poses.engine_position.y;
    pose_msg.infantry_3_position.x = robot_poses.infantry_3_position.x;
    pose_msg.infantry_3_position.y = robot_poses.infantry_3_position.y;
    pose_msg.infantry_4_position.x = robot_poses.infantry_4_position.x;
    pose_msg.infantry_4_position.y = robot_poses.infantry_4_position.y;
    pose_msg.sentry_position.x     = robot_poses.sentry_position.x;
    pose_msg.sentry_position.y     = robot_poses.sentry_position.y;
    pose_msg.drone_position.x      = robot_poses.drone_position.x;
    pose_msg.drone_position.y      = robot_poses.drone_position.y;
    pose_msg.hero_confidence       = robot_poses.hero_confidence;
    pose_msg.engine_confidence     = robot_poses.engine_confidence;
    pose_msg.infantry_3_confidence = robot_poses.infantry_3_confidence;
    pose_msg.infantry_4_confidence = robot_poses.infantry_4_confidence;
    pose_msg.sentry_confidence     = robot_poses.sentry_confidence;
    pose_msg.drone_confidence      = robot_poses.drone_confidence;
    pose_publisher_->publish(pose_msg);
}

auto ConfigsLoader(rclcpp::Node& node, camera_config::CameraConfig& camera,
    inference_config::InferenceConfig& inference,
    projection_config::ProjectionConfig& projection) -> std::expected<void, std::string> {
    try {
        node.get_parameter("enemy_color", camera.enemy_color);
        node.get_parameter("hero_" + camera.enemy_color, camera.hero_class_id);
        node.get_parameter("engine_" + camera.enemy_color, camera.engine_class_id);
        node.get_parameter("infantry_3_" + camera.enemy_color, camera.infantry_3_class_id);
        node.get_parameter("infantry_4_" + camera.enemy_color, camera.infantry_4_class_id);
        node.get_parameter("sentry_" + camera.enemy_color, camera.sentry_class_id);
        node.get_parameter("camera_matrix", camera.camera_matrix);
        node.get_parameter("distortion_coefficients", camera.distortion_coefficients);
        node.get_parameter("rotation", camera.rotation);
        node.get_parameter("translation", camera.translation);
        node.get_parameter("pub_topic_name", camera.pub_topic_name);
        node.get_parameter("sub_topic_name", camera.sub_topic_name);

        node.get_parameter("model_path", inference.model_path);
        node.get_parameter("conf_threshold", inference.conf_threshold);
        node.get_parameter("nms_threshold", inference.nms_threshold);
        node.get_parameter("min_length_width_rate", inference.min_length_width_rate);
        node.get_parameter("max_length_width_rate", inference.max_length_width_rate);
        node.get_parameter("use_openvino", inference.use_openvino);
        node.get_parameter("num_classes", inference.num_classes);
        node.get_parameter("model_input_width", inference.model_input_width);
        node.get_parameter("model_input_height", inference.model_input_height);

        node.get_parameter("mesh_path", projection.mesh_path);
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Error loading configuration: ") + e.what());
    }
    return {};
}

} // namespace radar_camera::node
