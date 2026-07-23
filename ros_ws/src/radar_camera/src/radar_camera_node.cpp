#include "radar_camera/radar_camera_node.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace radar_camera::node {

RadarCameraNode::RadarCameraNode()
    : Node("radar_camera_node") {
    auto ret = ConfigsLoader(*this, camera_config_, inference_config_, projection_config_);
    if (!ret) {
        RCLCPP_ERROR(get_logger(), "ConfigsLoader failed: %s", ret.error().c_str());
        throw std::runtime_error("ConfigsLoader failed: " + ret.error());
    }
    RCLCPP_INFO(get_logger(), "ConfigsLoader succeeded");

    model_inference_ = std::make_unique<model_inference::ModelInference>();
    // auto model_ret   = model_inference_->infer_init(inference_config_);
    // if (!model_ret) {
    //     RCLCPP_ERROR(
    //         get_logger(), "ModelInference infer_init failed: %s", model_ret.error().c_str());
    //     throw std::runtime_error("ModelInference infer_init failed: " + model_ret.error());
    // }
    // RCLCPP_INFO(get_logger(), "ModelInference infer_init succeeded");

    // auto cam_ret = projector_.proj_init_camera(camera_config_);
    // if (!cam_ret) {
    //     RCLCPP_ERROR(get_logger(), "Projector camera model init failed: %s",
    //     cam_ret.error().c_str()); throw std::runtime_error("Projector camera model init failed: "
    //     + cam_ret.error());
    // }
    // RCLCPP_INFO(get_logger(), "Camera init succeeded");

    // auto map_ret = projector_.proj_init_map(projection_config_);
    // if (!map_ret) {
    //     RCLCPP_ERROR(get_logger(), "Map init failed: %s", map_ret.error().c_str());
    //     throw std::runtime_error("Map init failed: " + map_ret.error());
    // }
    // RCLCPP_INFO(get_logger(), "Map init succeeded");

    pose_publisher_ = this->create_publisher<radar_interfaces::msg::CameraDetectionPose>(
        camera_config_.pub_topic_name, 10);
    RCLCPP_INFO(get_logger(), "Publisher created");

    shm_fd_ = shm_open(camera_config_.shm_name.c_str(), O_RDWR, 0666);
    if (shm_fd_ == -1) {
        throw std::runtime_error("SHM shm_open failed: " + camera_config_.shm_name);
    }
    RCLCPP_INFO(get_logger(), "SHM shm_open succeeded: %s", camera_config_.shm_name.c_str());

    ret = infer_thread_start();
    if (!ret) {
        throw std::runtime_error("infer_thread_start failed: " + ret.error());
    }
    RCLCPP_INFO(get_logger(), "infer_thread started");
}

RadarCameraNode::~RadarCameraNode() { infer_thread_stop(); }

auto RadarCameraNode::infer_thread_start() -> std::expected<void, std::string> {
    infer_running_ = true;
    infer_thread_  = std::thread([this]() {
        while (infer_running_.load(std::memory_order_acquire)) {
            auto t_loop = std::chrono::steady_clock::now();

            cv::Mat frame;
            std::chrono::steady_clock::time_point ts;
            auto ret =
                hikcamera::SHMRead(shm_fd_, frame, ts, camera_config_.width, camera_config_.height,
                    inference_config_.model_input_width, inference_config_.model_input_height);
            if (!ret.has_value()) {
                RCLCPP_WARN(get_logger(), "SHMRead failed: %s", ret.error().c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            capture_timestamp_ = ts;

            auto t0 = std::chrono::steady_clock::now();

            cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0 / 255.0,
                cv::Size(inference_config_.model_input_width, inference_config_.model_input_height),
                cv::Scalar(), false, false);

            auto pre_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0)
                              .count();

            // ============================================================
            // FIXME: uncomment when model file is available
            // ============================================================
            // auto tensor = model_inference_->infer_preprocess(
            //     frame, inference_config_.model_input_width,
            //     inference_config_.model_input_height);
            // if (!tensor) {
            //     RCLCPP_WARN(get_logger(), "Infer preprocess failed: %s", tensor.error().c_str());
            //     continue;
            // }
            // auto async_ret = model_inference_->infer_runtime_async(tensor->get());
            // if (!async_ret) {
            //     RCLCPP_WARN(get_logger(), "Inference async failed: %s",
            //     async_ret.error().c_str()); continue;
            // }
            // auto raw = model_inference_->infer_runtime_wait();
            // if (!raw) {
            //     RCLCPP_WARN(get_logger(), "Inference wait failed: %s", raw.error().c_str());
            //     continue;
            // }
            // auto dets = model_inference_->infer_postprocess(
            //     raw->get(), frame.cols, frame.rows);
            // if (!dets) {
            //     RCLCPP_WARN(get_logger(), "Infer postprocess failed: %s", dets.error().c_str());
            //     continue;
            // }
            // auto projected = projector_.proj_preprocess(dets->get());
            // if (!projected) {
            //     RCLCPP_WARN(get_logger(), "Projection preprocess failed: %s",
            //         projected.error().c_str());
            //     continue;
            // }
            // auto pose = projector_.proj_postprocess(*projected, dets->get());
            // if (!pose) {
            //     RCLCPP_WARN(get_logger(), "Projection postprocess failed: %s",
            //         pose.error().c_str());
            //     continue;
            // }
            // PublishCallback(*pose);
            // ============================================================

            auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t_loop)
                                .count();
            RCLCPP_INFO(get_logger(), "TIMING: pre=%ldus total=%ldus (%.1ffps) frame=%dx%d",
                (long)pre_us, (long)total_us, 1e6 / total_us, frame.cols, frame.rows);
        }
    });
    return { };
}

auto RadarCameraNode::infer_thread_stop() -> void {
    infer_running_ = false;
    if (infer_thread_.joinable()) infer_thread_.join();
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_fd_ = -1;
    }
}

auto RadarCameraNode::PublishCallback(const robot_pose::RobotPose& robot_poses) -> void {
    auto pose_msg                  = radar_interfaces::msg::CameraDetectionPose();
    pose_msg.header.stamp          = rclcpp::Time(capture_timestamp_.time_since_epoch().count());
    pose_msg.header.frame_id       = "map";
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
    inference_config::InferenceConfig& inference, projection_config::ProjectionConfig& projection)
    -> std::expected<void, std::string> {
    try {
        node.declare_parameter("enemy_color", std::string("blue"));
        node.declare_parameter("hero_red", 0);
        node.declare_parameter("engineer_red", 1);
        node.declare_parameter("infantry3_red", 2);
        node.declare_parameter("infantry4_red", 3);
        node.declare_parameter("sentry_red", 4);
        node.declare_parameter("drone_red", 5);
        node.declare_parameter("hero_blue", 6);
        node.declare_parameter("engineer_blue", 7);
        node.declare_parameter("infantry3_blue", 8);
        node.declare_parameter("infantry4_blue", 9);
        node.declare_parameter("sentry_blue", 10);
        node.declare_parameter("drone_blue", 11);
        node.declare_parameter("camera_matrix", std::vector<double> { 1, 0, 0, 0, 1, 0, 0, 0, 1 });
        node.declare_parameter("distortion_coefficients", std::vector<double> { 0, 0, 0, 0, 0 });
        node.declare_parameter("rotation", std::vector<double> { 0, 0, 0 });
        node.declare_parameter("translation", std::vector<double> { 0, 0, 0 });
        node.declare_parameter("pub_topic_name", std::string("/radar_camera/robot_pose"));
        node.declare_parameter("shm_name", std::string("/hikcamera_shm"));
        node.declare_parameter("width", 5472);
        node.declare_parameter("height", 3648);
        node.declare_parameter("model_path", std::string(""));
        node.declare_parameter("device_name", std::string("CPU"));
        node.declare_parameter("conf_threshold", 0.6);
        node.declare_parameter("min_length_width_rate", 0.8);
        node.declare_parameter("max_length_width_rate", 1.5);
        node.declare_parameter("use_openvino", true);
        node.declare_parameter("num_classes", 12);
        node.declare_parameter("model_input_width", 1280);
        node.declare_parameter("model_input_height", 1280);
        node.declare_parameter("mesh_path", std::string(""));

        node.get_parameter("enemy_color", camera.enemy_color);
        node.get_parameter("hero_" + camera.enemy_color, camera.hero_class_id);
        node.get_parameter("engineer_" + camera.enemy_color, camera.engine_class_id);
        node.get_parameter("infantry3_" + camera.enemy_color, camera.infantry_3_class_id);
        node.get_parameter("infantry4_" + camera.enemy_color, camera.infantry_4_class_id);
        node.get_parameter("sentry_" + camera.enemy_color, camera.sentry_class_id);
        node.get_parameter("drone_" + camera.enemy_color, camera.drone_class_id);
        node.get_parameter("camera_matrix", camera.camera_matrix);
        node.get_parameter("distortion_coefficients", camera.distortion_coefficients);
        node.get_parameter("rotation", camera.rotation);
        node.get_parameter("translation", camera.translation);
        node.get_parameter("pub_topic_name", camera.pub_topic_name);
        node.get_parameter("shm_name", camera.shm_name);
        node.get_parameter("width", camera.width);
        node.get_parameter("height", camera.height);
        node.get_parameter("model_path", inference.model_path);
        node.get_parameter("device_name", inference.device_name);
        node.get_parameter("conf_threshold", inference.conf_threshold);
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
    return { };
}

} // namespace radar_camera::node