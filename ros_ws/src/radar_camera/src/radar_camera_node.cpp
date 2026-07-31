#include "radar_camera/radar_camera_node.hpp"

#include <algorithm>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>
#include <mutex>
#include <scope>
#include <stdexcept>
#include <utility>

namespace radar_camera::node {

namespace {
    struct FdGuard {
        int fd = -1;
        ~FdGuard() {
            if (fd != -1) close(fd);
        }
    };
} // namespace

RadarCameraNode::RadarCameraNode()
    : Node("radar_camera_node") {
    auto cleanup_guard = std::scope_exit([this]() noexcept { constructor_cleanup(); });
    auto ret           = ConfigsLoader(*this, camera_config_, inference_config_, projection_config_,
        recording_config_, armor_refine_config_, number_refine_config_);
    if (!ret) {
        RCLCPP_ERROR(get_logger(), "ConfigsLoader failed: %s", ret.error().c_str());
        throw std::runtime_error("ConfigsLoader failed: " + ret.error());
    }
    RCLCPP_INFO(get_logger(), "ConfigsLoader succeeded");

    model_inference_ = std::make_unique<model_inference::ModelInference>();
    auto model_ret   = model_inference_->infer_init(inference_config_);
    if (!model_ret) {
        RCLCPP_ERROR(get_logger(), "ModelInference init failed: %s", model_ret.error().c_str());
        throw std::runtime_error("ModelInference init failed: " + model_ret.error());
    }
    RCLCPP_INFO(get_logger(), "ModelInference initialized: backend=%s model=%s",
        inference_config_.backend.c_str(), inference_config_.model_path.c_str());

    if (!armor_refine_config_.armor_model_path.empty()
        && !number_refine_config_.number_model_path.empty()) {
        auto refine_ret = armor_refiner_.init(armor_refine_config_, number_refine_config_);
        if (!refine_ret) {
            RCLCPP_WARN(get_logger(), "ArmorRefiner init skipped: %s", refine_ret.error().c_str());
        } else {
            armor_refine_enabled_ = true;
            RCLCPP_INFO(get_logger(), "ArmorRefiner initialized: L2=%s L3=%s",
                armor_refine_config_.armor_model_path.c_str(),
                number_refine_config_.number_model_path.c_str());
        }
    } else {
        RCLCPP_INFO(get_logger(), "ArmorRefiner disabled: L2/L3 model paths not set");
    }

    auto cam_ret = projector_.proj_init_camera(camera_config_);
    if (!cam_ret)
        RCLCPP_WARN(get_logger(), "Projector camera init skipped: %s", cam_ret.error().c_str());

    auto map_ret = projector_.proj_init_map(projection_config_);
    if (!map_ret)
        RCLCPP_WARN(get_logger(), "Projector map init skipped: %s", map_ret.error().c_str());

    pose_publisher_ = this->create_publisher<radar_interfaces::msg::CameraDetectionPose>(
        camera_config_.pub_topic_name, 10);
    RCLCPP_INFO(get_logger(), "Publisher created");

    FdGuard shm_guard { shm_open(camera_config_.shm_name.c_str(), O_RDWR, 0666) };
    if (shm_guard.fd == -1) {
        throw std::runtime_error("SHM shm_open failed: " + camera_config_.shm_name);
    }
    shm_fd_      = shm_guard.fd;
    shm_guard.fd = -1;
    RCLCPP_INFO(get_logger(), "SHM shm_open succeeded: %s", camera_config_.shm_name.c_str());

    ret = infer_thread_start();
    if (!ret) {
        throw std::runtime_error("infer_thread_start failed: " + ret.error());
    }
    RCLCPP_INFO(get_logger(), "infer_thread started");

    if (recording_config_.enabled) {
        auto components     = make_recording_components(recording_config_, camera_config_.shm_name);
        recording_fifo_     = std::move(components.fifo);
        raw_video_recorder_ = std::move(components.recorder);
        raw_shm_reader_     = std::move(components.reader);

        for (const auto component : recording_lifecycle_order()) {
            switch (component) {
            case LifecycleComponent::recorder: {
                auto recorder_ret = raw_video_recorder_->start();
                if (!recorder_ret) {
                    throw std::runtime_error(
                        "RawVideoRecorder start failed: " + recorder_ret.error());
                }
                break;
            }
            case LifecycleComponent::reader: {
                auto reader_ret = raw_shm_reader_->start();
                if (!reader_ret) {
                    throw std::runtime_error("RawShmReader start failed: " + reader_ret.error());
                }
                break;
            }
            case LifecycleComponent::monitor:
                recording_monitor_start();
                break;
            case LifecycleComponent::inference:
            case LifecycleComponent::shm:
                break;
            }
        }
        RCLCPP_INFO(
            get_logger(), "Raw recording started: %s", recording_config_.output_dir.c_str());
    }

    status_.store(NodeStatus::running, std::memory_order_release);
    cleanup_guard.release();
}

RadarCameraNode::~RadarCameraNode() {
    recording_monitor_stop();
    if (raw_shm_reader_) raw_shm_reader_->stop();
    if (raw_video_recorder_) raw_video_recorder_->stop();
    infer_thread_stop();
    status_.store(NodeStatus::stopped, std::memory_order_release);
}

auto RadarCameraNode::recording_monitor_start() -> void {
    recording_monitor_running_ = true;
    recording_monitor_thread_  = std::thread([this]() {
        while (recording_monitor_running_.load(std::memory_order_acquire)) {
            const auto recorder_state = raw_video_recorder_->state();
            const auto reader_state   = raw_shm_reader_->state();
            if (recorder_state == recording::RecorderState::failed
                || recorder_state == recording::RecorderState::overrun
                || reader_state == recording::ReaderState::failed
                || reader_state == recording::ReaderState::overrun) {
                const auto recorder_failed = recorder_state == recording::RecorderState::failed
                    || recorder_state == recording::RecorderState::overrun;
                const auto reason = recorder_failed ? raw_video_recorder_->failure_reason()
                                                    : raw_shm_reader_->failure_reason();
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Raw recording failed (state=%s): %s; stopping camera inference",
                    recorder_state == recording::RecorderState::overrun
                            || reader_state == recording::ReaderState::overrun
                        ? "OVERRUN"
                        : "FAILED",
                    reason.c_str());
                infer_running_     = false;
                const auto cleanup = constructor_cleanup_order(
                    { LifecycleComponent::recorder, LifecycleComponent::reader });
                for (const auto component : cleanup) {
                    if (component == LifecycleComponent::reader && raw_shm_reader_) {
                        raw_shm_reader_->stop();
                    } else if (component == LifecycleComponent::recorder && raw_video_recorder_) {
                        raw_video_recorder_->stop();
                    }
                }
                {
                    std::lock_guard lock(status_mutex_);
                    failure_reason_ = reason;
                }
                status_.store(NodeStatus::failed, std::memory_order_release);
                rclcpp::shutdown();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}

auto RadarCameraNode::status() const -> NodeStatus {
    return status_.load(std::memory_order_acquire);
}

auto RadarCameraNode::failure_reason() const -> std::string {
    std::lock_guard lock(status_mutex_);
    return failure_reason_;
}

auto recording_lifecycle_order() -> std::vector<LifecycleComponent> {
    return { LifecycleComponent::inference, LifecycleComponent::recorder,
        LifecycleComponent::reader, LifecycleComponent::monitor };
}

auto constructor_cleanup_order(const std::vector<LifecycleComponent>& started)
    -> std::vector<LifecycleComponent> {
    std::vector<LifecycleComponent> cleanup;
    if (std::find(started.begin(), started.end(), LifecycleComponent::monitor) != started.end()) {
        cleanup.emplace_back(LifecycleComponent::monitor);
    }
    for (auto it = started.rbegin(); it != started.rend(); ++it) {
        if (*it == LifecycleComponent::reader || *it == LifecycleComponent::recorder
            || *it == LifecycleComponent::inference) {
            cleanup.push_back(*it);
        }
    }
    if (std::find(started.begin(), started.end(), LifecycleComponent::shm) != started.end()) {
        cleanup.emplace_back(LifecycleComponent::shm);
    }
    return cleanup;
}

auto make_recording_components(
    const recording::RecordingConfig& config, const std::string& shm_name) -> RecordingComponents {
    if (!config.enabled) return { };
    auto fifo     = std::make_unique<recording::RecordingFifo>(config.buffer_pool_frames);
    auto recorder = std::make_unique<recording::RawVideoRecorder>(config, *fifo);
    auto reader =
        std::make_unique<recording::RawShmReader>(shm_name, config.width, config.height, *fifo);
    return { std::move(fifo), std::move(recorder), std::move(reader) };
}

auto RadarCameraNode::constructor_cleanup() noexcept -> void {
    std::vector<LifecycleComponent> started;
    if (shm_fd_ != -1) started.emplace_back(LifecycleComponent::shm);
    if (infer_thread_.joinable() || infer_running_)
        started.emplace_back(LifecycleComponent::inference);
    if (raw_video_recorder_) started.emplace_back(LifecycleComponent::recorder);
    if (raw_shm_reader_) started.emplace_back(LifecycleComponent::reader);
    if (recording_monitor_thread_.joinable() || recording_monitor_running_)
        started.emplace_back(LifecycleComponent::monitor);

    for (const auto component : constructor_cleanup_order(started)) {
        switch (component) {
        case LifecycleComponent::monitor:
            recording_monitor_stop();
            break;
        case LifecycleComponent::reader:
            if (raw_shm_reader_) raw_shm_reader_->stop();
            break;
        case LifecycleComponent::recorder:
            if (raw_video_recorder_) raw_video_recorder_->stop();
            break;
        case LifecycleComponent::inference:
            infer_thread_stop();
            break;
        case LifecycleComponent::shm:
            if (shm_fd_ != -1) {
                close(shm_fd_);
                shm_fd_ = -1;
            }
            break;
        }
    }
}

auto RadarCameraNode::recording_monitor_stop() -> void {
    recording_monitor_running_ = false;
    if (recording_monitor_thread_.joinable()) recording_monitor_thread_.join();
}

auto RadarCameraNode::infer_thread_start() -> std::expected<void, std::string> {
    infer_running_ = true;
    infer_thread_  = std::thread([this]() {
        while (infer_running_.load(std::memory_order_acquire)) {
            cv::Mat orig_frame;
            std::chrono::steady_clock::time_point ts;
            // Read at full sensor resolution (dst_w=0 → no internal resize).
            auto ret = hikcamera::SHMRead(shm_fd_, orig_frame, ts,
                camera_config_.width, camera_config_.height, 0, 0);
            if (!ret.has_value()) {
                RCLCPP_WARN(get_logger(), "SHMRead failed: %s", ret.error().c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            capture_timestamp_ = ts;

            // Resize to L1 model input separately; L2/L3 will crop from orig_frame.
            cv::Mat frame;
            cv::resize(orig_frame, frame,
                cv::Size(inference_config_.model_input_width,
                    inference_config_.model_input_height));

            auto tensor = model_inference_->infer_preprocess(frame,
                static_cast<size_t>(inference_config_.model_input_width),
                static_cast<size_t>(inference_config_.model_input_height));
            if (!tensor) {
                RCLCPP_WARN(
                    get_logger(), "Inference preprocess failed: %s", tensor.error().c_str());
                continue;
            }

            auto async_ret = model_inference_->infer_runtime_async();
            if (!async_ret) {
                RCLCPP_WARN(get_logger(), "Inference start failed: %s", async_ret.error().c_str());
                continue;
            }
            auto raw = model_inference_->infer_runtime_wait();
            if (!raw) {
                RCLCPP_WARN(get_logger(), "Inference wait failed: %s", raw.error().c_str());
                continue;
            }

            auto dets = model_inference_->infer_postprocess(raw->get(), frame.cols, frame.rows);
            if (!dets) {
                RCLCPP_WARN(get_logger(), "Inference postprocess failed: %s", dets.error().c_str());
                continue;
            }

            std::vector<detection::Detection> refined(dets->get());
            if (armor_refine_enabled_) {
                const float scale_x =
                    static_cast<float>(orig_frame.cols) / static_cast<float>(frame.cols);
                const float scale_y =
                    static_cast<float>(orig_frame.rows) / static_cast<float>(frame.rows);
                for (auto& det : refined) {
                    armor_refiner_.refine(
                        orig_frame, det, inference_config_.drone_class_ids, scale_x, scale_y);
                }
            }

            auto projected = projector_.proj_preprocess(refined);
            auto pose = std::expected<robot_pose::RobotPose, std::string>(std::unexpected("projecti"
                                                                                          "on "
                                                                                          "preproce"
                                                                                          "ss "
                                                                                          "faile"
                                                                                          "d"));
            if (!projected) {
                RCLCPP_WARN(
                    get_logger(), "Projection preprocess failed: %s", projected.error().c_str());
            } else {
                pose = projector_.proj_postprocess(*projected, refined);
                if (!pose) {
                    RCLCPP_WARN(
                        get_logger(), "Projection postprocess failed: %s", pose.error().c_str());
                }
            }

            if (pose) PublishCallback(*pose);
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
    inference_config::InferenceConfig& inference, projection_config::ProjectionConfig& projection,
    recording::RecordingConfig& recording, armor_refine::ArmorRefineConfig& armor,
    armor_refine::NumberRefineConfig& number) -> std::expected<void, std::string> {
    try {
        node.declare_parameter("enemy_color", std::string("blue"));
        node.declare_parameter("hero_blue", 6);
        node.declare_parameter("engineer_blue", 7);
        node.declare_parameter("infantry3_blue", 8);
        node.declare_parameter("infantry4_blue", 9);
        node.declare_parameter("sentry_blue", 10);
        node.declare_parameter("drone_blue", 11);
        node.declare_parameter("hero_red", 0);
        node.declare_parameter("engineer_red", 1);
        node.declare_parameter("infantry3_red", 2);
        node.declare_parameter("infantry4_red", 3);
        node.declare_parameter("sentry_red", 4);
        node.declare_parameter("drone_red", 5);
        node.declare_parameter("camera_matrix", std::vector<double> { 1, 0, 0, 0, 1, 0, 0, 0, 1 });
        node.declare_parameter("distortion_coefficients", std::vector<double> { 0, 0, 0, 0, 0 });
        node.declare_parameter("rotation", std::vector<double> { 0, 0, 0 });
        node.declare_parameter("translation", std::vector<double> { 0, 0, 0 });
        node.declare_parameter("pub_topic_name", std::string("/radar_camera/robot_pose"));
        node.declare_parameter("shm_name", std::string("/hikcamera_shm"));
        node.declare_parameter("width", 5472);
        node.declare_parameter("height", 3648);
        node.declare_parameter("model_path", std::string(""));
        node.declare_parameter("backend", std::string("openvino"));
        node.declare_parameter("device_name", std::string("CPU"));
        node.declare_parameter("conf_threshold", 0.3);
        node.declare_parameter("min_length_width_rate", 0.8);
        node.declare_parameter("max_length_width_rate", 1.5);
        node.declare_parameter("drone_min_length_width_rate", 2.0);
        node.declare_parameter("drone_max_length_width_rate", 10.0);
        node.declare_parameter("drone_class_ids", std::vector<std::int64_t> { 5, 11 });
        node.declare_parameter("use_openvino", true);
        node.declare_parameter("num_classes", 12);
        node.declare_parameter("model_input_width", 1280);
        node.declare_parameter("model_input_height", 1280);
        node.declare_parameter("mesh_path", std::string(""));
        node.declare_parameter("enable_raw_recording", false);
        node.declare_parameter("recording_output_dir", std::string("/model/devio"));
        node.declare_parameter("recording_width", 5472);
        node.declare_parameter("recording_height", 3648);
        node.declare_parameter("recording_fps", 20);
        node.declare_parameter("recording_bitrate", 40000000);
        node.declare_parameter("recording_gop", 20);
        node.declare_parameter("recording_encoder", std::string("h264_nvenc"));
        node.declare_parameter("recording_segment_duration_sec", 60);
        node.declare_parameter("recording_buffer_pool_frames", 8);
        node.declare_parameter("recording_max_buffer_bytes", 480000000);
        node.declare_parameter("armor_model_path", std::string(""));
        node.declare_parameter("armor_score_threshold", 0.8);
        node.declare_parameter("armor_nms_threshold", 0.3);
        node.declare_parameter("number_model_path", std::string(""));
        node.declare_parameter("number_conf_threshold", 0.8);

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
        node.get_parameter("backend", inference.backend);
        node.get_parameter("device_name", inference.device_name);
        node.get_parameter("conf_threshold", inference.conf_threshold);
        node.get_parameter("min_length_width_rate", inference.min_length_width_rate);
        node.get_parameter("max_length_width_rate", inference.max_length_width_rate);
        node.get_parameter("drone_min_length_width_rate", inference.drone_min_length_width_rate);
        node.get_parameter("drone_max_length_width_rate", inference.drone_max_length_width_rate);
        node.get_parameter("drone_class_ids", inference.drone_class_ids);
        node.get_parameter("use_openvino", inference.use_openvino);
        node.get_parameter("num_classes", inference.num_classes);
        node.get_parameter("model_input_width", inference.model_input_width);
        node.get_parameter("model_input_height", inference.model_input_height);

        node.get_parameter("mesh_path", projection.mesh_path);
        node.get_parameter("enable_raw_recording", recording.enabled);
        node.get_parameter("recording_output_dir", recording.output_dir);
        node.get_parameter("recording_width", recording.width);
        node.get_parameter("recording_height", recording.height);
        node.get_parameter("recording_fps", recording.fps);
        node.get_parameter("recording_bitrate", recording.bitrate);
        node.get_parameter("recording_gop", recording.gop);
        node.get_parameter("recording_encoder", recording.encoder);
        node.get_parameter("recording_segment_duration_sec", recording.segment_duration_sec);
        std::int64_t buffer_pool_frames = 0;
        std::int64_t max_buffer_bytes   = 0;
        node.get_parameter("recording_buffer_pool_frames", buffer_pool_frames);
        node.get_parameter("recording_max_buffer_bytes", max_buffer_bytes);
        if (buffer_pool_frames < 0 || max_buffer_bytes < 0) {
            return std::unexpected("recording buffer parameters must not be negative");
        }
        recording.buffer_pool_frames = static_cast<std::size_t>(buffer_pool_frames);
        recording.max_buffer_bytes   = static_cast<std::size_t>(max_buffer_bytes);
        if (recording.enabled) {
            const auto recording_ret = recording::validate_config(recording);
            if (!recording_ret) {
                return std::unexpected("recording configuration invalid: " + recording_ret.error());
            }
        }

        node.get_parameter("armor_model_path", armor.armor_model_path);
        double armor_score = armor.score_threshold;
        double armor_nms   = armor.nms_threshold;
        node.get_parameter("armor_score_threshold", armor_score);
        node.get_parameter("armor_nms_threshold", armor_nms);
        armor.score_threshold = static_cast<float>(armor_score);
        armor.nms_threshold   = static_cast<float>(armor_nms);

        node.get_parameter("number_model_path", number.number_model_path);
        double number_conf = number.conf_threshold;
        node.get_parameter("number_conf_threshold", number_conf);
        number.conf_threshold = static_cast<float>(number_conf);
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Error loading configuration: ") + e.what());
    }
    return { };
}

} // namespace radar_camera::node
