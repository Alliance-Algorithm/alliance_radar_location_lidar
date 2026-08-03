#include "radar_camera/radar_camera_node.hpp"

#include <algorithm>

#include <cstdint>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace radar_camera::node {

namespace {
    class ConstructorCleanupGuard {
    public:
        explicit ConstructorCleanupGuard(std::function<void()> cleanup)
            : cleanup_(std::move(cleanup)) { }
        ~ConstructorCleanupGuard() { cleanup_(); }
        void release() noexcept {
            cleanup_ = [] { };
        }

    private:
        std::function<void()> cleanup_;
    };
} // namespace

RadarCameraNode::RadarCameraNode()
    : Node("radar_camera_node") {
    auto cleanup_guard = ConstructorCleanupGuard([this]() noexcept { constructor_cleanup(); });
    auto ret           = ConfigsLoader(*this, camera_config_, inference_config_, projection_config_,
        armor_refine_config_, number_refine_config_);
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

    // TF: map→camera_optical_frame（GICP 配准 + 固定安装外参）。
    // 可用时用 TF 外参（跟随 GICP/锁定）；不可用时 fallback 写死初值。
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto map_ret = projector_.proj_init_map(projection_config_);
    if (!map_ret)
        RCLCPP_WARN(get_logger(), "Projector map init skipped: %s", map_ret.error().c_str());

    pose_publisher_ = this->create_publisher<radar_interfaces::msg::CameraDetectionPose>(
        camera_config_.pub_topic_name, 10);
    RCLCPP_INFO(get_logger(), "Publisher created");

    // SHM open 延迟到 infer_thread 内重试：相机驱动可能晚于本节点启动。
    // 重试超时（30s）仍失败则 Fatal + shutdown，保证错误可见。
    ret = infer_thread_start();
    if (!ret) {
        throw std::runtime_error("infer_thread_start failed: " + ret.error());
    }
    RCLCPP_INFO(get_logger(), "infer_thread started");

    status_.store(NodeStatus::running, std::memory_order_release);
    cleanup_guard.release();
}

RadarCameraNode::~RadarCameraNode() {
    infer_thread_stop();
    status_.store(NodeStatus::stopped, std::memory_order_release);
}

auto RadarCameraNode::status() const -> NodeStatus {
    return status_.load(std::memory_order_acquire);
}

auto RadarCameraNode::failure_reason() const -> std::string {
    std::lock_guard lock(status_mutex_);
    return failure_reason_;
}

auto lifecycle_order() -> std::vector<LifecycleComponent> {
    return { LifecycleComponent::inference, LifecycleComponent::shm };
}

auto constructor_cleanup_order(const std::vector<LifecycleComponent>& started)
    -> std::vector<LifecycleComponent> {
    std::vector<LifecycleComponent> cleanup;
    for (auto it = started.rbegin(); it != started.rend(); ++it) {
        if (*it == LifecycleComponent::inference) {
            cleanup.push_back(*it);
        }
    }
    if (std::find(started.begin(), started.end(), LifecycleComponent::shm) != started.end()) {
        cleanup.emplace_back(LifecycleComponent::shm);
    }
    return cleanup;
}

auto RadarCameraNode::constructor_cleanup() noexcept -> void {
    std::vector<LifecycleComponent> started;
    if (shm_reader_.is_open()) started.emplace_back(LifecycleComponent::shm);
    if (infer_thread_.joinable() || infer_running_)
        started.emplace_back(LifecycleComponent::inference);

    for (const auto component : constructor_cleanup_order(started)) {
        switch (component) {
        case LifecycleComponent::inference:
            infer_thread_stop();
            break;
        case LifecycleComponent::shm:
            break;
        }
    }
}

auto RadarCameraNode::infer_thread_start() -> std::expected<void, std::string> {
    infer_running_ = true;
    infer_thread_  = std::thread([this]() {
        // SHM open 重试：相机驱动可能晚于本节点启动。
        // 30s 超时仍失败 → Fatal + shutdown（保留错误可见性）。
        constexpr auto kShmOpenTimeout = std::chrono::seconds { 30 };
        const auto shm_open_start      = std::chrono::steady_clock::now();
        bool shm_ready                 = false;
        while (infer_running_.load(std::memory_order_acquire) && !shm_ready) {
            auto open_ret = shm_reader_.open(camera_config_.shm_name.c_str());
            if (open_ret) {
                shm_ready = true;
                RCLCPP_INFO(
                    get_logger(), "SHM open succeeded: %s", camera_config_.shm_name.c_str());
                break;
            }
            if (std::chrono::steady_clock::now() - shm_open_start > kShmOpenTimeout) {
                RCLCPP_FATAL(
                    get_logger(), "SHM open timed out after 30s: %s", open_ret.error().c_str());
                infer_running_.store(false, std::memory_order_release);
                rclcpp::shutdown();
                return;
            }
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "SHM not ready yet (waiting for camera driver): %s", open_ret.error().c_str());
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        while (infer_running_.load(std::memory_order_acquire)) {
            ++frame_count_;
            cv::Mat orig_frame;
            const auto t_frame = std::chrono::steady_clock::now();
            auto shm_frame = shm_reader_.wait_next(std::chrono::milliseconds { 100 });
            const auto t_wait = std::chrono::steady_clock::now();
            if (!shm_frame) {
                if (shm_frame.error().code != hikcamera::FrameReadErrorCode::Timeout) {
                    RCLCPP_WARN(get_logger(), "SHM read error (code=%d): %s",
                        static_cast<int>(shm_frame.error().code),
                        shm_frame.error().message.c_str());
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            if (!shm_frame->valid()) {
                RCLCPP_WARN(get_logger(), "SHM frame failed integrity check");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            capture_timestamp_ = std::chrono::steady_clock::time_point(
                std::chrono::nanoseconds(shm_frame->metadata().host_monotonic_ns));

            // 低频更新相机外参（TF 可用时跟随 GICP 锁定结果）
            if (!tf_ready_ || (frame_count_ & 0x3F) == 0) {
                update_camera_extrinsic_from_tf();
            }
            // 必须保留原图副本：流水线下 refine 在下一轮才消费，SHM 视图会被写者
            // 覆盖。复用预分配池（round-robin 3 槽），clone 退化为纯 memcpy。
            shm_frame->mat().copyTo(orig_pool_[frame_count_ % 3]);
            orig_frame = orig_pool_[frame_count_ % 3];
            const auto t_clone = std::chrono::steady_clock::now();

            // Letterbox 到 L1 模型输入（与 annotate/训练预处理一致，保持宽高比+黑边填充）。
            // 此前用拉伸 resize：小目标（远距离机器人/无人机）变形后模型检出率显著下降
            // （同一帧 annotate letterbox conf 0.94 vs camera resize dets=0）。
            const float lb_scale = std::min(
                static_cast<float>(inference_config_.model_input_width) / orig_frame.cols,
                static_cast<float>(inference_config_.model_input_height) / orig_frame.rows);
            const int resized_w = std::max(
                1, static_cast<int>(std::lround(orig_frame.cols * lb_scale)));
            const int resized_h = std::max(
                1, static_cast<int>(std::lround(orig_frame.rows * lb_scale)));
            const int pad_x = (inference_config_.model_input_width - resized_w) / 2;
            const int pad_y = (inference_config_.model_input_height - resized_h) / 2;
            cv::Mat frame(inference_config_.model_input_height,
                inference_config_.model_input_width, CV_8UC3, cv::Scalar::all(0));
            cv::Mat resized;
            cv::resize(orig_frame, resized, cv::Size(resized_w, resized_h));
            resized.copyTo(frame(cv::Rect(pad_x, pad_y, resized_w, resized_h)));
            const auto t_resize = std::chrono::steady_clock::now();

            auto tensor = model_inference_->infer_preprocess(frame,
                static_cast<size_t>(inference_config_.model_input_width),
                static_cast<size_t>(inference_config_.model_input_height));
            if (!tensor) {
                RCLCPP_WARN(
                    get_logger(), "Inference preprocess failed: %s", tensor.error().c_str());
                continue;
            }

            // 流水线：enqueue 帧 N（H2D u8 + GPU normalize + L1 前向）后，
            // GPU 跑 N 的同时 CPU 处理上一帧 N-1（wait 返回 N-1 结果）。
            auto async_ret = model_inference_->infer_runtime_async();
            if (!async_ret) {
                RCLCPP_WARN(get_logger(), "Inference start failed: %s", async_ret.error().c_str());
                continue;
            }
            const auto t_async = std::chrono::steady_clock::now();

            if (have_prev_) {
                auto raw = model_inference_->infer_runtime_wait();
                const auto t_waitgpu = std::chrono::steady_clock::now();
                if (!raw) {
                    RCLCPP_WARN(
                        get_logger(), "Inference wait failed: %s", raw.error().c_str());
                    have_prev_ = false;
                    prev_orig_frame_.release();
                    continue;
                }

                auto dets = model_inference_->infer_postprocess(
                    raw->get(), inference_config_.model_input_width,
                    inference_config_.model_input_height);
                if (!dets) {
                    RCLCPP_WARN(get_logger(), "Inference postprocess failed: %s",
                        dets.error().c_str());
                    have_prev_ = false;
                    prev_orig_frame_.release();
                    continue;
                }
                std::vector<detection::Detection> refined(dets->get());
                if (armor_refine_enabled_) {
                    // refine 用上一帧全分辨率原图（已 clone，跨帧安全）。
                    for (auto& det : refined) {
                        armor_refiner_.refine(prev_orig_frame_, det,
                            inference_config_.drone_class_ids, 1.0f, 1.0f);
                    }
                }

                // L1 检测在 letterbox 模型空间 (1280x1280)；映射回全分辨率原图：
                // (x - pad) / scale 逆变换（用上一帧的 pad/scale）。
                for (auto& det : refined) {
                    det.bbox.x      = (det.bbox.x - static_cast<float>(prev_pad_x_))
                        / prev_lb_scale_;
                    det.bbox.y      = (det.bbox.y - static_cast<float>(prev_pad_y_))
                        / prev_lb_scale_;
                    det.bbox.width /= prev_lb_scale_;
                    det.bbox.height /= prev_lb_scale_;
                    det.center.x = (det.center.x - static_cast<float>(prev_pad_x_))
                        / prev_lb_scale_;
                    det.center.y = (det.center.y - static_cast<float>(prev_pad_y_))
                        / prev_lb_scale_;
                }

                auto projected = projector_.proj_preprocess(refined);
                auto pose = std::expected<robot_pose::RobotPose, std::string>(
                    std::unexpected("projection preprocess failed"));
                if (!projected) {
                    RCLCPP_WARN(
                        get_logger(), "Projection preprocess failed: %s", projected.error().c_str());
                } else {
                    pose = projector_.proj_postprocess(*projected, refined);
                    if (!pose) {
                        RCLCPP_WARN(get_logger(), "Projection postprocess failed: %s",
                            pose.error().c_str());
                    }
                }

                const auto stamp = prev_capture_timestamp_;
                if (pose) PublishCallback(*pose, stamp);
                const auto t_done = std::chrono::steady_clock::now();
                if (frame_count_ % 100 == 0) {
                    const auto ms = [](auto a, auto b) {
                        return std::chrono::duration<double, std::milli>(b - a).count();
                    };
                    RCLCPP_INFO(get_logger(),
                        "[perf] wait=%5.1f clone=%5.1f resize=%5.1f async=%5.1f "
                        "gpu_wait=%5.1f post+refine+proj=%5.1f cycle=%5.1f n_dets=%zu",
                        ms(t_frame, t_wait), ms(t_wait, t_clone), ms(t_clone, t_resize),
                        ms(t_resize, t_async), ms(t_async, t_waitgpu), ms(t_waitgpu, t_done),
                        ms(t_frame, t_done), refined.size());
                }
            }

            // 帧 N 成为下一轮的"上一帧"
            prev_orig_frame_         = orig_frame;
            prev_lb_scale_           = lb_scale;
            prev_pad_x_              = pad_x;
            prev_pad_y_              = pad_y;
            prev_capture_timestamp_  = capture_timestamp_;
            have_prev_               = true;
        }
    });
    return { };
}

auto RadarCameraNode::infer_thread_stop() -> void {
    infer_running_ = false;
    if (infer_thread_.joinable()) infer_thread_.join();
}

void RadarCameraNode::update_camera_extrinsic_from_tf() {
    if (!tf_buffer_) return;
    geometry_msgs::msg::TransformStamped tf_msg;
    try {
        // 0 超时：TF 未就绪时立即返回（此前 0.1s 阻塞导致每帧 ~100ms 等待，
        // 推理被拖到 ~7fps；TF 就绪后下一帧即拿到，无需等待）。
        tf_msg = tf_buffer_->lookupTransform(
            "map", "camera_optical_frame", tf2::TimePointZero, tf2::durationFromSec(0.0));
    } catch (const tf2::TransformException&) {
        return; // TF 未就绪，保留 fallback 外参
    }
    Eigen::Isometry3d t_map_camera = Eigen::Isometry3d::Identity();
    t_map_camera.translation()     = Eigen::Vector3d(tf_msg.transform.translation.x,
        tf_msg.transform.translation.y, tf_msg.transform.translation.z);
    t_map_camera.linear()          = Eigen::Quaterniond(tf_msg.transform.rotation.w,
        tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z)
                                         .toRotationMatrix();
    projector_.set_map_camera(t_map_camera);
    if (!tf_ready_) {
        tf_ready_ = true;
        RCLCPP_INFO(get_logger(), "camera extrinsic from TF: t=(%.3f, %.3f, %.3f)",
            tf_msg.transform.translation.x, tf_msg.transform.translation.y,
            tf_msg.transform.translation.z);
    }
}

auto RadarCameraNode::PublishCallback(const robot_pose::RobotPose& robot_poses,
    std::chrono::steady_clock::time_point stamp) -> void {
    auto pose_msg                  = radar_interfaces::msg::CameraDetectionPose();
    pose_msg.header.stamp          = rclcpp::Time(stamp.time_since_epoch().count());
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
    armor_refine::ArmorRefineConfig& armor,
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
