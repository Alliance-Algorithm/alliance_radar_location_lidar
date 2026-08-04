#include "radar_bridge/radar_bridge_node.hpp"

namespace radar_bridge::node {

auto ConfigsLoader(rclcpp::Node& node, BridgeConfig& config) -> std::expected<void, std::string> {
    try {
        node.declare_parameter("zmq_pub_address", std::string("tcp://*:5556"));
        node.declare_parameter(
            "zmq_sub_addresses", std::vector<std::string> { "tcp://localhost:5558" });
        node.declare_parameter("shm_name", std::string("/hikcamera_shm"));
        node.declare_parameter("video_pub_address", std::string("tcp://*:5559"));
        node.declare_parameter("image_topic", std::string("/hikcamera_image"));
        node.declare_parameter("enable_inference", false);
        node.declare_parameter(
            "model_dir", std::string("/workspace/ros_ws/src/radar_camera/model"));
        node.declare_parameter("l1_conf", radar_camera::armor_infer::kL1Conf);
        node.declare_parameter("l2_conf", radar_camera::armor_infer::kL2Conf);
        node.declare_parameter("l3_conf", radar_camera::armor_infer::kL3Conf);

        config.zmq_pub_address   = node.get_parameter("zmq_pub_address").as_string();
        config.zmq_sub_addresses = node.get_parameter("zmq_sub_addresses").as_string_array();
        config.shm_name          = node.get_parameter("shm_name").as_string();
        config.video_pub_address = node.get_parameter("video_pub_address").as_string();
        config.image_topic       = node.get_parameter("image_topic").as_string();
        config.enable_inference  = node.get_parameter("enable_inference").as_bool();
        config.model_dir         = node.get_parameter("model_dir").as_string();
        config.l1_conf           = static_cast<float>(node.get_parameter("l1_conf").as_double());
        config.l2_conf           = static_cast<float>(node.get_parameter("l2_conf").as_double());
        config.l3_conf           = static_cast<float>(node.get_parameter("l3_conf").as_double());
    } catch (const std::exception& e) {
        return std::unexpected(std::string("ConfigsLoader failed: ") + e.what());
    }
    return { };
}

RadarBridgeNode::RadarBridgeNode()
    : Node("radar_bridge_node") {
    auto result = ConfigsLoader(*this, config_);
    if (!result.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "ConfigsLoader failed: %s", result.error().c_str());
        throw std::runtime_error("ConfigsLoader failed: " + result.error());
    }
    RCLCPP_INFO(this->get_logger(), "ConfigsLoader succeeded");

    auto zmq_ret = zmq_bridge_.zmqpub_init(config_.zmq_pub_address);
    if (!zmq_ret.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "zmqpub_init failed: %s", zmq_ret.error().c_str());
        throw std::runtime_error("zmqpub_init failed: " + zmq_ret.error());
    }
    RCLCPP_INFO(this->get_logger(), "zmqpub_init succeeded");

    auto sub_ret = zmq_bridge_.zmqsub_init(config_.zmq_sub_addresses);
    if (!sub_ret.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "zmqsub_init failed: %s", sub_ret.error().c_str());
        throw std::runtime_error("zmqsub_init failed: " + sub_ret.error());
    }
    RCLCPP_INFO(this->get_logger(), "zmqsub_init succeeded");

    game_state_publisher_ =
        this->create_publisher<radar_interfaces::msg::GameState>("/bridge/game_state", 10);

    lidar_pose_subscription_ = this->create_subscription<radar_interfaces::msg::LidarLocation>("/li"
                                                                                               "dar"
                                                                                               "/lo"
                                                                                               "cat"
                                                                                               "io"
                                                                                               "n",
        10, [this](const radar_interfaces::msg::LidarLocation& msg) {
            auto result = sub_lidar_pose_callback(msg);
            if (!result.has_value()) {
                RCLCPP_ERROR(this->get_logger(), "sub_lidar_pose_callback failed: %s",
                    result.error().c_str());
            }
        });

    zmq_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), [this]() {
        if (zmq_bridge_.zmqsub(game_state_)) {
            pub_game_state_callback();
        }
    });

    std::shared_ptr<radar_camera::armor_infer::ArmorInfer> infer;
    if (config_.enable_inference) {
        auto created = radar_camera::armor_infer::ArmorInfer::create(
            config_.model_dir, config_.l1_conf, config_.l2_conf, config_.l3_conf);
        if (created) {
            infer = *created;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "ArmorInfer init failed, video stream will passthrough: %s",
                created.error().c_str());
        }
    }
    auto init_ret =
        video_bridge_.video_init(config_.shm_name, config_.video_pub_address, std::move(infer));
    if (!init_ret.has_value()) {
        // SHM 可能尚未就绪（相机/回放器后启动）。坐标转发（ZMQ 5556）不依赖视频，
        // 每 2s 重试 video_init 直到 SHM 就绪，保证推流可用（调相机位姿需要画面）。
        RCLCPP_WARN(this->get_logger(),
            "video_init failed, will retry every 2s (coordinate forwarding unaffected): %s",
            init_ret.error().c_str());
        video_retry_timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
            auto retry =
                video_bridge_.video_init(config_.shm_name, config_.video_pub_address, nullptr);
            if (!retry.has_value()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "video retry pending: %s",
                    retry.error().c_str());
                return;
            }
            RCLCPP_INFO(this->get_logger(), "video_init succeeded (retry)");
            video_retry_timer_->cancel();
            video_retry_timer_.reset();
            auto video_ret = video_bridge_.video_thread();
            if (!video_ret.has_value()) {
                RCLCPP_ERROR(get_logger(), "video_thread failed: %s", video_ret.error().c_str());
            } else {
                RCLCPP_INFO(get_logger(), "video_thread started");
            }
        });
    } else {
        RCLCPP_INFO(this->get_logger(), "video_init succeeded");
        auto video_ret = video_bridge_.video_thread();
        if (!video_ret.has_value()) {
            RCLCPP_ERROR(this->get_logger(), "video_thread failed: %s", video_ret.error().c_str());
            throw std::runtime_error("video_thread failed: " + video_ret.error());
        }
        RCLCPP_INFO(this->get_logger(), "video_thread started");
    }
}
RadarBridgeNode::~RadarBridgeNode() { video_bridge_.video_thread_stop(); }
auto RadarBridgeNode::sub_lidar_pose_callback(const radar_interfaces::msg::LidarLocation& msg)
    -> std::expected<void, std::string> {
    lidar_location_.opponent_hero_x       = msg.opponent_hero_x;
    lidar_location_.opponent_hero_y       = msg.opponent_hero_y;
    lidar_location_.opponent_engineer_x   = msg.opponent_engineer_x;
    lidar_location_.opponent_engineer_y   = msg.opponent_engineer_y;
    lidar_location_.opponent_infantry_3_x = msg.opponent_infantry_3_x;
    lidar_location_.opponent_infantry_3_y = msg.opponent_infantry_3_y;
    lidar_location_.opponent_infantry_4_x = msg.opponent_infantry_4_x;
    lidar_location_.opponent_infantry_4_y = msg.opponent_infantry_4_y;
    lidar_location_.opponent_aerial_x     = msg.opponent_aerial_x;
    lidar_location_.opponent_aerial_y     = msg.opponent_aerial_y;
    lidar_location_.opponent_sentry_x     = msg.opponent_sentry_x;
    lidar_location_.opponent_sentry_y     = msg.opponent_sentry_y;

    lidar_location_.ally_hero_x       = msg.ally_hero_x;
    lidar_location_.ally_hero_y       = msg.ally_hero_y;
    lidar_location_.ally_engineer_x   = msg.ally_engineer_x;
    lidar_location_.ally_engineer_y   = msg.ally_engineer_y;
    lidar_location_.ally_infantry_3_x = msg.ally_infantry_3_x;
    lidar_location_.ally_infantry_3_y = msg.ally_infantry_3_y;
    lidar_location_.ally_infantry_4_x = msg.ally_infantry_4_x;
    lidar_location_.ally_infantry_4_y = msg.ally_infantry_4_y;
    lidar_location_.ally_aerial_x     = msg.ally_aerial_x;
    lidar_location_.ally_aerial_y     = msg.ally_aerial_y;
    lidar_location_.ally_sentry_x     = msg.ally_sentry_x;
    lidar_location_.ally_sentry_y     = msg.ally_sentry_y;
    // 0x0305 频率上限 5Hz：限频转发（值持续更新，仅发送节流）
    const auto now = std::chrono::steady_clock::now();
    if (now - last_location_send_ < std::chrono::duration<double>(1.0 / kLocationMaxHz)) {
        return { };
    }
    last_location_send_ = now;
    auto zmq_ret        = zmq_bridge_.zmqpub(lidar_location_);
    if (!zmq_ret.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "zmqpub failed: %s", zmq_ret.error().c_str());
    }
    return { };
}
auto RadarBridgeNode::pub_game_state_callback() -> std::expected<void, std::string> {
    auto msg              = radar_interfaces::msg::GameState();
    msg.cmd_id            = game_state_.cmd_id;
    msg.game_type         = game_state_.game_type;
    msg.game_progress     = game_state_.game_progress;
    msg.stage_remain_time = game_state_.stage_remain_time;
    msg.sync_timestamp    = game_state_.sync_timestamp;
    game_state_publisher_->publish(msg);
    return { };
}
} // namespace radar_bridge::node
