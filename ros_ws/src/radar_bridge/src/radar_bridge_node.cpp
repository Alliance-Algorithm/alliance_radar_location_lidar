#include "radar_bridge/radar_bridge_node.hpp"

namespace radar_bridge::node {

auto ConfigsLoader(rclcpp::Node& node, BridgeConfig& config) -> std::expected<void, std::string> {
    config.zmq_pub_address   = node.get_parameter("zmq_pub_address").as_string();
    config.zmq_sub_addresses = node.get_parameter("zmq_sub_addresses").as_string_array();
    config.shm_name          = node.get_parameter("shm_name").as_string();
    config.video_pub_address = node.get_parameter("video_pub_address").as_string();
    config.image_topic       = node.get_parameter("image_topic").as_string();
    config.video_width       = node.get_parameter("video_width").as_int();
    config.video_height      = node.get_parameter("video_height").as_int();
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

    zmq_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), [this]() {
            if (zmq_bridge_.zmqsub(game_state_)) {
                pub_game_state_callback();
            }
        });

    auto init_ret = video_bridge_.video_init(
        config_.shm_name, config_.video_pub_address, config_.video_width, config_.video_height);
    if (!init_ret.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "video_init failed: %s", init_ret.error().c_str());
        throw std::runtime_error("video_init failed: " + init_ret.error());
    }
    RCLCPP_INFO(this->get_logger(), "video_init succeeded");

    auto video_ret = video_bridge_.video_thread();
    if (!video_ret.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "video_thread failed: %s", video_ret.error().c_str());
        throw std::runtime_error("video_thread failed: " + video_ret.error());
    }
    RCLCPP_INFO(this->get_logger(), "video_thread started");
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
    zmq_bridge_.zmqpub(lidar_location_);
    return { };
}
auto RadarBridgeNode::pub_game_state_callback() -> std::expected<void, std::string> {
    auto msg = radar_interfaces::msg::GameState();
    msg.cmd_id            = game_state_.cmd_id;
    msg.game_type         = game_state_.game_type;
    msg.game_progress     = game_state_.game_progress;
    msg.stage_remain_time = game_state_.stage_remain_time;
    msg.sync_timestamp    = game_state_.sync_timestamp;
    game_state_publisher_->publish(msg);
    return { };
}
} // namespace radar_bridge::node
