#include "radar_bridge/videostream_bridge.hpp"
#include "radar_bridge/zmq_bridge.hpp"
#include "radar_bridge/zmq_data_format.hpp"
#include "radar_interfaces/msg/game_state.hpp"
#include "radar_interfaces/msg/lidar_location.hpp"
#include <atomic>
#include <expected>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace radar_bridge::node {

struct BridgeConfig {
    std::string zmq_pub_address;
    std::vector<std::string> zmq_sub_addresses;
    std::string shm_name;
    std::string video_pub_address;
    int video_width  = 0;
    int video_height = 0;
};

auto ConfigsLoader(rclcpp::Node& node, BridgeConfig& config)
    -> std::expected<void, std::string>;

class RadarBridgeNode final : public rclcpp::Node {
public:
    RadarBridgeNode();
    ~RadarBridgeNode() override;
    auto sub_lidar_pose_callback(const radar_interfaces::msg::LidarLocation& msg)
        -> std::expected<void, std::string>;
    auto pub_game_state_callback() -> std::expected<void, std::string>;

private:
    rclcpp::Publisher<radar_interfaces::msg::GameState>::SharedPtr game_state_publisher_;
    rclcpp::Subscription<radar_interfaces::msg::LidarLocation>::SharedPtr lidar_pose_subscription_;

    radar_bridge::zmqdata::pub::LidarLocation lidar_location_ { };
    radar_bridge::zmqdata::sub::TransmitGameState game_state_ { };

    std::atomic<bool> zmqpub_thread_running_ { false };
    std::atomic<bool> zmqsub_thread_running_ { false };

    BridgeConfig config_ { };
    radar_bridge::videostream_bridge::VideoBridge video_bridge_;
};

} // namespace radar_bridge::node
