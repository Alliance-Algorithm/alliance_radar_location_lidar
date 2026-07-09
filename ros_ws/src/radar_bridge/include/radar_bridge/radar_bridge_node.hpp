#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "radar_bridge/zmq_bridge.hpp"
#include "radar_bridge/zmq_data_format.hpp"
#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <expected>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace radar_bridge::node {

class RadarBridgeNode final : public rclcpp::Node {
public:
    RadarBridgeNode();
    auto sub_lidar_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
        radar_bridge::zmqdata::pub::LidarLocation& lidar_location)
        -> std::expected<void, std::string>;
    auto pub_game_state_callback(radar_bridge::zmqdata::sub::TransmitGameState& game_state)
        -> std::expected<void, std::string>;

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_game_state_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_lidar_pose_;
    std::atomic<bool> zmqpub_thread_running_ { false };
    std::atomic<bool> zmqsub_thread_running_ { false };
};

} // namespace radar_bridge::node
