#include "radar_bridge/radar_bridge_node.hpp"
#include "radar_bridge/zmq_bridge.hpp"
#include "radar_bridge/zmq_data_format.hpp"

namespace radar_bridge::node {

RadarBridgeNode::RadarBridgeNode()
    : Node("radar_bridge_node") {
    pub_game_state_ = this->create_publisher<std_msgs::msg::String>("/bridge/game_state", 10);

    sub_lidar_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/li"
                                                                                               "dar"
                                                                                               "/po"
                                                                                               "se",
        10,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
            radar_bridge::zmqdata::pub::LidarLocation& lidar_location) {
            return sub_lidar_pose_callback(msg, lidar_location);
        });
    auto sub_lidar_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
        radar_bridge::zmqdata::pub::LidarLocation & lidar_location)
        -> std::expected<void, std::string> {
        if (!msg) {
            return std::unexpected("Received null message pointer");
        }
        return { };
    };
    auto pub_game_state_callback(radar_bridge::zmqdata::sub::TransmitGameState & game_state)
        -> std::expected<void, std::string> {
        auto msg = std_msgs::msg::String();
        msg.data = game_state;
        pub_game_state_->publish(msg);
        return { };
    };
}

} // namespace radar_bridge::node
