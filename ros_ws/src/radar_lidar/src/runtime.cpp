#include "radar_lidar/radar_lidar_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radar_lidar::node::RadarLidarNode>();
    if (!rclcpp::ok()) return 1;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
