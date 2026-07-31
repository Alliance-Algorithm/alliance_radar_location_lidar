#include "radar_lidar/radar_lidar_node.hpp"
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radar_lidar::node::RadarLidarNode>();
    if (!rclcpp::ok()) return 1;
    rclcpp::spin(node);
    const bool failure_requested = node->failure_requested();
    rclcpp::shutdown();
    return failure_requested ? EXIT_FAILURE : EXIT_SUCCESS;
}
