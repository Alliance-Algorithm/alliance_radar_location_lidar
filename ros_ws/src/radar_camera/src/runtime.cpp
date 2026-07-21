#include <rclcpp/rclcpp.hpp>

#include "radar_camera/radar_camera_node.hpp"

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radar_camera::node::RadarCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
