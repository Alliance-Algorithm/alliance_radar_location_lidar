#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>

#include "radar_camera/radar_camera_node.hpp"

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    std::shared_ptr<radar_camera::node::RadarCameraNode> node;
    try {
        node = std::make_shared<radar_camera::node::RadarCameraNode>();
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("radar_camera_runtime"), "radar_camera startup failed: %s",
            error.what());
        rclcpp::shutdown();
        return 1;
    }
    const auto failed = node->status() == radar_camera::node::NodeStatus::failed;
    rclcpp::shutdown();
    return failed ? 1 : 0;
}
