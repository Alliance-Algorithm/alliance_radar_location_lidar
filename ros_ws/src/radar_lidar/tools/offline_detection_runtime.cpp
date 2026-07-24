#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "radar_lidar/offline_detection_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<radar_lidar::node::OfflineDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
