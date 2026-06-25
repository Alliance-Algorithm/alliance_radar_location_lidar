#include <rclcpp/rclcpp.hpp>

#include "radar_fusion/fusion_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<radar::fusion::FusionNode>());
    rclcpp::shutdown();
    return 0;
}
