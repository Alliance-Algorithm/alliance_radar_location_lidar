#include <rclcpp/rclcpp.hpp>

#include "radar_fusion/radar_fusion_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<radar_fusion::node::RadarFusionNode>());
    rclcpp::shutdown();
    return 0;
}
