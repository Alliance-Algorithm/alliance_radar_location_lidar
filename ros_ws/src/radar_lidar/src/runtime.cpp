#include <rclcpp/rclcpp.hpp>

#include "radar_lidar/pipeline.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radar::LidarPipeline>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
