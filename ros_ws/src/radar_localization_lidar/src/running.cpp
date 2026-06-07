#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "model_preprocess.hpp"
int main()
{
    auto model_processor = std::make_unique<Radar::process::model::ModelProcess>();
    const auto config_path = std::string("/workspace/ros_ws/src/radar_localization_lidar/config/setting.yaml");
    YAML::Node model_config = YAML::LoadFile(config_path);
    auto result = model_processor->ConfigLoader(model_config);
    if (!result) {
        RCLCPP_ERROR(rclcpp::get_logger("model_preprocess"), "Failed to load model config: %s", result.error().c_str());
        return 1;
    }
    result = model_processor->ProcessModel();
    if (!result) {
        RCLCPP_ERROR(rclcpp::get_logger("model_preprocess"), "Failed to process model: %s", result.error().c_str());
        return 1;
    }
    auto point_cloud = model_processor->TransformModeltoPointCloud();
    if (!point_cloud) {
        RCLCPP_ERROR(rclcpp::get_logger("model_preprocess"), "Failed to transform model to point cloud: %s", point_cloud.error().c_str());
        return 1;
    }
    for (const auto& point : point_cloud.value()) {
        RCLCPP_INFO(rclcpp::get_logger("model_preprocess"), "Point: [%f, %f, %f]", point.x(), point.y(), point.z());
    }
    return 0;
    
}