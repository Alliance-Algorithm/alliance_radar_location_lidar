#include "model_preprocess.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
int main() {
    auto model_processor = std::make_unique<Radar::process::model::ModelProcess>();
    YAML::Node model_config;
    try {
        const auto config_path = ament_index_cpp::get_package_share_directory("radar_calibration")
            + "/config/setting.yaml";
        model_config = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("model_preprocess"),
            "Failed to resolve/load model config: %s", e.what());
        return 1;
    }
    auto result = model_processor->ConfigLoader(model_config);
    if (!result) {
        RCLCPP_ERROR(rclcpp::get_logger("model_preprocess"), "Failed to load model config: %s",
            result.error().c_str());
        return 1;
    }
    result = model_processor->ProcessModel();
    if (!result) {
        RCLCPP_ERROR(rclcpp::get_logger("model_preprocess"), "Failed to process model: %s",
            result.error().c_str());
        return 1;
    }
    result = model_processor->TransformModeltoPointCloud();
    if (!result) {
        RCLCPP_ERROR(rclcpp::get_logger("model_preprocess"),
            "Failed to transform model to point cloud: %s", result.error().c_str());
        return 1;
    }
    const auto& point_cloud = model_processor->GetPointCloud();
#ifdef RADAR_DEBUG
    RCLCPP_INFO(rclcpp::get_logger("model_preprocess"), "Generated %zu points", point_cloud.size());
    for (const auto& point : point_cloud) {
        RCLCPP_INFO(rclcpp::get_logger("model_preprocess"), "Point: [%f, %f, %f]", point.x(),
            point.y(), point.z());
    }
#else
    (void)point_cloud;
#endif
    return 0;
}
