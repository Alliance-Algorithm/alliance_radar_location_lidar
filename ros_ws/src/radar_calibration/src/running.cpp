#include "model_preprocess.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>

int main() {
    auto model_processor = std::make_unique<Radar::process::model::ModelProcess>();
    YAML::Node model_config;

    // Try installed config first, fall back to source-tree path
    std::filesystem::path config_path = "/workspace/ros_ws/src/radar_calibration/config/"
                                        "setting.yaml";
    try {
        const auto installed =
            std::filesystem::path { ament_index_cpp::get_package_share_directory("radar_"
                                                                                 "calibration") }
            / "config" / "setting.yaml";
        if (std::filesystem::exists(installed)) {
            config_path = installed;
        }
    } catch (const std::exception&) {
        // ament index unavailable — use source-tree fallback
    }

    try {
        model_config = YAML::LoadFile(config_path.string());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("model_preprocess"), "Failed to load model config %s: %s",
            config_path.string().c_str(), e.what());
        return 1;
    }
    auto result = model_processor->ConfigLoader(model_config);
    if (!result) {
        RCLCPP_ERROR(rclcpp::get_logger("model_preprocess"), "Failed to load model config: %s",
            result.error().c_str());
        return 1;
    }
    result = model_processor->LoadPointCloud();
    if (!result) {
        RCLCPP_ERROR(rclcpp::get_logger("model_preprocess"), "Failed to load point cloud: %s",
            result.error().c_str());
        return 1;
    }
    const auto& point_cloud = model_processor->GetPointCloud();
    (void)point_cloud;
    RCLCPP_INFO(rclcpp::get_logger("model_preprocess"), "Model preprocess done: %zu points from %s",
        point_cloud.size(), model_processor->GetMapPath().string().c_str());
    return 0;
}
