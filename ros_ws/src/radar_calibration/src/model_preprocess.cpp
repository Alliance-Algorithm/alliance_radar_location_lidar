#include "model_preprocess.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>

#include <fstream>

using namespace Radar::process::model;

class ModelProcess::Impl {
public:
    std::filesystem::path map_path;
    std::vector<Eigen::Vector3f> point_cloud;

    auto ConfigLoader(const YAML::Node& model_config) -> std::expected<bool, std::string> {
        const auto model_node = model_config["Model"];
        if (!model_node) {
            return std::unexpected("Missing Model section");
        }

        if (const auto map_node = model_node["map_path"]; map_node.IsDefined()) {
            if (!map_node.IsScalar()) {
                return std::unexpected("Model.map_path must be a string");
            }
            const auto str_path = map_node.as<std::string>();
            if (str_path.empty()) {
                return std::unexpected("Model.map_path must not be empty");
            }
            map_path = str_path;
        } else {
            return std::unexpected("Model.map_path is required (should point to the PCD generated "
                                   "by model_to_map)");
        }

        std::error_code ec;
        if (!std::filesystem::exists(map_path, ec)) {
            return std::unexpected("Map PCD not found: " + map_path.string());
        }
        return true;
    }

    auto LoadPointCloud() -> std::expected<bool, std::string> {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path.string(), cloud) == -1) {
            return std::unexpected("Failed to load PCD: " + map_path.string());
        }

        point_cloud.clear();
        point_cloud.reserve(cloud.size());
        for (const auto& pt : cloud.points) {
            point_cloud.emplace_back(pt.x, pt.y, pt.z);
        }

        RCLCPP_INFO(rclcpp::get_logger("model_preprocess"), "Loaded %zu points from %s",
            point_cloud.size(), map_path.string().c_str());
        return true;
    }

    [[nodiscard]] auto GetPointCloud() const noexcept -> const std::vector<Eigen::Vector3f>& {
        return point_cloud;
    }

    [[nodiscard]] auto GetMapPath() const noexcept -> const std::filesystem::path& {
        return map_path;
    }
};

auto ModelProcess::ConfigLoader(const YAML::Node& model_config)
    -> std::expected<bool, std::string> {
    return impl_->ConfigLoader(model_config);
}

ModelProcess::ModelProcess() noexcept
    : impl_ { std::make_unique<Impl>() } { }

auto ModelProcess::LoadPointCloud() -> std::expected<bool, std::string> {
    return impl_->LoadPointCloud();
}

auto ModelProcess::GetPointCloud() const noexcept -> const std::vector<Eigen::Vector3f>& {
    return impl_->GetPointCloud();
}

auto ModelProcess::GetMapPath() const noexcept -> const std::filesystem::path& {
    return impl_->GetMapPath();
}

ModelProcess::~ModelProcess() noexcept = default;
