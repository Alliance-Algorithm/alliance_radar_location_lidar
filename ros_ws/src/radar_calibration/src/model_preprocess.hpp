#pragma once

#include <Eigen/Core>
#include <expected>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace Radar::process::model {

class ModelProcess final {
public:
    ModelProcess() noexcept;
    ~ModelProcess() noexcept;

    auto ConfigLoader(const YAML::Node& model_config) -> std::expected<bool, std::string>;
    auto LoadPointCloud() -> std::expected<bool, std::string>;

    [[nodiscard]] auto GetPointCloud() const noexcept -> const std::vector<Eigen::Vector3f>&;
    [[nodiscard]] auto GetMapPath() const noexcept -> const std::filesystem::path&;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace Radar::process::model
