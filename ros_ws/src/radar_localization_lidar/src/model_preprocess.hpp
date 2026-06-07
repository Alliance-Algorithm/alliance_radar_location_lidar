#include <string>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <expected>
#include <Eigen/Core>  
namespace Radar::process::model{
    class ModelProcess final{
        public:
            ModelProcess() noexcept;
            ~ModelProcess() noexcept;
            auto ConfigLoader(const YAML::Node& model_config)->std::expected<bool, std::string>; 
            auto ProcessModel()->std::expected<bool, std::string>;
            auto TransformModeltoPointCloud()->std::expected<bool, std::string>;
            [[nodiscard]] auto GetPointCloud() const noexcept->const std::vector<Eigen::Vector3f>&;
        private:
            class Impl;
            std::unique_ptr<Impl> impl_;
    };
}