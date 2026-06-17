#include <expected>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
namespace Radar::process::image {
class ImageProcess final {
public:
    ImageProcess() noexcept  = default;
    ~ImageProcess() noexcept = default;
    auto ConfigLoader(const YAML::Node& camera_config) -> std::expected<bool, std::string>;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}