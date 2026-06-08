#include <Eigen/Core>
#include <expected>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
namespace Radar::capture {
class PointCloudCapture final {
public:
    struct PointCloudData {
        std::vector<Eigen::Vector3f> points;
        std::chrono::steady_clock::time_point timestamp;
    };
    PointCloudCapture() noexcept;
    ~PointCloudCapture() noexcept;
    auto PointCloudSubscriber(const std::string& topic_name) -> std::expected<bool, std::string>;
    auto GetLatestPointCloud() -> std::expected<PointCloudData, std::string>;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
}
