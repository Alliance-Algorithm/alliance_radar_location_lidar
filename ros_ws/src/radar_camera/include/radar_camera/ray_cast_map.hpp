#pragma once
#include <expected>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace radar_camera::projection {

struct Triangle {
    Eigen::Vector3f v0, v1, v2;
};

class RayCastMap {
public:
    RayCastMap() = default;
    ~RayCastMap() = default;

    auto map_init(const std::string& mesh_path) -> std::expected<void, std::string>;

    auto map_intersect(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction) const
        -> std::optional<Eigen::Vector3f>;

    [[nodiscard]] auto triangles() const -> const std::vector<Triangle>& { return triangles_; }
    [[nodiscard]] auto empty() const -> bool { return triangles_.empty(); }

private:
    struct Vertex {
        Eigen::Vector3f position;
        Eigen::Vector3f normal;
        Eigen::Vector2f uv = Eigen::Vector2f::Zero();
    };

    std::vector<Triangle> triangles_;
};

} // namespace radar_camera::projection
