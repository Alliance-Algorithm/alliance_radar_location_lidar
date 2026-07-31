#include "radar_lidar/grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <limits>
#include <string>

namespace radar_lidar::grid_map {

namespace {

    struct Cell {
        std::size_t count = 0;
        float z_min       = std::numeric_limits<float>::max();
        float z_max       = std::numeric_limits<float>::lowest();
    };

    auto floor_to_resolution(double v, double res) -> double { return std::floor(v / res) * res; }

    auto ceil_to_resolution(double v, double res) -> double { return std::ceil(v / res) * res; }

} // namespace

auto rasterize(const pcl::PointCloud<pcl::PointXYZ>& cloud, const GridMapParams& params,
    const std::optional<Bounds>& bounds) -> std::expected<GridMapResult, std::string> {
    if (params.resolution <= 0.0) return std::unexpected("resolution must be > 0");
    if (params.min_points < 1) return std::unexpected("min_points must be >= 1");
    if (params.dilate < 0) return std::unexpected("dilate must be >= 0");
    if (params.height_threshold < 0.0) return std::unexpected("height_threshold must be >= 0");
    if (cloud.empty() && !bounds.has_value())
        return std::unexpected("empty cloud and no bounds provided");

    const auto res = params.resolution;

    double x_min = 0.0, y_min = 0.0, x_max = 0.0, y_max = 0.0;
    if (bounds.has_value()) {
        const auto& b = *bounds;
        if (b.x_max <= b.x_min || b.y_max <= b.y_min)
            return std::unexpected("bounds are degenerate (x_max <= x_min or y_max <= y_min)");
        x_min = b.x_min;
        y_min = b.y_min;
        x_max = b.x_max;
        y_max = b.y_max;
    } else {
        double first_x = 0.0, first_y = 0.0;
        bool have_finite = false;
        for (const auto& pt : cloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
            first_x     = pt.x;
            first_y     = pt.y;
            have_finite = true;
            break;
        }
        if (!have_finite) return std::unexpected("no finite points and no bounds provided");
        x_min = floor_to_resolution(first_x, res);
        x_max = ceil_to_resolution(first_x, res);
        y_min = floor_to_resolution(first_y, res);
        y_max = ceil_to_resolution(first_y, res);
        for (const auto& pt : cloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
            x_min = std::min(x_min, floor_to_resolution(pt.x, res));
            x_max = std::max(x_max, ceil_to_resolution(pt.x, res));
            y_min = std::min(y_min, floor_to_resolution(pt.y, res));
            y_max = std::max(y_max, ceil_to_resolution(pt.y, res));
        }
    }

    const auto width  = static_cast<int>(std::lround((x_max - x_min) / res));
    const auto height = static_cast<int>(std::lround((y_max - y_min) / res));
    if (width <= 0 || height <= 0)
        return std::unexpected("degenerate map extent after bounds computation");

    std::vector<Cell> cells(static_cast<std::size_t>(width) * height);

    const auto index_of = [&](double x, double y) -> std::size_t {
        const auto ix = static_cast<int>(std::floor((x - x_min) / res));
        const auto iy = static_cast<int>(std::floor((y - y_min) / res));
        if (ix < 0 || ix >= width || iy < 0 || iy >= height) return static_cast<std::size_t>(-1);
        return static_cast<std::size_t>(iy) * width + static_cast<std::size_t>(ix);
    };

    for (const auto& pt : cloud.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        const auto idx = index_of(pt.x, pt.y);
        if (idx == static_cast<std::size_t>(-1)) continue;
        auto& cell = cells[idx];
        cell.count += 1;
        cell.z_min = std::min(cell.z_min, pt.z);
        cell.z_max = std::max(cell.z_max, pt.z);
    }

    GridMapResult result;
    result.width      = width;
    result.height     = height;
    result.origin_x   = x_min;
    result.origin_y   = y_min;
    result.resolution = res;
    result.data.resize(cells.size(), -1);

    const auto min_points = static_cast<std::size_t>(params.min_points);
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const auto& cell = cells[i];
        if (cell.count == 0) continue;
        if (cell.count >= min_points && (cell.z_max - cell.z_min) > params.height_threshold) {
            result.data[i] = 0; // 障碍
        } else {
            result.data[i] = 100; // 空闲
        }
    }

    // 膨胀: 以每个障碍格为中心, 半径 dilate 的圆域内格子置障碍
    if (params.dilate > 0) {
        const auto d                 = params.dilate;
        std::vector<int8_t> expanded = result.data;
        for (int iy = 0; iy < height; ++iy) {
            for (int ix = 0; ix < width; ++ix) {
                if (result.data[static_cast<std::size_t>(iy) * width + ix] != 0) continue;
                const auto iy0 = std::max(0, iy - d);
                const auto iy1 = std::min(height - 1, iy + d);
                const auto ix0 = std::max(0, ix - d);
                const auto ix1 = std::min(width - 1, ix + d);
                for (int jy = iy0; jy <= iy1; ++jy) {
                    for (int jx = ix0; jx <= ix1; ++jx) {
                        const auto dy = jy - iy;
                        const auto dx = jx - ix;
                        if (dx * dx + dy * dy <= d * d) {
                            expanded[static_cast<std::size_t>(jy) * width + jx] = 0;
                        }
                    }
                }
            }
        }
        result.data = std::move(expanded);
    }

    return result;
}

namespace {

    constexpr unsigned char kObstacleGray = 0;
    constexpr unsigned char kUnknownGray  = 205;
    constexpr unsigned char kFreeGray     = 254;

} // namespace

auto save_pgm_yaml(const std::string& output_prefix, const GridMapResult& result)
    -> std::expected<void, std::string> {
    const auto pgm_path  = output_prefix + ".pgm";
    const auto yaml_path = output_prefix + ".yaml";

    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) return std::unexpected(std::format("Cannot open file: {}", pgm_path));
    pgm << "P5\n" << result.width << ' ' << result.height << "\n255\n";
    for (int row = 0; row < result.height; ++row) {
        const auto iy = result.height - 1 - row; // 行 0 = 世界系 y 最大
        for (int ix = 0; ix < result.width; ++ix) {
            const auto v    = result.data[static_cast<std::size_t>(iy) * result.width + ix];
            const auto gray = v < 0 ? kUnknownGray : (v == 0 ? kObstacleGray : kFreeGray);
            pgm.put(static_cast<char>(gray));
        }
    }
    if (!pgm) return std::unexpected(std::format("Failed writing PGM: {}", pgm_path));

    std::ofstream yaml(yaml_path);
    if (!yaml) return std::unexpected(std::format("Cannot open file: {}", yaml_path));
    const auto image_name = std::filesystem::path(pgm_path).filename().string();
    yaml << "image: " << image_name << "\n"
         << "mode: trinary\n"
         << "resolution: " << result.resolution << "\n"
         << "origin: [" << result.origin_x << ", " << result.origin_y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
    if (!yaml) return std::unexpected(std::format("Failed writing YAML: {}", yaml_path));
    return { };
}

} // namespace radar_lidar::grid_map
