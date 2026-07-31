#include <charconv>
#include <chrono>
#include <expected>
#include <format>
#include <print>
#include <string>
#include <string_view>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "radar_lidar/grid_map.hpp"

namespace {

using radar_lidar::grid_map::Bounds;
using radar_lidar::grid_map::GridMapParams;
using radar_lidar::grid_map::GridMapResult;
using radar_lidar::grid_map::rasterize;
using radar_lidar::grid_map::save_pgm_yaml;

struct Args {
    std::string map_path;
    std::string output_prefix = "map";
    double resolution       = 0.05;
    double height_threshold = 0.3;
    int min_points          = 3;
    int dilate              = 0;
    std::optional<Bounds> bounds;
    bool verbose = false;
};

auto usage(std::string_view prog) -> std::string {
    return std::format("Usage: {} <map.pcd> [options]\n"
                       "Options:\n"
                       "  --output <prefix>         output prefix -> <prefix>.pgm/.yaml (default map)\n"
                       "  --resolution <float>      cell size in meters (default 0.05)\n"
                       "  --height-threshold <f>    z-spread obstacle threshold in m (default 0.3)\n"
                       "  --min-points <int>        minimum points per cell (default 3)\n"
                       "  --dilate <int>            obstacle dilation radius in cells (default 0)\n"
                       "  --bounds xmin ymin xmax ymax   explicit map bounds (default: cloud bbox)\n"
                       "  --verbose                 print stats\n",
        prog);
}

template <typename T> auto parse_number(std::string_view sv) -> std::expected<T, std::string> {
    T value { };
    const auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), value);
    if (ec != std::errc { } || ptr != sv.data() + sv.size()) {
        return std::unexpected(std::format("Invalid number: '{}'", sv));
    }
    return value;
}

template <typename T>
auto checked_assign(T& dest, std::string_view val) -> std::expected<void, std::string> {
    auto n = parse_number<T>(val);
    if (!n) return std::unexpected(n.error());
    dest = *n;
    return { };
}

template <typename T>
auto checked_assign_bounded(T& dest, std::string_view val, std::string_view name, T min,
    bool min_exclusive) -> std::expected<void, std::string> {
    auto n = parse_number<T>(val);
    if (!n) return std::unexpected(n.error());
    if (min_exclusive ? (*n <= min) : (*n < min)) {
        return std::unexpected(
            std::format("{} must be {} {}, got '{}'", name, min_exclusive ? ">" : ">=", min, val));
    }
    dest = *n;
    return { };
}

auto parse_args(int argc, char** argv) -> std::expected<Args, std::string> {
    if (argc < 2) return std::unexpected(usage(argv[0]));

    Args args;
    args.map_path = argv[1];

    for (int i = 2; i < argc; ++i) {
        std::string_view arg = argv[i];

        if (arg == "--verbose") {
            args.verbose = true;
            continue;
        }
        if (arg == "--bounds") {
            if (i + 4 >= argc) {
                return std::unexpected(
                    std::format("ERROR: --bounds requires xmin ymin xmax ymax\n{}", usage(argv[0])));
            }
            Bounds b;
            for (auto* dest : { &b.x_min, &b.y_min, &b.x_max, &b.y_max }) {
                if (auto r = checked_assign(*dest, argv[++i]); !r) return std::unexpected(r.error());
            }
            args.bounds = b;
            continue;
        }

        if (i + 1 >= argc) {
            return std::unexpected(std::format("ERROR: {} requires a value\n{}", arg, usage(argv[0])));
        }
        const std::string val = argv[++i];

        if (arg == "--output") {
            args.output_prefix = val;
        } else if (arg == "--resolution") {
            if (auto r = checked_assign_bounded(args.resolution, val, "--resolution", 0.0, true); !r)
                return std::unexpected(r.error());
        } else if (arg == "--height-threshold") {
            if (auto r = checked_assign_bounded(args.height_threshold, val, "--height-threshold", 0.0, false); !r)
                return std::unexpected(r.error());
        } else if (arg == "--min-points") {
            if (auto r = checked_assign_bounded(args.min_points, val, "--min-points", 0, true); !r)
                return std::unexpected(r.error());
        } else if (arg == "--dilate") {
            if (auto r = checked_assign_bounded(args.dilate, val, "--dilate", 0, false); !r)
                return std::unexpected(r.error());
        } else {
            return std::unexpected(std::format("ERROR: unknown argument '{}'\n{}", arg, usage(argv[0])));
        }
    }
    return args;
}

} // namespace

int main(int argc, char** argv) {
    auto args_result = parse_args(argc, argv);
    if (!args_result) {
        std::println(stderr, "{}", args_result.error());
        return 1;
    }
    const auto& args = *args_result;

    std::println("[grid_map_tool] Loading map: {}", args.map_path);
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(args.map_path, *cloud) == -1) {
        std::println(stderr, "[grid_map_tool] ERROR: Failed to load PCD");
        return 1;
    }
    std::println("[grid_map_tool] Loaded: {} points", cloud->size());
    if (cloud->empty()) {
        std::println(stderr, "[grid_map_tool] ERROR: empty point cloud");
        return 1;
    }

    GridMapParams params;
    params.resolution       = args.resolution;
    params.height_threshold = args.height_threshold;
    params.min_points       = args.min_points;
    params.dilate           = args.dilate;

    const auto t0 = std::chrono::high_resolution_clock::now();
    auto grid_result = rasterize(*cloud, params, args.bounds);
    if (!grid_result) {
        std::println(stderr, "[grid_map_tool] ERROR: {}", grid_result.error());
        return 1;
    }
    const auto& grid = *grid_result;

    const auto t1 = std::chrono::high_resolution_clock::now();
    const auto ms  = std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (args.verbose) {
        std::size_t obstacle = 0, free = 0, unknown = 0;
        for (const auto v : grid.data) {
            if (v < 0) ++unknown;
            else if (v == 0) ++obstacle;
            else ++free;
        }
        std::println("[grid_map_tool] === Grid stats ===");
        std::println("  size:        {} x {} cells ({:.3f} m x {:.3f} m)", grid.width, grid.height,
            grid.width * grid.resolution, grid.height * grid.resolution);
        std::println("  resolution:  {:.3f} m", grid.resolution);
        std::println("  origin:      ({:.3f}, {:.3f})", grid.origin_x, grid.origin_y);
        std::println("  obstacle:    {}  free: {}  unknown: {}", obstacle, free, unknown);
        std::println("  rasterize:   {:.1f} ms", ms);
    }

    if (auto r = save_pgm_yaml(args.output_prefix, grid); !r) {
        std::println(stderr, "[grid_map_tool] ERROR: {}", r.error());
        return 1;
    }
    std::println("[grid_map_tool] Written: {}.pgm / {}.yaml", args.output_prefix, args.output_prefix);

    return 0;
}
