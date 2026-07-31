# Global Grid Map Tool Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Offline CLI tool converting a registered global PCD (e.g. `model/generated/jinan_field_map_reg.pcd`) into a `map_server`-compatible PGM + YAML grid map, using height-range obstacle classification inspired by `rmcs-local-map`.

**Architecture:** ROS-free core algorithm in `tools/radar_lidar/grid_map.{hpp,cpp}` (compiled into `radar_lidar_offline_lib`), thin CLI entry `tools/grid_map_tool.cpp`, gtest in `test/test_grid_map.cpp`. Mirrors the existing `offline_registration` / `registration_tool` split.

**Tech Stack:** C++23, PCL (io, point_cloud), Eigen, gtest, colcon/ament.

## Global Constraints

- C++23, format/std::expected/std::println style, no third-party beyond PCL/Eigen (see existing `registration_tool.cpp`).
- Grid cell classification: `count >= min_points && z_max - z_min > height_threshold` -> obstacle (0); `count >= 1` -> free (100); otherwise unknown (-1).
- Defaults: `resolution=0.05`, `height_threshold=0.3`, `min_points=3`, `dilate=0`.
- PGM row 0 = largest world y; YAML `origin: [x_min, y_min, 0]`, `negate: 0`, `occupied_thresh: 0.65`, `free_thresh: 0.196`; grayscale 0=occupied, 205=unknown, 254=free.
- Algorithm code must not depend on rclcpp.
- Final verification: run tool on `jinan_field_map_reg.pcd`, inspect PNG, archive `.pgm` + `.yaml` to `model/generated/`.

---

### Task 1: Core rasterize algorithm (hpp/cpp + gtest)

**Files:**
- Create: `ros_ws/src/radar_lidar/tools/radar_lidar/grid_map.hpp`
- Create: `ros_ws/src/radar_lidar/tools/radar_lidar/grid_map.cpp`
- Create: `ros_ws/src/radar_lidar/test/test_grid_map.cpp`
- Modify: `ros_ws/src/radar_lidar/CMakeLists.txt` (offline_lib sources + test source)

**Interfaces:**
- Produces (used by Tasks 2-3):
  - `namespace radar_lidar::grid_map`
  - `struct GridMapParams { double resolution = 0.05; double height_threshold = 0.3; int min_points = 3; int dilate = 0; };`
  - `struct Bounds { double x_min = 0.0, y_min = 0.0, x_max = 0.0, y_max = 0.0; };`
  - `struct GridMapResult { int width = 0, height = 0; double origin_x = 0.0, origin_y = 0.0; double resolution = 0.05; std::vector<int8_t> data; };` (row-major, row iy = world y_min + iy*res; 0=obstacle, 100=free, -1=unknown)
  - `auto rasterize(const pcl::PointCloud<pcl::PointXYZ>& cloud, const GridMapParams& params, const std::optional<Bounds>& bounds) -> std::expected<GridMapResult, std::string>;`

- [ ] **Step 1: Write the failing tests**

`test/test_grid_map.cpp`:

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <expected>
#include <optional>

#include <gtest/gtest.h>

#include "radar_lidar/grid_map.hpp"

using radar_lidar::grid_map::Bounds;
using radar_lidar::grid_map::GridMapParams;
using radar_lidar::grid_map::GridMapResult;
using radar_lidar::grid_map::rasterize;

namespace {

// 地面: z=0 网格; 墙: x 在 [wall_x0, wall_x1) 内的竖直面 z 从 0 到 wall_height
auto make_ground_and_wall(double wall_x0, double wall_x1, double wall_height)
    -> pcl::PointCloud<pcl::PointXYZ>::Ptr {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(400);
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 20; ++j) {
            cloud->emplace_back(0.025f + 0.05f * i, 0.025f + 0.05f * j, 0.0f);
        }
    }
    for (float z = 0.0f; z <= wall_height; z += 0.1f) {
        for (float x = static_cast<float>(wall_x0); x < wall_x1; x += 0.02f) {
            cloud->emplace_back(x, 0.5f, z);
        }
    }
    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;
    return cloud;
}

} // namespace

// 地面: 20x20 网格覆盖 [0,1]x[0,1] (每格中心一个点, z=0);
// 墙: x 在 [wall_x0, wall_x1) 内的竖直面 z 从 0 到 wall_height;
// bounds 取 {0,0,2,2} (40x40 格), 留出 [1,2]x[1,2] 空区域验证 unknown
TEST(GridMapTest, GroundAndWall) {
    const auto cloud = make_ground_and_wall(0.5, 0.6, 1.0);
    const auto result = rasterize(*cloud, GridMapParams {}, std::optional<Bounds> { { 0, 0, 2, 2 } });
    ASSERT_TRUE(result.has_value()) << result.error();
    const auto& map = *result;
    EXPECT_EQ(map.width, 40);
    EXPECT_EQ(map.height, 40);
    EXPECT_NEAR(map.origin_x, 0.0, 1e-9);
    EXPECT_NEAR(map.origin_y, 0.0, 1e-9);
    const auto cell = [&](int ix, int iy) { return map.data[iy * map.width + ix]; };
    EXPECT_EQ(cell(10, 10), 0) << "wall column (ix=10) should be obstacle";
    EXPECT_EQ(cell(11, 10), 0) << "wall column (ix=11) should be obstacle";
    EXPECT_EQ(cell(5, 10), 100) << "ground cell should be free";
    EXPECT_EQ(cell(30, 30), -1) << "empty corner (ix=30,iy=30) should be unknown";
}

TEST(GridMapTest, HeightThresholdBoundary) {
    const auto short_wall = make_ground_and_wall(0.5, 0.6, 0.29);
    const auto short_map  = rasterize(*short_wall, GridMapParams {}, std::optional<Bounds> { { 0, 0, 2, 2 } });
    ASSERT_TRUE(short_map.has_value());
    EXPECT_EQ(short_map->data[10 * 40 + 10], 100) << "wall 0.29m < threshold 0.3 -> free";

    const auto tall_wall = make_ground_and_wall(0.5, 0.6, 0.31);
    const auto tall_map  = rasterize(*tall_wall, GridMapParams {}, std::optional<Bounds> { { 0, 0, 2, 2 } });
    ASSERT_TRUE(tall_map.has_value());
    EXPECT_EQ(tall_map->data[10 * 40 + 10], 0) << "wall 0.31m > threshold 0.3 -> obstacle";
}

TEST(GridMapTest, MinPointsFiltersIsolatedHighPoint) {
    // 单个离群点(很高)在空白区域, 点数不足 min_points -> 判 free 而非 obstacle
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->emplace_back(0.025f, 0.025f, 2.0f);
    cloud->width = 1;
    cloud->height = 1;
    cloud->is_dense = true;
    const auto result = rasterize(*cloud, GridMapParams { .min_points = 3 },
        std::optional<Bounds> { { 0, 0, 2, 2 } });
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->data[0], 100) << "single point below min_points -> free, not obstacle";
}

TEST(GridMapTest, DilateExpandsObstacles) {
    const auto cloud = make_ground_and_wall(0.5, 0.6, 1.0);
    GridMapParams params { .dilate = 1 };
    const auto result = rasterize(*cloud, params, std::optional<Bounds> { { 0, 0, 2, 2 } });
    ASSERT_TRUE(result.has_value());
    const auto& map = *result;
    const auto cell = [&](int ix, int iy) { return map.data[iy * map.width + ix]; };
    EXPECT_EQ(cell(9, 10), 0) << "cell left of wall within dilate=1 -> obstacle";
    EXPECT_EQ(cell(12, 10), 0) << "cell right of wall within dilate=1 -> obstacle";
    EXPECT_EQ(cell(10, 9), 0) << "cell below wall center within dilate=1 -> obstacle";
    EXPECT_EQ(cell(5, 5), 100) << "far ground cell unchanged";
}

TEST(GridMapTest, CoordinateMappingYFlip) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->emplace_back(0.025f, 0.975f, 2.0f); // 世界系 y 最大处
    cloud->width = 1;
    cloud->height = 1;
    cloud->is_dense = true;
    const auto result = rasterize(*cloud, GridMapParams { .min_points = 1 },
        std::optional<Bounds> { { 0, 0, 2, 2 } });
    ASSERT_TRUE(result.has_value());
    // iy = floor(0.975/0.05) = 19 (y 最大); PGM 行 0 = y 最大, 由写文件逻辑翻转
    EXPECT_EQ(result->data[19 * 40 + 0], 100);
    EXPECT_EQ(result->data[0], -1) << "y_min corner stays unknown";
}

TEST(GridMapTest, EmptyCloudWithBoundsGivesAllUnknown) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const auto result = rasterize(*cloud, GridMapParams {}, std::optional<Bounds> { { 0, 0, 2, 2 } });
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(std::ranges::all_of(result->data, [](int8_t v) { return v == -1; }));
}

TEST(GridMapTest, EmptyCloudWithoutBoundsFails) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const auto result = rasterize(*cloud, GridMapParams {}, std::nullopt);
    ASSERT_FALSE(result.has_value());
}

TEST(GridMapTest, InvalidParamsFail) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->emplace_back(0.0f, 0.0f, 0.0f);
    cloud->width = 1;
    cloud->height = 1;
    cloud->is_dense = true;
    EXPECT_FALSE(rasterize(*cloud, GridMapParams { .resolution = 0.0 }, std::nullopt).has_value());
    EXPECT_FALSE(rasterize(*cloud, GridMapParams { .min_points = 0 }, std::nullopt).has_value());
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run:
```bash
colcon build --packages-select radar_lidar --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5
```
Expected: compile error `grid_map.hpp` not found (include not yet created).

- [ ] **Step 3: Create `tools/radar_lidar/grid_map.hpp`**

```cpp
#pragma once

#include <cstdint>
#include <expected>
#include <optional>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace radar_lidar::grid_map {

struct GridMapParams {
    double resolution       = 0.05; // 每格边长 (m)
    double height_threshold = 0.3;  // 格内 z_max - z_min 超过此值判障碍
    int min_points          = 3;    // 判定障碍所需最少点数
    int dilate              = 0;    // 障碍膨胀半径 (格)
};

struct Bounds {
    double x_min = 0.0, y_min = 0.0, x_max = 0.0, y_max = 0.0;
};

struct GridMapResult {
    int width = 0, height = 0;
    double origin_x = 0.0, origin_y = 0.0; // 世界系左下角
    double resolution = 0.05;
    // row-major, data[iy * width + ix], iy=0 对应世界系 y_min
    // 0 = 障碍, 100 = 空闲, -1 = 未知
    std::vector<int8_t> data;
};

// 将稠密全局点云栅格化为 2D 占用网格。
// bounds 为空时使用点云 bbox (扩到分辨率整数倍); 点云为空且无 bounds 时报错。
auto rasterize(const pcl::PointCloud<pcl::PointXYZ>& cloud, const GridMapParams& params,
    const std::optional<Bounds>& bounds) -> std::expected<GridMapResult, std::string>;

} // namespace radar_lidar::grid_map
```

- [ ] **Step 4: Create `tools/radar_lidar/grid_map.cpp`**

```cpp
#include "radar_lidar/grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>

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
        x_min = floor_to_resolution(cloud.points.front().x, res);
        x_max = ceil_to_resolution(cloud.points.front().x, res);
        y_min = floor_to_resolution(cloud.points.front().y, res);
        y_max = ceil_to_resolution(cloud.points.front().y, res);
        for (const auto& pt : cloud.points) {
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
        const auto idx = index_of(pt.x, pt.y);
        if (idx == static_cast<std::size_t>(-1)) continue;
        auto& cell      = cells[idx];
        cell.count      += 1;
        cell.z_min       = std::min(cell.z_min, pt.z);
        cell.z_max       = std::max(cell.z_max, pt.z);
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
        const auto d = params.dilate;
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

} // namespace radar_lidar::grid_map
```

- [ ] **Step 5: Register sources in `CMakeLists.txt`**

Add `tools/radar_lidar/grid_map.cpp` to the `radar_lidar_offline_lib` source list (after `tools/offline_test_node.cpp`), and add `test/test_grid_map.cpp` to the `radar_tests` source list (after `test/test_offline_visualization.cpp`). The existing `target_include_directories(... PRIVATE tools)` already exposes `radar_lidar/grid_map.hpp`.

- [ ] **Step 6: Run tests to verify they pass**

Run:
```bash
colcon build --packages-select radar_lidar --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5
colcon test --packages-select radar_lidar --ctest-args -R radar_tests 2>&1 | tail -3
colcon test-result --verbose 2>&1 | grep -E "grid_map|passed|failed" | head -20
```
Expected: all 9 `GridMapTest.*` cases PASS.

- [ ] **Step 7: Commit**

```bash
git add ros_ws/src/radar_lidar/tools/radar_lidar/grid_map.hpp \
        ros_ws/src/radar_lidar/tools/radar_lidar/grid_map.cpp \
        ros_ws/src/radar_lidar/test/test_grid_map.cpp \
        ros_ws/src/radar_lidar/CMakeLists.txt
git commit -m "feat(radar_lidar): rasterize global PCD into occupancy grid"
```

---

### Task 2: PGM + YAML serialization

**Files:**
- Modify: `ros_ws/src/radar_lidar/tools/radar_lidar/grid_map.hpp` (add `save_pgm_yaml` declaration)
- Modify: `ros_ws/src/radar_lidar/tools/radar_lidar/grid_map.cpp` (add implementation)
- Modify: `ros_ws/src/radar_lidar/test/test_grid_map.cpp` (add serialization tests)

**Interfaces:**
- Consumes: `GridMapResult` from Task 1.
- Produces (used by Task 3): `auto save_pgm_yaml(const std::string& output_prefix, const GridMapResult& result) -> std::expected<void, std::string>;` — writes `<output_prefix>.pgm` (P5 binary) and `<output_prefix>.yaml`.

- [ ] **Step 1: Write the failing tests**

Append to `test/test_grid_map.cpp` (add `#include <filesystem>`, `#include <fstream>`, `#include <sstream>`, `#include <string>`):

```cpp
namespace {

auto read_file(const std::filesystem::path& path) -> std::string {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

} // namespace

TEST(GridMapTest, SavePgmYamlWritesValidFiles) {
    GridMapResult map;
    map.width      = 4;
    map.height     = 4;
    map.origin_x   = -0.1;
    map.origin_y   = -0.2;
    map.resolution = 0.05;
    // iy=0: 空闲,空闲,未知,未知; iy=1..2: 全未知; iy=3 (y 最大): 障碍,障碍,未知,未知
    map.data = { 100, 100, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, -1, -1 };

    const auto tmp = std::filesystem::temp_directory_path() / "grid_map_test";
    std::filesystem::create_directories(tmp);
    const auto prefix = (tmp / "test_map").string();
    const auto result = save_pgm_yaml(prefix, map);
    ASSERT_TRUE(result.has_value()) << result.error();

    const auto pgm = read_file(tmp / "test_map.pgm");
    // "P5\n4 4\n255\n" 头部 = 11 字节, 之后 16 个灰度字节
    ASSERT_GT(pgm.size(), 11u);
    EXPECT_EQ(pgm.substr(0, 10), "P5\n4 4\n255\n");
    const auto body = pgm.substr(11);
    ASSERT_EQ(body.size(), 16u);
    // 行 0 = 世界系 y 最大 -> data 的 iy=3 (两障碍格 0,0)
    EXPECT_EQ(static_cast<unsigned char>(body[0]), 0u);
    EXPECT_EQ(static_cast<unsigned char>(body[1]), 0u);
    // 最后一行 = 世界系 y 最小 -> data 的 iy=0 (空闲格 100 -> 254)
    EXPECT_EQ(static_cast<unsigned char>(body[12]), 254u);
    EXPECT_EQ(static_cast<unsigned char>(body[13]), 254u);
    // 未知格 -> 205
    EXPECT_EQ(static_cast<unsigned char>(body[2]), 205u);

    const auto yaml = read_file(tmp / "test_map.yaml");
    EXPECT_TRUE(yaml.find("image: test_map.pgm") != std::string::npos);
    EXPECT_TRUE(yaml.find("resolution: 0.05") != std::string::npos);
    EXPECT_TRUE(yaml.find("origin: [-0.1, -0.2, 0.0]") != std::string::npos);
    EXPECT_TRUE(yaml.find("negate: 0") != std::string::npos);
    EXPECT_TRUE(yaml.find("occupied_thresh: 0.65") != std::string::npos);
    EXPECT_TRUE(yaml.find("free_thresh: 0.196") != std::string::npos);
    EXPECT_TRUE(yaml.find("mode: trinary") != std::string::npos);
}

TEST(GridMapTest, SavePgmYamlFailsOnBadPath) {
    GridMapResult map;
    map.width    = 1;
    map.height   = 1;
    map.data     = { -1 };
    map.resolution = 0.05;
    const auto result = save_pgm_yaml("/nonexistent_dir_xyz/out", map);
    EXPECT_FALSE(result.has_value());
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run:
```bash
colcon build --packages-select radar_lidar --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5
colcon test --packages-select radar_lidar --ctest-args -R radar_tests 2>&1 | tail -3
```
Expected: compile error `save_pgm_yaml` not declared.

- [ ] **Step 3: Add declaration to `grid_map.hpp`**

After `rasterize` declaration:

```cpp
// 写 map_server 标准 PGM(P5)+YAML。PGM 行 0 = 世界系 y 最大。
auto save_pgm_yaml(const std::string& output_prefix, const GridMapResult& result)
    -> std::expected<void, std::string>;
```

- [ ] **Step 4: Add implementation to `grid_map.cpp`**

Add includes: `<filesystem>`, `<fstream>`, `<string>`.

```cpp
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
            const auto v = result.data[static_cast<std::size_t>(iy) * result.width + ix];
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run:
```bash
colcon build --packages-select radar_lidar --cmake-args -DBUILD_TESTING=ON 2>&1 | tail -5
colcon test --packages-select radar_lidar --ctest-args -R radar_tests 2>&1 | tail -3
colcon test-result --verbose 2>&1 | grep -E "GridMapTest|passed|failed" | head -20
```
Expected: all 11 `GridMapTest.*` cases PASS.

- [ ] **Step 6: Commit**

```bash
git add ros_ws/src/radar_lidar/tools/radar_lidar/grid_map.hpp \
        ros_ws/src/radar_lidar/tools/radar_lidar/grid_map.cpp \
        ros_ws/src/radar_lidar/test/test_grid_map.cpp
git commit -m "feat(radar_lidar): serialize grid map to PGM + YAML (map_server format)"
```

---

### Task 3: CLI tool `grid_map_tool`

**Files:**
- Create: `ros_ws/src/radar_lidar/tools/grid_map_tool.cpp`
- Modify: `ros_ws/src/radar_lidar/CMakeLists.txt` (executable target + install)

**Interfaces:**
- Consumes: `grid_map::rasterize`, `grid_map::save_pgm_yaml` from Tasks 1-2.
- Produces: `grid_map_tool` executable with CLI below.

```
Usage: grid_map_tool <map.pcd> [options]
  --output <prefix>         output prefix -> <prefix>.pgm + <prefix>.yaml (default map)
  --resolution <float>      cell size in meters (default 0.05)
  --height-threshold <f>    z-spread obstacle threshold in meters (default 0.3)
  --min-points <int>        minimum points per cell (default 3)
  --dilate <int>            obstacle dilation radius in cells (default 0)
  --bounds xmin ymin xmax ymax   explicit map bounds (default: cloud bbox)
  --verbose                 print stats
```

- [ ] **Step 1: Create `tools/grid_map_tool.cpp`**

Follow the `registration_tool.cpp` style exactly (std::from_chars parsers, `checked_assign_bounded`, `std::expected`, `std::println`):

```cpp
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
```

- [ ] **Step 2: Register target in `CMakeLists.txt`**

After the `offline_test_node` target block:

```cmake
ament_auto_add_executable(grid_map_tool
    tools/grid_map_tool.cpp
)
target_link_libraries(grid_map_tool ${PROJECT_NAME}_offline_lib)
```

And add `grid_map_tool` to the `install(TARGETS ...)` list at the bottom.

- [ ] **Step 3: Build**

Run:
```bash
colcon build --packages-select radar_lidar 2>&1 | tail -5
```
Expected: build OK, executable at `install/radar_lidar/lib/radar_lidar/grid_map_tool`.

- [ ] **Step 4: Smoke test CLI**

Run:
```bash
./install/radar_lidar/lib/radar_lidar/grid_map_tool 2>&1; echo "exit=$?"
./install/radar_lidar/lib/radar_lidar/grid_map_tool /nonexistent.pcd --verbose 2>&1; echo "exit=$?"
./install/radar_lidar/lib/radar_lidar/grid_map_tool --badflag 2>&1; echo "exit=$?"
```
Expected: usage text exit 1; load-fail error exit 1; unknown-arg error exit 1.

- [ ] **Step 5: Commit**

```bash
git add ros_ws/src/radar_lidar/tools/grid_map_tool.cpp ros_ws/src/radar_lidar/CMakeLists.txt
git commit -m "feat(radar_lidar): add grid_map_tool CLI for PCD -> grid map"
```

---

### Task 4: Real-data verification and archiving

**Files:**
- Output (archived): `model/generated/jinan_field_map_reg.pgm`, `model/generated/jinan_field_map_reg.yaml`

- [ ] **Step 1: Run tool on the registered arena map**

Run (paths relative to the repo root; inside the dev container use `/workspace` instead of the host path):
```bash
./ros_ws/install/radar_lidar/lib/radar_lidar/grid_map_tool \
  model/generated/jinan_field_map_reg.pcd \
  --output model/generated/jinan_field_map_reg --verbose
```
Expected: stats printed (~28.2 m x 15.2 m -> 564 x 304 cells), files written. Sanity-check the obstacle cell count is neither 0 nor the entire map.

- [ ] **Step 2: Render PNG for inspection**

Run:
```bash
python3 - <<'EOF'
from pathlib import Path

raw = Path('model/generated/jinan_field_map_reg.pgm').read_bytes()
head, w, h, _ = raw.split(b'\n', 3)
assert head == b'P5'
w, h = int(w), int(h)
header_len = len(b'P5\n') + len(f'{w} {h}\n'.encode()) + len(b'255\n')
data = raw[header_len:]
assert len(data) == w * h, (len(data), w * h)
try:
    import numpy as np
    from PIL import Image
except ImportError:
    raise SystemExit('need numpy+PIL: pip install numpy pillow')
img = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
Image.fromarray(img).save('model/generated/jinan_field_map_reg.png')
print('saved png', w, h)
EOF
```
Expected: PNG opens; arena outline (walls/covers) visible as black on gray/white. If obstacle cells look noisy, retry with `--dilate 1` or raise `--min-points` (e.g. 5) and re-run Task 4 Step 1 before archiving.

- [ ] **Step 3: Verify map_server compatibility (optional but recommended)**

If a ROS 2 environment is available:
```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=model/generated/jinan_field_map_reg.yaml &
sleep 2
ros2 topic echo /map --once | head -5
```
Expected: `/map` publishes `nav_msgs/msg/OccupancyGrid` with matching `info.resolution` / `info.origin`.

- [ ] **Step 4: Commit archived outputs**

```bash
git add model/generated/jinan_field_map_reg.pgm model/generated/jinan_field_map_reg.yaml
git commit -m "chore(model): archive global grid map for jinan field"
```

---

## Self-Review Notes

- Spec coverage: rasterize algorithm (Task 1), PGM/YAML map_server convention (Task 2), CLI + flags (Task 3), real-data verification + archiving (Task 4) — all spec sections covered.
- Type consistency: `GridMapParams` / `Bounds` / `GridMapResult` / `rasterize` / `save_pgm_yaml` signatures identical across tasks; `data` semantics (0/100/-1, row 0 = y_min) consistent between rasterize, save_pgm_yaml, and tests.
- YAML `origin` prints via std::ostream with default precision — `-0.1` prints as `-0.1`; test asserts `origin: [-0.1, -0.2, 0.0]`. For the real map, `origin_x/origin_y` are whole-ish values (e.g. -14.1, -7.6); fine for map_server.
