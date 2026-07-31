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
