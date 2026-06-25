#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <gtest/gtest.h>

#include "radar_lidar/map_data.hpp"

namespace {

// 生成一个临时 PCD 文件用于测试
std::string make_test_pcd(const std::string& path, int n_points, float spacing = 0.2f) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width    = n_points;
    cloud.height   = 1;
    cloud.is_dense = true;
    int side       = static_cast<int>(std::sqrt(static_cast<float>(n_points)));
    for (int i = 0; i < side; ++i) {
        for (int j = 0; j < side; ++j) {
            cloud.emplace_back(i * spacing, j * spacing, 0.0f);
        }
    }
    cloud.width = cloud.size();
    pcl::io::savePCDFileBinary(path, cloud);
    return path;
}

class MapDataTest : public ::testing::Test {
protected:
    void SetUp() override {
        test_pcd_ = "/tmp/radar_test_map.pcd";
        make_test_pcd(test_pcd_, 100, 0.2f);
    }
    void TearDown() override { std::filesystem::remove(test_pcd_); }
    std::string test_pcd_;
};

} // namespace

TEST_F(MapDataTest, LoadValidPCD) {
    auto result = radar::MapData::Load(test_pcd_, 0.1);
    ASSERT_TRUE(result.has_value()) << result.error();
    auto map = *result;
    EXPECT_GT(map->size(), 0u);
}

TEST_F(MapDataTest, LoadNonexistentPCD) {
    auto result = radar::MapData::Load("/tmp/nonexistent_map.pcd", 0.1);
    EXPECT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("Failed to load"), std::string::npos);
}

TEST_F(MapDataTest, VoxelDownsampling) {
    auto result = radar::MapData::Load(test_pcd_, 0.1);
    ASSERT_TRUE(result.has_value());
    auto map = *result;
    EXPECT_GT(map->size(), 0u);

    auto result2 = radar::MapData::Load(test_pcd_, 0.5);
    ASSERT_TRUE(result2.has_value());
    auto map2 = *result2;
    EXPECT_LT(map2->size(), map->size());
}

TEST_F(MapDataTest, KdTreeAccessible) {
    auto result = radar::MapData::Load(test_pcd_, 0.1);
    ASSERT_TRUE(result.has_value());
    auto map = *result;

    const auto& tree = map->sgicp_tree();
    EXPECT_EQ(tree.kdtree.indices.size(), map->size());

    // 验证 PCL KdTree 可用
    const auto& pcl_tree = map->pcl_tree();
    std::vector<int> idx(1);
    std::vector<float> dist(1);
    pcl::PointXYZ query(0.0f, 0.0f, 0.0f);
    int found = pcl_tree.nearestKSearch(query, 1, idx, dist);
    EXPECT_EQ(found, 1);
    EXPECT_EQ(idx[0], 0);
}
