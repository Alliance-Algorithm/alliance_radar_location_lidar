#include <gtest/gtest.h>

#include <limits>

#include "radar_lidar/offline_visualization.hpp"

namespace {

auto make_map_cloud() -> pcl::PointCloud<pcl::PointXYZ> {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.emplace_back(0.0f, 0.0f, 0.0f);
    cloud.emplace_back(1.0f, 0.0f, 0.0f);
    cloud.emplace_back(std::numeric_limits<float>::infinity(), 0.0f, 0.0f);
    cloud.width  = cloud.size();
    cloud.height = 1;
    return cloud;
}

auto make_aligned_scan() -> radar::types::PointCloud {
    return {
        Eigen::Vector3d(0.5, 1.0, 0.0),
        Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
    };
}

} // namespace

TEST(OfflineVisualizationTest, ColoredCloudUsesRequestedBgrColor) {
    const auto cloud =
        radar::offline::make_colored_cloud(make_map_cloud(), radar::offline::kMapColorBgr);

    ASSERT_EQ(cloud.size(), 2u);
    EXPECT_EQ(cloud[0].b, radar::offline::kMapColorBgr.b);
    EXPECT_EQ(cloud[0].g, radar::offline::kMapColorBgr.g);
    EXPECT_EQ(cloud[0].r, radar::offline::kMapColorBgr.r);
}

TEST(OfflineVisualizationTest, OverlayCloudSeparatesMapAndScanColors) {
    const auto overlay = radar::offline::make_overlay_cloud(
        make_map_cloud(), make_aligned_scan(), radar::offline::kMapColorBgr,
        radar::offline::kAlignedScanColorBgr);

    ASSERT_EQ(overlay.size(), 3u);

    EXPECT_EQ(overlay[0].b, radar::offline::kMapColorBgr.b);
    EXPECT_EQ(overlay[0].g, radar::offline::kMapColorBgr.g);
    EXPECT_EQ(overlay[0].r, radar::offline::kMapColorBgr.r);

    EXPECT_EQ(overlay.back().b, radar::offline::kAlignedScanColorBgr.b);
    EXPECT_EQ(overlay.back().g, radar::offline::kAlignedScanColorBgr.g);
    EXPECT_EQ(overlay.back().r, radar::offline::kAlignedScanColorBgr.r);
}
