#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gtest/gtest.h>

#include "radar_lidar/cluster.hpp"
#include "radar_lidar/dynamic_cloud.hpp"
#include "radar_lidar/types.hpp"

namespace {

auto make_wall(int nx, int ny, double step) -> pcl::PointCloud<pcl::PointXYZ>::Ptr {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(static_cast<size_t>(nx) * ny);
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j)
            cloud->emplace_back(i * step, j * step, 0.0);
    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;
    return cloud;
}

} // namespace

TEST(DynamicCloudTest, ExtractsObstacleFromWall) {
    auto map = make_wall(10, 10, 0.5);

    radar::types::PointCloud scan;
    for (const auto& pt : map->points)
        scan.emplace_back(pt.x, pt.y, pt.z);
    for (int i = 0; i < 10; ++i)
        scan.emplace_back(5.0 + i * 0.05, 5.0, 0.5);

    radar::DynamicCloudConfig cfg;
    cfg.distance_threshold = 0.1;
    cfg.num_threads        = 2;
    cfg.accumulate_frames  = 0;
    cfg.use_roi            = false;

    radar::DynamicCloudStage stage(cfg);
    stage.set_map(map);

    auto result = stage.process(scan);
    ASSERT_TRUE(result.has_value()) << result.error();

    EXPECT_EQ(result->size(), 10u);
    for (const auto& p : *result) {
        EXPECT_NEAR(p.x(), 5.0, 0.5);
        EXPECT_NEAR(p.y(), 5.0, 0.1);
        EXPECT_NEAR(p.z(), 0.5, 0.1);
    }
}

TEST(DynamicCloudTest, EmptyScanReturnsError) {
    auto map = make_wall(5, 5, 0.5);

    radar::DynamicCloudConfig cfg;
    cfg.accumulate_frames = 0;
    radar::DynamicCloudStage stage(cfg);
    stage.set_map(map);

    radar::types::PointCloud empty;
    auto result = stage.process(empty);
    EXPECT_FALSE(result.has_value());
}

TEST(DynamicCloudTest, NoMapReturnsError) {
    radar::DynamicCloudConfig cfg;
    radar::DynamicCloudStage stage(cfg);

    radar::types::PointCloud scan;
    scan.emplace_back(1.0, 2.0, 3.0);
    auto result = stage.process(scan);
    EXPECT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("Map not set"), std::string::npos);
}

TEST(ClusterTest, SingleCluster) {
    radar::types::PointCloud points;
    for (int i = 0; i < 10; ++i)
        points.emplace_back(1.0 + i * 0.05, 2.0, 3.0);

    radar::ClusterConfig cfg;
    cfg.cluster_tolerance = 0.25;
    cfg.min_cluster_size  = 5;
    cfg.max_cluster_size  = 1000;

    radar::ClusterStage stage(cfg);
    auto result = stage.process(points);
    ASSERT_TRUE(result.has_value()) << result.error();
    ASSERT_EQ(result->size(), 1u);

    const auto& cluster = (*result)[0];
    EXPECT_EQ(cluster.point_count, 10);
    EXPECT_NEAR(cluster.centroid.x(), 1.225, 0.1);
    EXPECT_NEAR(cluster.centroid.y(), 2.0, 0.1);
    EXPECT_NEAR(cluster.centroid.z(), 3.0, 0.1);

    EXPECT_NEAR(cluster.min_bound.x(), 1.0, 0.05);
    EXPECT_NEAR(cluster.max_bound.x(), 1.45, 0.05);
}

TEST(ClusterTest, TwoClusters) {
    radar::types::PointCloud points;
    for (int i = 0; i < 10; ++i)
        points.emplace_back(i * 0.05, 0.0, 0.0);
    for (int i = 0; i < 10; ++i)
        points.emplace_back(10.0 + i * 0.05, 0.0, 0.0);

    radar::ClusterConfig cfg;
    cfg.cluster_tolerance = 0.25;
    cfg.min_cluster_size  = 5;
    cfg.max_cluster_size  = 1000;

    radar::ClusterStage stage(cfg);
    auto result = stage.process(points);
    ASSERT_TRUE(result.has_value()) << result.error();
    EXPECT_EQ(result->size(), 2u);
}

TEST(ClusterTest, EmptyInputReturnsEmpty) {
    radar::ClusterConfig cfg;
    radar::ClusterStage stage(cfg);

    radar::types::PointCloud empty;
    auto result = stage.process(empty);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->empty());
}

TEST(ClusterTest, TooFewPointsFiltered) {
    radar::types::PointCloud points;
    points.emplace_back(0, 0, 0);
    points.emplace_back(0.1, 0, 0);
    points.emplace_back(0.2, 0, 0);

    radar::ClusterConfig cfg;
    cfg.cluster_tolerance = 0.25;
    cfg.min_cluster_size  = 5;
    cfg.max_cluster_size  = 1000;

    radar::ClusterStage stage(cfg);
    auto result = stage.process(points);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->empty());
}
