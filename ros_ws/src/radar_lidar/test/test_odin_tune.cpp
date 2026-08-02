#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "radar_lidar/odin_tune/pose_buffer.hpp"
#include "radar_lidar/odin_tune/background_model.hpp"
#include "radar_lidar/odin_tune/frame_differencer.hpp"
#include "radar_lidar/odin_tune/map_differencer.hpp"
#include "radar_lidar/data_format.hpp"

namespace {

auto make_pose(double x, double y, double yaw) -> Eigen::Isometry3d {
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation()     = Eigen::Vector3d(x, y, 0.0);
    p.linear()          = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return p;
}

} // namespace

TEST(PoseBufferTest, LookupExactStamp) {
    radar_lidar::odin_tune::PoseBuffer buf;
    const auto pose = make_pose(1.0, 2.0, 0.3);
    buf.add(100, pose);

    auto result = buf.lookup(100);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->isApprox(pose, 1e-9));
}

TEST(PoseBufferTest, LookupNearest) {
    radar_lidar::odin_tune::PoseBuffer buf;
    buf.add(100, make_pose(0.0, 0.0, 0.0));
    buf.add(200, make_pose(1.0, 1.0, 0.0));

    auto result = buf.lookup(199);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->translation().x(), 1.0, 1e-9);
}

TEST(PoseBufferTest, LookupTooOldReturnsNullopt) {
    radar_lidar::odin_tune::PoseBuffer buf(1'000);  // 1us 跨度
    buf.add(100, make_pose(0.0, 0.0, 0.0));

    EXPECT_FALSE(buf.lookup(100 + 1'001).has_value());
}

TEST(PoseBufferTest, ClearEmptiesBuffer) {
    radar_lidar::odin_tune::PoseBuffer buf;
    buf.add(100, make_pose(0.0, 0.0, 0.0));
    buf.clear();

    EXPECT_FALSE(buf.lookup(100).has_value());
}

TEST(BackgroundModelTest, EmptyModelAlignsEmpty) {
    radar_lidar::odin_tune::BackgroundModel model(5);

    auto cloud = model.align_to(Eigen::Isometry3d::Identity());
    EXPECT_TRUE(cloud.empty());
}

TEST(BackgroundModelTest, SamePoseKeepsPoints) {
    radar_lidar::odin_tune::BackgroundModel model(5);
    radar_lidar::types::PointCloud pts {
        Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 2.0, 0.0) };
    const auto pose = make_pose(5.0, 5.0, 0.0);
    model.add(pts, pose);

    auto cloud = model.align_to(pose);
    ASSERT_EQ(cloud.size(), 2u);
    EXPECT_NEAR(cloud[0].x(), 1.0, 1e-9);
    EXPECT_NEAR(cloud[1].y(), 2.0, 1e-9);
}

TEST(BackgroundModelTest, RelativeTransformAlignsFrames) {
    radar_lidar::odin_tune::BackgroundModel model(5);
    // 帧 A：雷达在原点，看到 (10,0,0)
    radar_lidar::types::PointCloud pts { Eigen::Vector3d(10.0, 0.0, 0.0) };
    model.add(pts, make_pose(0.0, 0.0, 0.0));
    // 帧 B：雷达移到 (5,0,0) 看向同一点 (10,0,0)（世界系），相对变换 T_B^{-1}*T_A
    const auto pose_b = make_pose(5.0, 0.0, 0.0);

    auto cloud = model.align_to(pose_b);
    ASSERT_EQ(cloud.size(), 1u);
    // 帧 A 的点 (10,0,0) 变换到 B 雷达系 = (5,0,0)
    EXPECT_NEAR(cloud[0].x(), 5.0, 1e-9);
}

TEST(BackgroundModelTest, WindowSlides) {
    radar_lidar::odin_tune::BackgroundModel model(2);
    model.add({ Eigen::Vector3d(1, 0, 0) }, make_pose(0, 0, 0));
    model.add({ Eigen::Vector3d(2, 0, 0) }, make_pose(0, 0, 0));
    model.add({ Eigen::Vector3d(3, 0, 0) }, make_pose(0, 0, 0));

    EXPECT_EQ(model.frame_count(), 2);
    auto cloud = model.align_to(make_pose(0, 0, 0));
    ASSERT_EQ(cloud.size(), 2u);
    EXPECT_NEAR(cloud[0].x(), 2.0, 1e-9);
    EXPECT_NEAR(cloud[1].x(), 3.0, 1e-9);
}

TEST(BackgroundModelTest, ClearResets) {
    radar_lidar::odin_tune::BackgroundModel model(5);
    model.add({ Eigen::Vector3d(1, 0, 0) }, make_pose(0, 0, 0));
    model.clear();

    EXPECT_EQ(model.frame_count(), 0);
    EXPECT_TRUE(model.align_to(Eigen::Isometry3d::Identity()).empty());
}

TEST(FrameDifferencerTest, EmptyBackgroundYieldsEmpty) {
    radar_lidar::odin_tune::FrameDifferencer diff(0.3);
    radar_lidar::types::PointCloud cur { Eigen::Vector3d(1, 0, 0) };

    EXPECT_TRUE(diff.differ(cur, { }).empty());
}

TEST(FrameDifferencerTest, ClosePointIsStatic) {
    radar_lidar::odin_tune::FrameDifferencer diff(0.3);
    radar_lidar::types::PointCloud cur { Eigen::Vector3d(0.0, 0.0, 0.0) };
    radar_lidar::types::PointCloud bg { Eigen::Vector3d(0.1, 0.0, 0.0) };

    EXPECT_TRUE(diff.differ(cur, bg).empty());
}

TEST(FrameDifferencerTest, FarPointIsDynamic) {
    radar_lidar::odin_tune::FrameDifferencer diff(0.3);
    radar_lidar::types::PointCloud cur { Eigen::Vector3d(0.0, 0.0, 0.0) };
    radar_lidar::types::PointCloud bg { Eigen::Vector3d(1.0, 0.0, 0.0) };

    auto result = diff.differ(cur, bg);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].x(), 0.0, 1e-9);
}

TEST(FrameDifferencerTest, ExactlyThresholdIsStatic) {
    radar_lidar::odin_tune::FrameDifferencer diff(0.3);
    radar_lidar::types::PointCloud cur { Eigen::Vector3d(0.0, 0.0, 0.0) };
    radar_lidar::types::PointCloud bg { Eigen::Vector3d(0.3, 0.0, 0.0) };

    EXPECT_TRUE(diff.differ(cur, bg).empty());
}

TEST(FrameDifferencerTest, ThresholdCanBeUpdated) {
    radar_lidar::odin_tune::FrameDifferencer diff(0.3);
    radar_lidar::types::PointCloud cur { Eigen::Vector3d(0.0, 0.0, 0.0) };
    radar_lidar::types::PointCloud bg { Eigen::Vector3d(0.5, 0.0, 0.0) };

    diff.set_distance_threshold(0.6);
    EXPECT_TRUE(diff.differ(cur, bg).empty());
}

TEST(MapDifferencerTest, StaticTargetNotInMapIsDetected) {
    // 地图背景：一面墙 (x=10, y∈[0,2])
    radar_lidar::types::PointCloud map_pts;
    for (double y = 0.0; y <= 2.0; y += 0.5)
        map_pts.emplace_back(10.0, y, 0.0);

    radar_lidar::odin_tune::MapDifferencer diff(map_pts, 0.3);

    // 静止目标：不在墙上的一个静止目标点 (x=8, y=1) —— 与地图最近距离 2m
    radar_lidar::types::PointCloud scan {
        Eigen::Vector3d(8.0, 1.0, 0.0),  // 静止目标 → 应检出
        Eigen::Vector3d(10.0, 0.5, 0.0), // 墙上的点 → 应过滤
    };
    auto result = diff.differ(scan);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].x(), 8.0, 1e-9);
}

TEST(MapDifferencerTest, StaticAndMovingBothDetected) {
    radar_lidar::types::PointCloud map_pts { Eigen::Vector3d(0.0, 0.0, 0.0) };
    radar_lidar::odin_tune::MapDifferencer diff(map_pts, 0.3);

    radar_lidar::types::PointCloud scan {
        Eigen::Vector3d(2.0, 0.0, 0.0), // 移动目标
        Eigen::Vector3d(-1.0, 0.0, 0.0), // 静止目标
    };
    auto result = diff.differ(scan);
    ASSERT_EQ(result.size(), 2u);
}

TEST(MapDifferencerTest, MapPointFiltered) {
    radar_lidar::types::PointCloud map_pts { Eigen::Vector3d(1.0, 1.0, 0.0) };
    radar_lidar::odin_tune::MapDifferencer diff(map_pts, 0.3);

    radar_lidar::types::PointCloud scan { Eigen::Vector3d(1.1, 1.0, 0.0) }; // 距地图 0.1m
    EXPECT_TRUE(diff.differ(scan).empty());
}
