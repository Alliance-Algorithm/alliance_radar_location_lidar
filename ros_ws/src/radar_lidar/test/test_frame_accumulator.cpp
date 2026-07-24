#include <gtest/gtest.h>

#include "radar_lidar/data_format.hpp"
#include "radar_lidar/frame_accumulator.hpp"

namespace {

const Eigen::Vector3d kP1 { 1, 0, 0 };
const Eigen::Vector3d kP2 { 2, 0, 0 };
const Eigen::Vector3d kP3 { 3, 0, 0 };
const Eigen::Vector3d kP4 { 4, 0, 0 };

TEST(FrameAccumulatorTest, SlidesWindowEvictsOldest) {
    radar_lidar::frame_accumulator::FrameAccumulator acc(3);
    acc.push({ kP1 });
    acc.push({ kP2 });
    acc.push({ kP3 });
    EXPECT_EQ(acc.size(), 3u);

    acc.push({ kP4 });
    EXPECT_EQ(acc.size(), 3u);

    auto pts = acc.all_points();
    ASSERT_EQ(pts.size(), 3u);
    // frames 2,3,4 retained; frame 1 evicted
    EXPECT_TRUE(pts[0].isApprox(kP2));
    EXPECT_TRUE(pts[1].isApprox(kP3));
    EXPECT_TRUE(pts[2].isApprox(kP4));
}

TEST(FrameAccumulatorTest, ClearEmpties) {
    radar_lidar::frame_accumulator::FrameAccumulator acc(5);
    acc.push({ kP1 });
    acc.push({ kP2 });

    acc.clear();
    EXPECT_EQ(acc.size(), 0u);
    EXPECT_TRUE(acc.all_points().empty());
}

TEST(FrameAccumulatorTest, EmptyAccumulator) {
    radar_lidar::frame_accumulator::FrameAccumulator acc;

    EXPECT_EQ(acc.size(), 0u);
    EXPECT_TRUE(acc.all_points().empty());
}

} // namespace
