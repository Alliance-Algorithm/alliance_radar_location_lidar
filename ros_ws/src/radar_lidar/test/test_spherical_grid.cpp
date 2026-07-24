#include <gtest/gtest.h>

#include <algorithm>
#include <tuple>

#include "radar_lidar/data_format.hpp"
#include "radar_lidar/spherical_grid.hpp"

namespace {

constexpr double kGridDeg = 1.0;
const Eigen::Vector3d kNear { 1.0, 0.0, 0.0 };
const Eigen::Vector3d kFar { 2.0, 0.0, 0.0 };
const Eigen::Vector3d kOther { 0.0, 1.0, 0.0 };

TEST(SphericalGridTest, AddAndExtractReturnsFarthestPerBin) {
    radar_lidar::spherical_grid::SphericalGrid grid(kGridDeg);
    radar_lidar::types::PointCloud pts = { kNear, kFar, kOther };

    grid.add(pts);

    auto result = grid.extract();
    ASSERT_EQ(result.size(), 2u);

    // (2,0,0) retained (farthest in its bin); (1,0,0) evicted
    bool found_far =
        std::any_of(result.begin(), result.end(), [](const auto& p) { return p.isApprox(kFar); });
    bool found_near =
        std::any_of(result.begin(), result.end(), [](const auto& p) { return p.isApprox(kNear); });
    EXPECT_TRUE(found_far);
    EXPECT_FALSE(found_near);
}

TEST(SphericalGridTest, ExtractClearsState) {
    radar_lidar::spherical_grid::SphericalGrid grid(0.5);
    grid.add({ Eigen::Vector3d(1, 0, 0) });
    ASSERT_GT(grid.size(), 0u);

    std::ignore = grid.extract();
    EXPECT_EQ(grid.size(), 0u);
    EXPECT_TRUE(grid.extract().empty());
}

TEST(SphericalGridTest, ClearResetsSize) {
    radar_lidar::spherical_grid::SphericalGrid grid(0.5);
    grid.add({ Eigen::Vector3d(1, 0, 0) });
    EXPECT_GT(grid.size(), 0u);
    grid.clear();
    EXPECT_EQ(grid.size(), 0u);
}

TEST(SphericalGridTest, EmptyExtractReturnsEmpty) {
    radar_lidar::spherical_grid::SphericalGrid grid(kGridDeg);
    EXPECT_TRUE(grid.extract().empty());
}

} // namespace
