// test_rgb_colorizer.cpp — Integration/orchestration test for color_world_points()
//
// Task 5 of the FAST-LIVO2 RGB map/replay plan.
// RED phase: color_world_points() does not exist yet.
// This test verifies the full projection→colorization→voxel pipeline
// with synthetic calibration, image, cloud, and pose.

#include <gtest/gtest.h>

#include <cstdint>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// The function under test — will NOT compile before implementation (RED).
#include "radar_fast_livo2_rgb/rgb_colorizer.hpp"
#include "radar_fast_livo2_rgb/color_voxel_map.hpp"
#include "radar_fast_livo2_rgb/pcd_trigger.hpp"
#include "radar_fast_livo2_rgb/rgb_projection.hpp"

using namespace radar::fast_livo2::rgb;

// ── Test helpers ──────────────────────────────────────────────────────

/// 4-arg overload: explicitly set intrinsics with arbitrary cx/cy.
/// The 0-arg version in rgb_projection.hpp hardcodes cx=50,cy=40.
inline auto make_identity_calibration(double fx, double fy,
                                       double cx, double cy) -> Calibration
{
    return Calibration {
        .fx = fx, .fy = fy, .cx = cx, .cy = cy,
        .rotation_lidar_camera = Eigen::Matrix3d::Identity(),
        .translation_lidar_camera = Eigen::Vector3d::Zero()
    };
}

/// Convenience: vector of world-frame points from an initializer list.
inline auto make_world_cloud(
    std::initializer_list<Eigen::Vector3d> points)
    -> std::vector<Eigen::Vector3d>
{
    return std::vector<Eigen::Vector3d>(points);
}

/// Identity odometry pose: T_world_lidar = I → lidar origin at world origin.
inline auto identity_odom() -> Eigen::Isometry3d
{
    return Eigen::Isometry3d::Identity();
}

// ════════════════════════════════════════════════════════════════════════
// Core integration test (from task-5-brief Step 1)
// ════════════════════════════════════════════════════════════════════════

TEST(RgbColorizer, PublishesProjectedRgbForVisibleWorldPoint)
{
    // Calibration with cx=1,cy=1 so (0,0,2) projects to pixel (1,1)
    // in the 3×3 test image.
    const auto calibration =
        make_identity_calibration(100.0, 100.0, 1.0, 1.0);

    // 3×3 BGR8 image — all black except centre pixel is (B=30,G=20,R=10).
    cv::Mat bgr(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    bgr.at<cv::Vec3b>(1, 1) = cv::Vec3b(30, 20, 10);

    // Single world point at (0, 0, 2) — with identity odometry this
    // is also the lidar-frame point.
    const auto map = color_world_points(
        make_world_cloud({Eigen::Vector3d{0.0, 0.0, 2.0}}),
        bgr,
        identity_odom(),
        calibration,
        QualityWeights{});

    const auto cloud = map.to_point_cloud();
    ASSERT_EQ(cloud.size(), 1U);
    // Packed 0xRRGGBB: R=10=0x0A, G=20=0x14, B=30=0x1E → 0x0A141E.
    EXPECT_EQ(unpack_rgb(cloud.front()), 0x0A141EU);
}

// ════════════════════════════════════════════════════════════════════════
// Occlusion: point behind another should not be colorized
// ════════════════════════════════════════════════════════════════════════

TEST(RgbColorizer, OccludedPointIsNotColorized)
{
    const auto calibration =
        make_identity_calibration(100.0, 100.0, 1.0, 1.0);

    // 3×3 BGR8 with distinct colours at (1,0) and (1,1).
    cv::Mat bgr(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    bgr.at<cv::Vec3b>(0, 1) = cv::Vec3b(255, 0, 0);    // blue
    bgr.at<cv::Vec3b>(1, 1) = cv::Vec3b(0, 255, 0);    // green

    // Two world points that project to the same pixel (1,1).
    // Point A: depth=2.0 (nearer) → should get green.
    // Point B: depth=3.0 (farther) → should be occluded.
    const auto map = color_world_points(
        make_world_cloud({
            Eigen::Vector3d{0.0, 0.0, 2.0},   // depth = 2.0
            Eigen::Vector3d{0.0, 0.0, 3.0}    // depth = 3.0, same u,v
        }),
        bgr,
        identity_odom(),
        calibration,
        QualityWeights{});

    const auto cloud = map.to_point_cloud();
    ASSERT_EQ(cloud.size(), 1U);
    // Only the nearer point survives. Green = 0x0000FF00.
    EXPECT_EQ(unpack_rgb(cloud.front()), 0x0000FF00U);
}

// ════════════════════════════════════════════════════════════════════════
// Multiple visible points in different pixels
// ════════════════════════════════════════════════════════════════════════

TEST(RgbColorizer, MultipleVisiblePointsEachGetTheirColor)
{
    // Use a larger image and calibration so two distinct world points
    // project to two distinct pixels.
    cv::Mat bgr(20, 20, CV_8UC3, cv::Scalar(0, 0, 0));
    bgr.at<cv::Vec3b>(5, 5) = cv::Vec3b(10, 20, 30);   // packed: 0x1E140A
    bgr.at<cv::Vec3b>(5, 15) = cv::Vec3b(40, 50, 60);  // packed: 0x3C3228

    // Calibration: fx=fy=100, cx=cy=10.
    // Point 1: lidar-frame (0, 0, 2) → u=10, v=10 → pixel (10,10).
    // Hmm, that doesn't land on (5,5). Let me work this out.
    //
    // Want: project (x, y, z) to (col=5, row=5) in a 20×20 image.
    // u = fx*x/z + cx,  v = fy*y/z + cy
    // 5 = 100*x/2 + 10 → x = -0.1
    // 5 = 100*y/2 + 10 → y = -0.1
    // So point (-0.1, -0.1, 2.0) projects to (5, 5).

    const auto calibration =
        make_identity_calibration(100.0, 100.0, 10.0, 10.0);

    const auto map = color_world_points(
        make_world_cloud({
            Eigen::Vector3d{-0.1, -0.1, 2.0},    // → pixel (5, 5)
            Eigen::Vector3d{ 0.1, -0.1, 2.0}     // → pixel (15, 5), since u=100*0.1/2+10=15
        }),
        bgr,
        identity_odom(),
        calibration,
        QualityWeights{});

    const auto cloud = map.to_point_cloud();
    ASSERT_EQ(cloud.size(), 2U);

    std::set<uint32_t> colours;
    for (const auto& pt : cloud) {
        colours.insert(unpack_rgb(pt));
    }
    EXPECT_EQ(colours.size(), 2U);
    EXPECT_TRUE(colours.count(0x1E140AU));  // BGR(10,20,30) → RGB(30,20,10)
    EXPECT_TRUE(colours.count(0x3C3228U));  // BGR(40,50,60) → RGB(60,50,40)
}

// ════════════════════════════════════════════════════════════════════════
// Non-identity odometry transform
// ════════════════════════════════════════════════════════════════════════

TEST(RgbColorizer, RespectsOdometryTransform)
{
    // Camera at world origin looking down +Z. Same calibration as baseline.
    const auto calibration =
        make_identity_calibration(100.0, 100.0, 1.0, 1.0);

    cv::Mat bgr(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    bgr.at<cv::Vec3b>(1, 1) = cv::Vec3b(30, 20, 10);

    // Odometry: lidar is at (0, 0, -1) in world frame, looking down +Z.
    // T_world_lidar: translates world origin by (0,0,-1).
    // A world point at (0, 0, 1) is (0, 0, 2) in lidar frame.
    // Projects to pixel (1,1) with cx=1,cy=1.
    Eigen::Isometry3d odom = Eigen::Isometry3d::Identity();
    odom.translation() = Eigen::Vector3d(0.0, 0.0, -1.0);

    const auto map = color_world_points(
        make_world_cloud({Eigen::Vector3d{0.0, 0.0, 1.0}}),
        bgr,
        odom,
        calibration,
        QualityWeights{});

    const auto cloud = map.to_point_cloud();
    ASSERT_EQ(cloud.size(), 1U);
    EXPECT_EQ(unpack_rgb(cloud.front()), 0x0A141EU);
}

// ════════════════════════════════════════════════════════════════════════
// Empty world cloud → empty map
// ════════════════════════════════════════════════════════════════════════

TEST(RgbColorizer, EmptyCloudReturnsEmptyMap)
{
    const auto calibration =
        make_identity_calibration(100.0, 100.0, 50.0, 40.0);
    cv::Mat bgr(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));

    const auto map = color_world_points(
        {}, bgr, identity_odom(), calibration,
        QualityWeights{});

    EXPECT_EQ(map.size(), 0U);
}

// ════════════════════════════════════════════════════════════════════════
// Regression M1: quality scores survive merge into a global map.
// Two calls produce the same voxel; the second has higher quality;
// the global map must retain the higher-quality colour regardless of
// call order.
// ════════════════════════════════════════════════════════════════════════

TEST(RgbColorizer, HigherQualityReplacesLowerAcrossCalls)
{
    // Same calibration as baseline: (0,0,2) → pixel (1,1) in 3×3.
    const auto calibration =
        make_identity_calibration(100.0, 100.0, 1.0, 1.0);

    // Call 1: low-quality colour at pixel (1,1).
    cv::Mat bgr1(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    bgr1.at<cv::Vec3b>(1, 1) = cv::Vec3b(10, 10, 10);  // grey, ~packed 0x0A0A0A

    auto map1 = color_world_points(
        make_world_cloud({Eigen::Vector3d{0.0, 0.0, 2.0}}),
        bgr1, identity_odom(), calibration, QualityWeights{});

    // Call 2: different colour at same pixel, same point → higher quality
    // because we use a brighter image (larger gradient = higher quality).
    cv::Mat bgr2(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    bgr2.at<cv::Vec3b>(1, 1) = cv::Vec3b(30, 20, 10);  // same as baseline, 0x0A141E
    bgr2.at<cv::Vec3b>(0, 0) = cv::Vec3b(255, 255, 255);  // create high gradient

    auto map2 = color_world_points(
        make_world_cloud({Eigen::Vector3d{0.0, 0.0, 2.0}}),
        bgr2, identity_odom(), calibration, QualityWeights{});

    // Merge into a global map — order: low-quality first, then high.
    ColorVoxelMap global(0.10);
    map1.for_each_voxel([&](const Eigen::Vector3d& pos,
                             uint32_t rgb, double quality) {
        global.insert_if_better(pos, rgb, quality);
    });
    map2.for_each_voxel([&](const Eigen::Vector3d& pos,
                             uint32_t rgb, double quality) {
        global.insert_if_better(pos, rgb, quality);
    });

    const auto cloud = global.to_point_cloud();
    ASSERT_EQ(cloud.size(), 1U);
    // The second call's colour should win because its quality is higher
    // (brighter pixel → higher gradient → higher quality_score).
    EXPECT_EQ(unpack_rgb(cloud.front()), 0x0A141EU);
}

// ════════════════════════════════════════════════════════════════════════
// Regression M1 reverse-order: high-quality first, then low.
// The first (higher quality) colour must survive.
// ════════════════════════════════════════════════════════════════════════

TEST(RgbColorizer, LowerQualityDoesNotReplaceHigherAcrossCalls)
{
    const auto calibration =
        make_identity_calibration(100.0, 100.0, 1.0, 1.0);

    // Call 1: high quality (bright image context).
    cv::Mat bgr_high(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    bgr_high.at<cv::Vec3b>(1, 1) = cv::Vec3b(30, 20, 10);
    bgr_high.at<cv::Vec3b>(0, 0) = cv::Vec3b(255, 255, 255);

    auto map_high = color_world_points(
        make_world_cloud({Eigen::Vector3d{0.0, 0.0, 2.0}}),
        bgr_high, identity_odom(), calibration, QualityWeights{});

    // Call 2: low quality (flat image).
    cv::Mat bgr_low(3, 3, CV_8UC3, cv::Scalar(50, 50, 50));

    auto map_low = color_world_points(
        make_world_cloud({Eigen::Vector3d{0.0, 0.0, 2.0}}),
        bgr_low, identity_odom(), calibration, QualityWeights{});

    ColorVoxelMap global(0.10);
    map_high.for_each_voxel([&](const Eigen::Vector3d& pos,
                                 uint32_t rgb, double quality) {
        global.insert_if_better(pos, rgb, quality);
    });
    map_low.for_each_voxel([&](const Eigen::Vector3d& pos,
                                uint32_t rgb, double quality) {
        global.insert_if_better(pos, rgb, quality);
    });

    const auto cloud = global.to_point_cloud();
    ASSERT_EQ(cloud.size(), 1U);
    EXPECT_EQ(unpack_rgb(cloud.front()), 0x0A141EU);
}

// ════════════════════════════════════════════════════════════════════════
// M5: PCD trigger transition logic
// ════════════════════════════════════════════════════════════════════════

TEST(PcdTriggerTransition, NoTriggerReturnsFalse)
{
    EXPECT_FALSE(pcd_trigger_transition(false, 0));
    EXPECT_FALSE(pcd_trigger_transition(false, -1));
}

TEST(PcdTriggerTransition, SuccessResetsTrigger)
{
    EXPECT_FALSE(pcd_trigger_transition(true, 0));
}

TEST(PcdTriggerTransition, FailureRetainsTrigger)
{
    EXPECT_TRUE(pcd_trigger_transition(true, -1));
    EXPECT_TRUE(pcd_trigger_transition(true, 1));
    EXPECT_TRUE(pcd_trigger_transition(true, 42));
}

TEST(PcdTriggerTransition, IdempotentWhenAlreadyFalse)
{
    EXPECT_FALSE(pcd_trigger_transition(false, 0));
    EXPECT_FALSE(pcd_trigger_transition(false, 0));
}

// ════════════════════════════════════════════════════════════════════════
// Time-tolerance helper (used by find_nearest_odom)
// ════════════════════════════════════════════════════════════════════════

TEST(TimeTolerance, ZeroDeltaWithinAnyNominalTolerance)
{
    EXPECT_TRUE(within_tolerance_ns(0, 50'000'000));
    EXPECT_TRUE(within_tolerance_ns(0, 0));
}

TEST(TimeTolerance, DeltaWithinTolerancePasses)
{
    EXPECT_TRUE(within_tolerance_ns(49'000'000, 50'000'000));
    EXPECT_TRUE(within_tolerance_ns(50'000'000, 50'000'000));
}

TEST(TimeTolerance, DeltaExceedingToleranceFails)
{
    EXPECT_FALSE(within_tolerance_ns(50'000'001, 50'000'000));
    EXPECT_FALSE(within_tolerance_ns(100'000'000, 50'000'000));
}

TEST(TimeTolerance, NegativeDeltaFails)
{
    EXPECT_FALSE(within_tolerance_ns(-1, 50'000'000));
}

// ══════════════════════════════════════════════════════════════════
// Regression: colour sampling with RGB vs BGR source order
// ══════════════════════════════════════════════════════════════════

TEST(RgbColorizer, PacksCorrectRGBFromBGRSource)
{
    // Replay mode (BGR source): cv::Vec3b(30, 20, 10) = B=30,G=20,R=10
    // pack_rgb() treats ch2 as R=10, ch1 as G=20, ch0 as B=30 → 0x0A141E
    const auto calibration =
        make_identity_calibration(100.0, 100.0, 1.0, 1.0);

    cv::Mat bgr(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    bgr.at<cv::Vec3b>(1, 1) = cv::Vec3b(30, 20, 10);  // B=30, G=20, R=10

    const auto map = color_world_points(
        make_world_cloud({Eigen::Vector3d{0.0, 0.0, 2.0}}),
        bgr, identity_odom(), calibration,
        QualityWeights{}, 0.10, ColorFormat::BGR);

    const auto cloud = map.to_point_cloud();
    ASSERT_EQ(cloud.size(), 1U);
    EXPECT_EQ(unpack_rgb(cloud.front()), 0x0A141EU);
}

TEST(RgbColorizer, PacksCorrectRGBFromRGBSource)
{
    // Live SHM mode (RGB source): pixel is R=10, G=20, B=30.
    // SHM stores bytes [R=10, G=20, B=30]; after memcpy to cv::Mat
    // the channels are [10, 20, 30] which OpenCV interprets as
    // B=10, G=20, R=30 — i.e. Vec3b(10, 20, 30).
    // pack_rgb_from_rgb_order() treats ch0 as R=10, ch1 as G=20,
    // ch2 as B=30 → packed 0x0A141E.
    const auto calibration =
        make_identity_calibration(100.0, 100.0, 1.0, 1.0);

    cv::Mat rgb(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    // SHM RGB data: R=10,G=20,B=30 → cv::Mat Vec3b(10, 20, 30)
    rgb.at<cv::Vec3b>(1, 1) = cv::Vec3b(10, 20, 30);

    const auto map = color_world_points(
        make_world_cloud({Eigen::Vector3d{0.0, 0.0, 2.0}}),
        rgb, identity_odom(), calibration,
        QualityWeights{}, 0.10, ColorFormat::RGB);

    const auto cloud = map.to_point_cloud();
    ASSERT_EQ(cloud.size(), 1U);
    // pack_rgb_from_rgb_order: (R<<16)|(G<<8)|B = (10<<16)|(20<<8)|30 = 0x0A141E
    EXPECT_EQ(unpack_rgb(cloud.front()), 0x0A141EU);
}

TEST(RgbColorizer, RGBvsBGRSourceProducesSamePackedColor)
{
    // Same logical colour R=10,G=20,B=30, provided in both formats,
    // should produce identical packed RGB regardless of source order.
    const auto calibration =
        make_identity_calibration(100.0, 100.0, 1.0, 1.0);

    // BGR source: cv::Vec3b(30, 20, 10) = B=30,G=20,R=10 → 0x0A141E
    cv::Mat bgr(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    bgr.at<cv::Vec3b>(1, 1) = cv::Vec3b(30, 20, 10);

    const auto map_bgr = color_world_points(
        make_world_cloud({Eigen::Vector3d{0.0, 0.0, 2.0}}),
        bgr, identity_odom(), calibration,
        QualityWeights{}, 0.10, ColorFormat::BGR);

    // RGB source: SHM R=10,G=20,B=30 → cv::Mat Vec3b(10, 20, 30) → 0x0A141E
    cv::Mat rgb(3, 3, CV_8UC3, cv::Scalar(0, 0, 0));
    rgb.at<cv::Vec3b>(1, 1) = cv::Vec3b(10, 20, 30);

    const auto map_rgb = color_world_points(
        make_world_cloud({Eigen::Vector3d{0.0, 0.0, 2.0}}),
        rgb, identity_odom(), calibration,
        QualityWeights{}, 0.10, ColorFormat::RGB);

    const auto cloud_bgr = map_bgr.to_point_cloud();
    const auto cloud_rgb = map_rgb.to_point_cloud();
    ASSERT_EQ(cloud_bgr.size(), 1U);
    ASSERT_EQ(cloud_rgb.size(), 1U);
    EXPECT_EQ(unpack_rgb(cloud_bgr.front()), unpack_rgb(cloud_rgb.front()));
    EXPECT_EQ(unpack_rgb(cloud_bgr.front()), 0x0A141EU);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
