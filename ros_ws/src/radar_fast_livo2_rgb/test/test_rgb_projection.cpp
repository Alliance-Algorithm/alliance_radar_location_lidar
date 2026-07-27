// test_rgb_projection.cpp — Unit tests for RGB projection primitives
//
// Tests follow the brief: projection, rejection behind camera, nearest-depth
// update, and BGR→packed-RGB conversion. Additional tests cover calibration
// validation, depth buffer construction, visibility, and quality scoring.

#include <gtest/gtest.h>
#include <limits>
#include <opencv2/core.hpp>

#include "radar_fast_livo2_rgb/rgb_projection.hpp"

using namespace radar::fast_livo2::rgb;

// ════════════════════════════════════════════════════════════════════════
// Perspective projection
// ════════════════════════════════════════════════════════════════════════

TEST(RgbProjection, ProjectsForwardPointIntoCalibratedPixel)
{
    const Calibration calibration {
        .fx = 100.0, .fy = 100.0, .cx = 50.0, .cy = 40.0,
        .rotation_lidar_camera = Eigen::Matrix3d::Identity(),
        .translation_lidar_camera = Eigen::Vector3d::Zero()
    };

    const auto projection = project_lidar_point(
        Eigen::Vector3d { 1.0, 0.0, 2.0 }, calibration);

    ASSERT_TRUE(projection.has_value());
    EXPECT_DOUBLE_EQ(projection->u, 100.0);
    EXPECT_DOUBLE_EQ(projection->v, 40.0);
    EXPECT_DOUBLE_EQ(projection->depth, 2.0);
}

TEST(RgbProjection, RejectsPointBehindCamera)
{
    const Calibration calibration = make_identity_calibration();
    EXPECT_FALSE(
        project_lidar_point(Eigen::Vector3d { 0.0, 0.0, -1.0 }, calibration)
            .has_value());
}

// ════════════════════════════════════════════════════════════════════════
// Depth buffer
// ════════════════════════════════════════════════════════════════════════

TEST(RgbProjection, KeepsNearestDepthAtSamePixel)
{
    cv::Mat depth(8, 8, CV_32FC1,
                  cv::Scalar(std::numeric_limits<float>::infinity()));
    update_nearest_depth(depth, 3, 4, 4.0F);
    update_nearest_depth(depth, 3, 4, 2.0F);
    EXPECT_FLOAT_EQ(depth.at<float>(4, 3), 2.0F);
}

// ════════════════════════════════════════════════════════════════════════
// Color packing
// ════════════════════════════════════════════════════════════════════════

TEST(RgbProjection, PacksBgrAsStandardRgb)
{
    EXPECT_EQ(pack_rgb(cv::Vec3b { 3, 2, 1 }), 0x010203U);
}

// ════════════════════════════════════════════════════════════════════════
// Additional: calibration validation
// ════════════════════════════════════════════════════════════════════════

TEST(RgbProjection, DetectsInvalidZeroFocalLength)
{
    Calibration cal = make_identity_calibration();
    cal.fx = 0.0;
    EXPECT_FALSE(project_lidar_point(Eigen::Vector3d { 1.0, 0.0, 2.0 }, cal)
                     .has_value());
}

TEST(RgbProjection, DetectsInvalidNegativeFocalLength)
{
    Calibration cal = make_identity_calibration();
    cal.fy = -1.0;
    EXPECT_FALSE(project_lidar_point(Eigen::Vector3d { 1.0, 0.0, 2.0 }, cal)
                     .has_value());
}

TEST(RgbProjection, DetectsNanInExtrinsics)
{
    Calibration cal = make_identity_calibration();
    cal.rotation_lidar_camera(0, 0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(project_lidar_point(Eigen::Vector3d { 1.0, 0.0, 2.0 }, cal)
                     .has_value());
}

// ════════════════════════════════════════════════════════════════════════
// Additional: depth buffer construction and visibility
// ════════════════════════════════════════════════════════════════════════

TEST(RgbProjection, BuildDepthBufferFilledWithInf)
{
    auto buf = build_depth_buffer(10, 8);
    EXPECT_EQ(buf.rows, 8);
    EXPECT_EQ(buf.cols, 10);
    EXPECT_EQ(buf.type(), CV_32FC1);
    EXPECT_TRUE(std::isinf(buf.at<float>(0, 0)));
    EXPECT_TRUE(std::isinf(buf.at<float>(7, 9)));
}

TEST(RgbProjection, IsVisibleWhenDepthIsClosest)
{
    cv::Mat depth(4, 4, CV_32FC1,
                  cv::Scalar(std::numeric_limits<float>::infinity()));
    EXPECT_TRUE(is_visible(depth, 2, 1, 3.0F));
    update_nearest_depth(depth, 2, 1, 3.0F);
    // Same depth is visible (within tolerance)
    EXPECT_TRUE(is_visible(depth, 2, 1, 3.0F));
    // Farther point is not visible
    EXPECT_FALSE(is_visible(depth, 2, 1, 5.0F));
    // Closer point is visible
    EXPECT_TRUE(is_visible(depth, 2, 1, 2.0F));
}

TEST(RgbProjection, IsVisibleOutOfBoundsReturnsFalse)
{
    cv::Mat depth(4, 4, CV_32FC1,
                  cv::Scalar(std::numeric_limits<float>::infinity()));
    EXPECT_FALSE(is_visible(depth, -1, 0, 1.0F));
    EXPECT_FALSE(is_visible(depth, 0, -1, 1.0F));
    EXPECT_FALSE(is_visible(depth, 4, 0, 1.0F));
    EXPECT_FALSE(is_visible(depth, 0, 4, 1.0F));
}

// ════════════════════════════════════════════════════════════════════════
// Additional: raw-byte pack_rgb overload
// ════════════════════════════════════════════════════════════════════════

TEST(RgbProjection, PacksRawBgrBytes)
{
    EXPECT_EQ(pack_rgb(0xAA, 0xBB, 0xCC), 0xCCBBAAU);
}

// ════════════════════════════════════════════════════════════════════════
// Additional: quality score
// ════════════════════════════════════════════════════════════════════════

TEST(RgbProjection, QualityScoreIsFiniteForForwardPoint)
{
    Calibration cal = make_identity_calibration();
    auto proj = project_lidar_point(Eigen::Vector3d { 1.0, 0.0, 2.0 }, cal);
    ASSERT_TRUE(proj.has_value());
    // With identity weights and a forward point, the score should be positive and finite
    double score = quality_score(Eigen::Vector3d { 1.0, 0.0, 2.0 }, *proj,
                                 cal, 100, 80);
    EXPECT_TRUE(std::isfinite(score));
    EXPECT_GT(score, 0.0);
}

TEST(RgbProjection, QualityScoreReturnsZeroForOrigin)
{
    Calibration cal = make_identity_calibration();
    Eigen::Vector3d origin { 0.0, 0.0, 0.0 };
    // Point at origin has zero distance; score should be zero
    auto proj = project_lidar_point(Eigen::Vector3d { 0.0, 0.0, 1.0 }, cal);
    ASSERT_TRUE(proj.has_value());
    double score = quality_score(origin, *proj, cal, 100, 80);
    EXPECT_EQ(score, 0.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
