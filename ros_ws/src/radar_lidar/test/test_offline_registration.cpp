#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <gtest/gtest.h>
#include <numbers>

#include "radar_lidar/offline_registration.hpp"

using radar_lidar::registration::deg_to_rad;
using radar_lidar::registration::is_better_score;
using radar_lidar::registration::rad_to_deg;
using radar_lidar::registration::RegistrationScore;
using radar_lidar::registration::score_alignment;

namespace {

auto make_flat_plane(size_t n, double z = 0.0) -> pcl::PointCloud<pcl::PointXYZ>::Ptr {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    cloud->reserve(n * n);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            cloud->emplace_back(
                static_cast<float>(i), static_cast<float>(j), static_cast<float>(z));
        }
    }
    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;
    return cloud;
}

} // namespace

// ── deg_to_rad / rad_to_deg ──────────────────────────────────────────────

TEST(RegistrationUtilsTest, DegRadRoundtrip) {
    EXPECT_NEAR(deg_to_rad(180.0), std::numbers::pi, 1e-12);
    EXPECT_NEAR(rad_to_deg(std::numbers::pi), 180.0, 1e-12);
    EXPECT_NEAR(deg_to_rad(0.0), 0.0, 1e-12);
    EXPECT_NEAR(rad_to_deg(0.0), 0.0, 1e-12);
}

// ── is_better_score ────────────────────────────────────────────────────────

TEST(RegistrationScoreTest, InlierRatioDominant) {
    RegistrationScore a { .inlier_ratio = 0.9, .rmse = 10.0 };
    RegistrationScore b { .inlier_ratio = 0.5, .rmse = 1.0 };
    EXPECT_TRUE(is_better_score(a, b));
    EXPECT_FALSE(is_better_score(b, a));
}

TEST(RegistrationScoreTest, RmseBreaksTie) {
    RegistrationScore a { .inlier_ratio = 0.9, .rmse = 1.0 };
    RegistrationScore b { .inlier_ratio = 0.9, .rmse = 2.0 };
    EXPECT_TRUE(is_better_score(a, b));
    EXPECT_FALSE(is_better_score(b, a));
}

TEST(RegistrationScoreTest, DefaultIsWorst) {
    RegistrationScore a { .inlier_ratio = 0.0, .rmse = std::numeric_limits<double>::max() };
    RegistrationScore b { .inlier_ratio = 0.0, .rmse = 0.1 };
    // Same inlier ratio, lower rmse is better
    EXPECT_TRUE(is_better_score(b, a));
}

// ── score_alignment ─────────────────────────────────────────────────────────

class ScoreAlignmentTest : public ::testing::Test {
protected:
    void SetUp() override {
        map_cloud_ = make_flat_plane(10, 0.0);
        map_tree_.setInputCloud(map_cloud_);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ> map_tree_;
};

TEST_F(ScoreAlignmentTest, PerfectMatchGivesFullInlierRatio) {
    radar_lidar::types::PointCloud source;
    for (const auto& pt : map_cloud_->points) {
        source.emplace_back(pt.x, pt.y, pt.z);
    }

    const auto score = score_alignment(map_tree_, source, Eigen::Isometry3d::Identity(), 1.0);

    EXPECT_NEAR(score.inlier_ratio, 1.0, 1e-6) << "All points should be inliers for identity "
                                                  "transform";
    EXPECT_LT(score.rmse, 1e-6) << "RMSE should be near zero for exact match";
}

TEST_F(ScoreAlignmentTest, OffsetReducesInlierRatio) {
    radar_lidar::types::PointCloud source;
    // Shift source by a large amount so points fall outside threshold
    for (const auto& pt : map_cloud_->points) {
        source.emplace_back(pt.x + 100.0, pt.y + 100.0, pt.z);
    }

    const auto score = score_alignment(map_tree_, source, Eigen::Isometry3d::Identity(), 1.0);

    EXPECT_NEAR(score.inlier_ratio, 0.0, 1e-6) << "No points should be inliers after large offset";
    EXPECT_NEAR(score.rmse, std::numeric_limits<double>::max(), 1e-12) << "RMSE should be max when "
                                                                          "no inliers";
}

TEST_F(ScoreAlignmentTest, EmptySourceReturnsDefault) {
    radar_lidar::types::PointCloud empty;
    const auto score = score_alignment(map_tree_, empty, Eigen::Isometry3d::Identity(), 1.0);

    EXPECT_NEAR(score.inlier_ratio, 0.0, 1e-6);
    EXPECT_NEAR(score.rmse, std::numeric_limits<double>::max(), 1e-12);
}

TEST_F(ScoreAlignmentTest, KnownTransformRecoversInliers) {
    // Create a source cloud that is the map points transformed by a known translation
    const Eigen::Vector3d offset(1.0, 2.0, 0.0);
    radar_lidar::types::PointCloud source;
    for (const auto& pt : map_cloud_->points) {
        source.emplace_back(pt.x + offset.x(), pt.y + offset.y(), pt.z + offset.z());
    }

    // Score with the inverse transform should give high inlier ratio
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation()     = -offset;
    const auto score    = score_alignment(map_tree_, source, T, 0.5);

    // After inverse transform, all points should be close to original map points
    EXPECT_GT(score.inlier_ratio, 0.95) << "Most points should be inliers";
    EXPECT_LT(score.rmse, 0.5) << "RMSE should be within threshold";
}
