#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <filesystem>
#include <gtest/gtest.h>
#include <numbers>
#include <ranges>

#include "radar_lidar/localization.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/types.hpp"

namespace {

constexpr auto deg_to_rad(double deg) -> double { return deg * std::numbers::pi / 180.0; }

auto make_cube_surface(double size, double step) -> pcl::PointCloud<pcl::PointXYZ>::Ptr {
    auto cloud        = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const double half = size / 2.0;
    for (double x = -half; x <= half; x += step) {
        for (double y = -half; y <= half; y += step) {
            cloud->emplace_back(x, y, -half);
            cloud->emplace_back(x, y, half);
            cloud->emplace_back(x, -half, y);
            cloud->emplace_back(x, half, y);
            cloud->emplace_back(-half, x, y);
            cloud->emplace_back(half, x, y);
        }
    }
    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;
    return cloud;
}

auto make_frame_from_cloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) -> radar::types::Frame {
    auto points = cloud.points
        | std::views::transform([](const auto& pt) { return Eigen::Vector3d(pt.x, pt.y, pt.z); })
        | std::ranges::to<std::vector<Eigen::Vector3d>>();
    return { .points = std::move(points) };
}

class LocalizationTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto cube = make_cube_surface(2.0, 0.1);
        pcl::io::savePCDFileBinary(map_pcd_, *cube);
    }
    void TearDown() override { std::filesystem::remove(map_pcd_); }

    static constexpr const char* map_pcd_ = "/tmp/radar_test_loc_map.pcd";
};

} // namespace

TEST_F(LocalizationTest, IdentityTransform) {
    auto map_result = radar::MapData::Load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();
    auto map = *map_result;

    radar::config::LocalizationConfig cfg;
    cfg.num_threads        = 2;
    cfg.max_iterations     = 50;
    cfg.max_corr_distance  = 1.0;
    cfg.use_spherical_grid = false;
    cfg.accumulate_frames  = 0;

    auto localization = radar::LocalizationStage(map, cfg);
    auto frame        = make_frame_from_cloud(map->pcl_cloud());

    auto result = localization.process(frame);
    ASSERT_TRUE(result.has_value()) << result.error();

    const auto& T = result->T;
    auto trans    = T.translation();
    EXPECT_NEAR(trans.x(), 0.0, 0.05);
    EXPECT_NEAR(trans.y(), 0.0, 0.05);
    EXPECT_NEAR(trans.z(), 0.0, 0.05);
    EXPECT_TRUE(result->converged);
}

TEST_F(LocalizationTest, KnownTranslation) {
    auto map_result = radar::MapData::Load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();
    auto map = *map_result;

    radar::config::LocalizationConfig cfg;
    cfg.num_threads        = 2;
    cfg.max_iterations     = 100;
    cfg.max_corr_distance  = 2.0;
    cfg.use_spherical_grid = false;
    cfg.accumulate_frames  = 0;

    auto localization = radar::LocalizationStage(map, cfg);

    Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();
    init_pose.translation()     = Eigen::Vector3d(-0.5, -0.3, 0.0);
    localization.set_initial_pose(init_pose);

    const Eigen::Vector3d shift(0.5, 0.3, 0.0);
    auto points = map->pcl_cloud().points | std::views::transform([&shift](const auto& pt) {
        return Eigen::Vector3d(pt.x + shift.x(), pt.y + shift.y(), pt.z + shift.z());
    }) | std::ranges::to<std::vector<Eigen::Vector3d>>();
    auto frame  = radar::types::Frame { .points = std::move(points) };

    auto result = localization.process(frame);
    ASSERT_TRUE(result.has_value()) << result.error();

    auto trans = result->T.translation();
    EXPECT_NEAR(trans.x(), -0.5, 0.1) << "Expected T.x ~ -0.5, got " << trans.x();
    EXPECT_NEAR(trans.y(), -0.3, 0.1) << "Expected T.y ~ -0.3, got " << trans.y();
    EXPECT_NEAR(trans.z(), 0.0, 0.1) << "Expected T.z ~  0.0, got " << trans.z();
    EXPECT_TRUE(result->converged);
}

TEST_F(LocalizationTest, KnownRotation) {
    auto map_result = radar::MapData::Load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();
    auto map = *map_result;

    radar::config::LocalizationConfig cfg;
    cfg.num_threads        = 2;
    cfg.max_iterations     = 100;
    cfg.max_corr_distance  = 2.0;
    cfg.use_spherical_grid = false;
    cfg.accumulate_frames  = 0;

    auto localization = radar::LocalizationStage(map, cfg);

    Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();
    init_pose.linear() =
        Eigen::AngleAxisd(deg_to_rad(-15.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    localization.set_initial_pose(init_pose);

    const Eigen::Matrix3d R =
        Eigen::AngleAxisd(deg_to_rad(15.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    auto points = map->pcl_cloud().points | std::views::transform([&R](const auto& pt) {
        return R * Eigen::Vector3d(pt.x, pt.y, pt.z);
    }) | std::ranges::to<std::vector<Eigen::Vector3d>>();
    auto frame  = radar::types::Frame { .points = std::move(points) };

    auto result = localization.process(frame);
    ASSERT_TRUE(result.has_value()) << result.error();

    const auto& R_result = result->T.rotation();
    const Eigen::AngleAxisd aa(R_result);
    EXPECT_NEAR(std::abs(aa.axis().z()), 1.0, 0.2) << "Rotation axis should be |Z|";
    EXPECT_NEAR(std::abs(aa.angle()), deg_to_rad(15.0), 0.1) << "Expected ~15deg rotation";
    EXPECT_TRUE(result->converged);
}

TEST_F(LocalizationTest, EmptyScanReturnsError) {
    auto map_result = radar::MapData::Load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value());
    auto map = *map_result;

    radar::config::LocalizationConfig cfg;
    auto localization = radar::LocalizationStage(map, cfg);

    radar::types::Frame empty_frame;
    auto result = localization.process(empty_frame);
    EXPECT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("Empty scan"), std::string::npos);
}

TEST_F(LocalizationTest, SetInitialPoseAndReset) {
    auto map_result = radar::MapData::Load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value());
    auto map = *map_result;

    radar::config::LocalizationConfig cfg;
    auto localization = radar::LocalizationStage(map, cfg);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation()     = Eigen::Vector3d(1.0, 2.0, 3.0);
    localization.set_initial_pose(pose);

    EXPECT_NO_THROW(localization.reset());
}
