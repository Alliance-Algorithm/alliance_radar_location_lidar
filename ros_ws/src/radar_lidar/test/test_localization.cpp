#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <gtest/gtest.h>

#include "radar_lidar/localization.hpp"
#include "radar_lidar/map_data.hpp"
#include "radar_lidar/types.hpp"

namespace {

// 在立方体表面均匀采样生成点云
pcl::PointCloud<pcl::PointXYZ>::Ptr make_cube_surface(float size, float step) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    float half = size / 2.0f;
    for (float x = -half; x <= half; x += step) {
        for (float y = -half; y <= half; y += step) {
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

std::string save_temp_pcd(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& path) {
    pcl::io::savePCDFileBinary(path, *cloud);
    return path;
}

class LocalizationTest : public ::testing::Test {
protected:
    void SetUp() override {
        map_pcd_ = "/tmp/radar_test_loc_map.pcd";
        // 2m 立方体，0.1m 步长 → ~2400 点
        auto cube = make_cube_surface(2.0f, 0.1f);
        save_temp_pcd(cube, map_pcd_);
    }
    void TearDown() override { std::filesystem::remove(map_pcd_); }
    std::string map_pcd_;
};

} // namespace

TEST_F(LocalizationTest, IdentityTransform) {
    // 扫描 = 地图（无变换），配准应得到接近 Identity 的结果
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

    // 用地图自己的点做"扫描"
    radar::types::Frame frame;
    for (const auto& pt : map->pcl_cloud().points) {
        frame.points.emplace_back(pt.x, pt.y, pt.z);
    }

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
    // 对地图施加已知平移 (0.5, 0.3, 0) 得到扫描
    // GICP 的 T_target_source 应 ≈ 平移的逆 (-0.5, -0.3, 0)
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

    // 施加平移得到扫描点
    const Eigen::Vector3d shift(0.5, 0.3, 0.0);
    radar::types::Frame frame;
    for (const auto& pt : map->pcl_cloud().points) {
        frame.points.emplace_back(pt.x + shift.x(), pt.y + shift.y(), pt.z + shift.z());
    }

    auto result = localization.process(frame);
    ASSERT_TRUE(result.has_value()) << result.error();

    auto trans = result->T.translation();
    EXPECT_NEAR(trans.x(), -0.5, 0.1) << "Expected T.x ≈ -0.5, got " << trans.x();
    EXPECT_NEAR(trans.y(), -0.3, 0.1) << "Expected T.y ≈ -0.3, got " << trans.y();
    EXPECT_NEAR(trans.z(), 0.0, 0.1) << "Expected T.z ≈ 0.0, got " << trans.z();
    EXPECT_TRUE(result->converged);
}

TEST_F(LocalizationTest, KnownRotation) {
    // 对地图施加绕 Z 轴 15 度旋转得到扫描
    // GICP 应恢复接近 -15 度的旋转
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

    // 初始位姿给 -15 度帮助收敛
    Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();
    double angle                = -15.0 * M_PI / 180.0;
    init_pose.linear() = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    localization.set_initial_pose(init_pose);

    // 施加 +15 度旋转得到扫描
    double rot_rad    = 15.0 * M_PI / 180.0;
    Eigen::Matrix3d R = Eigen::AngleAxisd(rot_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    radar::types::Frame frame;
    for (const auto& pt : map->pcl_cloud().points) {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d rotated = R * p;
        frame.points.push_back(rotated);
    }

    auto result = localization.process(frame);
    ASSERT_TRUE(result.has_value()) << result.error();

    // 验证旋转：恢复的旋转应 ≈ -15 度（绕 Z）
    const auto& R_result = result->T.rotation();
    Eigen::AngleAxisd aa(R_result);
    // 旋转轴应接近 Z 轴（取绝对值，四元数符号不影响旋转）
    EXPECT_NEAR(std::abs(aa.axis().z()), 1.0, 0.2) << "Rotation axis should be |Z|";
    EXPECT_NEAR(std::abs(aa.angle()), 15.0 * M_PI / 180.0, 0.1) << "Expected ~15deg rotation "
                                                                   "magnitude";
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

    // set_initial_pose 不应崩溃
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation()     = Eigen::Vector3d(1.0, 2.0, 3.0);
    localization.set_initial_pose(pose);

    // reset 不应崩溃
    localization.reset();
    EXPECT_NO_THROW(localization.reset());
}
