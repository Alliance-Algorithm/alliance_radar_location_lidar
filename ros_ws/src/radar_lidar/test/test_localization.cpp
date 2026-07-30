#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <gtest/gtest.h>
#include <limits>
#include <numbers>
#include <ranges>

#include "radar_lidar/data_format.hpp"
#include "radar_lidar/localization_stage.hpp"
#include "radar_lidar/map_data.hpp"

namespace radar_lidar::localization {

class LocalizationStageTestPeer {
public:
    static void apply_result(LocalizationStage& stage, const types::PoseEstimate& result) {
        stage.apply_registration_result(result);
    }

    static auto previous_pose(const LocalizationStage& stage) -> const Eigen::Isometry3d& {
        return stage.prev_pose_;
    }
};

} // namespace radar_lidar::localization

namespace {

constexpr auto deg_to_rad(double deg) -> double { return deg * std::numbers::pi / 180.0; }

auto make_cube_surface(double size, double step) -> pcl::PointCloud<pcl::PointXYZ>::Ptr {
    auto cloud            = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const double half     = size / 2.0;
    const double offset_x = 15.0;
    const double offset_y = 0.0;
    const double offset_z = 1.0;
    for (double x = -half; x <= half; x += step) {
        for (double y = -half; y <= half; y += step) {
            cloud->emplace_back(x + offset_x, y + offset_y, -half + offset_z);
            cloud->emplace_back(x + offset_x, y + offset_y, half + offset_z);
            cloud->emplace_back(x + offset_x, -half + offset_y, y + offset_z);
            cloud->emplace_back(x + offset_x, half + offset_y, y + offset_z);
            cloud->emplace_back(-half + offset_x, x + offset_y, y + offset_z);
            cloud->emplace_back(half + offset_x, x + offset_y, y + offset_z);
        }
    }
    cloud->width    = cloud->size();
    cloud->height   = 1;
    cloud->is_dense = true;
    return cloud;
}

auto make_frame_from_cloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
    -> radar_lidar::types::Frame {
    std::vector<Eigen::Vector3d> points;
    points.reserve(cloud.points.size());
    for (const auto& pt : cloud.points) {
        points.emplace_back(pt.x, pt.y, pt.z);
    }
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
    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();
    auto map = *map_result;

    radar_lidar::config::LocalizationConfig cfg;
    cfg.num_threads        = 2;
    cfg.max_iterations     = 50;
    cfg.max_corr_distance  = 1.0;
    cfg.use_spherical_grid = false;
    cfg.accumulate_frames  = 0;

    auto localization = radar_lidar::localization::LocalizationStage(map, cfg);
    auto frame        = make_frame_from_cloud(map->pcl_cloud());

    auto result = localization.process(frame);
    ASSERT_TRUE(result.has_value()) << result.error();

    const auto& T = result->t_map_lidar;
    auto trans    = T.translation();
    EXPECT_NEAR(trans.x(), 0.0, 0.05);
    EXPECT_NEAR(trans.y(), 0.0, 0.05);
    EXPECT_NEAR(trans.z(), 0.0, 0.05);
}

TEST_F(LocalizationTest, KnownTranslation) {
    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();
    auto map = *map_result;

    radar_lidar::config::LocalizationConfig cfg;
    cfg.num_threads        = 2;
    cfg.max_iterations     = 100;
    cfg.max_corr_distance  = 2.0;
    cfg.use_spherical_grid = false;
    cfg.accumulate_frames  = 0;

    auto localization = radar_lidar::localization::LocalizationStage(map, cfg);

    Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();
    init_pose.translation()     = Eigen::Vector3d(-0.5, -0.3, 0.0);
    localization.set_initial_pose(init_pose);

    const Eigen::Vector3d shift(0.5, 0.3, 0.0);
    std::vector<Eigen::Vector3d> points;
    const auto& pts = map->pcl_cloud().points;
    points.reserve(pts.size());
    for (const auto& pt : pts) {
        points.emplace_back(pt.x + shift.x(), pt.y + shift.y(), pt.z + shift.z());
    }
    auto frame = radar_lidar::types::Frame { .points = std::move(points) };

    auto result = localization.process(frame);
    ASSERT_TRUE(result.has_value()) << result.error();

    auto trans = result->t_map_lidar.translation();
    EXPECT_NEAR(trans.x(), -0.5, 0.1) << "Expected T.x ~ -0.5, got " << trans.x();
    EXPECT_NEAR(trans.y(), -0.3, 0.1) << "Expected T.y ~ -0.3, got " << trans.y();
    EXPECT_NEAR(trans.z(), 0.0, 0.1) << "Expected T.z ~  0.0, got " << trans.z();
}

TEST_F(LocalizationTest, ConfiguredInitialPoseDoesNotLockBeforeRegistration) {
    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();
    auto map = *map_result;

    radar_lidar::config::LocalizationConfig cfg;
    cfg.num_threads        = 2;
    cfg.max_iterations     = 100;
    cfg.max_corr_distance  = 2.0;
    cfg.use_spherical_grid = false;
    cfg.accumulate_frames  = 0;
    cfg.has_initial_pose   = true;
    cfg.initial_tx         = -0.5;
    cfg.initial_ty         = -0.3;

    auto localization = radar_lidar::localization::LocalizationStage(map, cfg);
    EXPECT_FALSE(localization.is_locked());

    const Eigen::Vector3d shift(0.5, 0.3, 0.0);
    std::vector<Eigen::Vector3d> points;
    const auto& pts = map->pcl_cloud().points;
    points.reserve(pts.size());
    for (const auto& pt : pts) {
        points.emplace_back(pt.x + shift.x(), pt.y + shift.y(), pt.z + shift.z());
    }

    auto result = localization.process(radar_lidar::types::Frame { .points = std::move(points) });
    ASSERT_TRUE(result.has_value()) << result.error();
    EXPECT_NE(result->fitness_score, 0.0);
}

TEST_F(LocalizationTest, FitnessEqualToThresholdLocks) {
    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();

    radar_lidar::config::LocalizationConfig cfg;
    cfg.lock_fitness  = 0.2;
    auto localization = radar_lidar::localization::LocalizationStage(*map_result, cfg);

    radar_lidar::types::PoseEstimate result;
    result.converged     = true;
    result.fitness_score = 0.2;
    radar_lidar::localization::LocalizationStageTestPeer::apply_result(localization, result);

    EXPECT_TRUE(localization.is_locked());
    EXPECT_EQ(localization.state(), radar_lidar::localization::RegistrationState::LOCKED);
    EXPECT_TRUE(localization.failure_reason().empty());
}

TEST_F(LocalizationTest, SetInitialPoseAfterLockRequiresRealRegistration) {
    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();

    radar_lidar::config::LocalizationConfig cfg;
    cfg.num_threads        = 2;
    cfg.max_iterations     = 100;
    cfg.max_corr_distance  = 2.0;
    cfg.use_spherical_grid = false;
    cfg.accumulate_frames  = 0;
    auto localization      = radar_lidar::localization::LocalizationStage(*map_result, cfg);

    radar_lidar::types::PoseEstimate accepted;
    accepted.converged = true;
    radar_lidar::localization::LocalizationStageTestPeer::apply_result(localization, accepted);
    ASSERT_TRUE(localization.is_locked());

    Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
    initial_pose.translation()     = Eigen::Vector3d(-0.5, -0.3, 0.0);
    localization.set_initial_pose(initial_pose);

    EXPECT_FALSE(localization.is_locked());
    EXPECT_EQ(localization.state(), radar_lidar::localization::RegistrationState::REGISTERING);
    EXPECT_TRUE(localization.failure_reason().empty());

    auto frame = make_frame_from_cloud(map_result.value()->pcl_cloud());
    for (auto& point : frame.points) {
        point += Eigen::Vector3d(0.5, 0.3, 0.0);
    }
    auto result = localization.process(frame);
    ASSERT_TRUE(result.has_value()) << result.error();
    EXPECT_NE(result->fitness_score, 0.0);
}

TEST_F(LocalizationTest, SetInitialPoseClearsPreviousRejectionReason) {
    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();
    auto localization = radar_lidar::localization::LocalizationStage(*map_result, { });

    radar_lidar::types::PoseEstimate rejected;
    rejected.converged = false;
    radar_lidar::localization::LocalizationStageTestPeer::apply_result(localization, rejected);
    ASSERT_FALSE(localization.failure_reason().empty());

    localization.set_initial_pose(Eigen::Isometry3d::Identity());

    EXPECT_FALSE(localization.is_locked());
    EXPECT_EQ(localization.state(), radar_lidar::localization::RegistrationState::REGISTERING);
    EXPECT_TRUE(localization.failure_reason().empty());
}

TEST_F(LocalizationTest, AcceptedResultKeepsRegisteringWhenLockStrategyDisabled) {
    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();
    radar_lidar::config::LocalizationConfig cfg;
    cfg.use_lock_strategy = false;
    auto localization     = radar_lidar::localization::LocalizationStage(*map_result, cfg);

    radar_lidar::types::PoseEstimate accepted;
    accepted.converged                 = true;
    accepted.t_map_lidar.translation() = Eigen::Vector3d(2.0, 0.0, 0.0);
    radar_lidar::localization::LocalizationStageTestPeer::apply_result(localization, accepted);

    EXPECT_FALSE(localization.is_locked());
    EXPECT_EQ(localization.state(), radar_lidar::localization::RegistrationState::REGISTERING);
    EXPECT_TRUE(localization.failure_reason().empty());
    EXPECT_TRUE(radar_lidar::localization::LocalizationStageTestPeer::previous_pose(localization)
            .isApprox(accepted.t_map_lidar));
}

TEST_F(LocalizationTest, RejectedResultKeepsPreviousEstimateWhenLockStrategyDisabled) {
    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();
    radar_lidar::config::LocalizationConfig cfg;
    cfg.use_lock_strategy = false;
    auto localization     = radar_lidar::localization::LocalizationStage(*map_result, cfg);

    Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();
    initial_pose.translation()     = Eigen::Vector3d(1.0, 0.0, 0.0);
    localization.set_initial_pose(initial_pose);
    radar_lidar::types::PoseEstimate rejected;
    rejected.converged = false;
    radar_lidar::localization::LocalizationStageTestPeer::apply_result(localization, rejected);

    EXPECT_FALSE(localization.is_locked());
    EXPECT_EQ(localization.state(), radar_lidar::localization::RegistrationState::REGISTERING);
    EXPECT_FALSE(localization.failure_reason().empty());
    EXPECT_TRUE(radar_lidar::localization::LocalizationStageTestPeer::previous_pose(localization)
            .isApprox(initial_pose));
}

TEST_F(LocalizationTest, InvalidRegistrationResultsDoNotLockOrReplaceInitialEstimate) {
    struct RejectedResult {
        const char* name;
        radar_lidar::types::PoseEstimate result;
    };

    radar_lidar::types::PoseEstimate non_converged;
    non_converged.converged = false;

    radar_lidar::types::PoseEstimate nan_fitness;
    nan_fitness.converged     = true;
    nan_fitness.fitness_score = std::numeric_limits<double>::quiet_NaN();

    radar_lidar::types::PoseEstimate non_finite_transform;
    non_finite_transform.converged                     = true;
    non_finite_transform.t_map_lidar.translation().x() = std::numeric_limits<double>::infinity();

    radar_lidar::types::PoseEstimate excessive_translation;
    excessive_translation.converged                 = true;
    excessive_translation.t_map_lidar.translation() = Eigen::Vector3d(40.1, 0.0, 0.0);

    radar_lidar::types::PoseEstimate excessive_rotation;
    excessive_rotation.converged = true;
    excessive_rotation.t_map_lidar.linear() =
        Eigen::AngleAxisd(1.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    const std::array rejected_results {
        RejectedResult { "non-converged", non_converged },
        RejectedResult { "NaN fitness", nan_fitness },
        RejectedResult { "non-finite transform", non_finite_transform },
        RejectedResult { "excessive translation", excessive_translation },
        RejectedResult { "excessive rotation", excessive_rotation },
    };

    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();

    for (const auto& rejected : rejected_results) {
        SCOPED_TRACE(rejected.name);
        radar_lidar::config::LocalizationConfig cfg;
        cfg.has_initial_pose  = true;
        cfg.initial_tx        = 1.0;
        cfg.initial_ty        = 2.0;
        cfg.initial_tz        = 3.0;
        cfg.lock_fitness      = 0.2;
        cfg.max_translation_m = 40.0;
        cfg.max_rotation_rad  = 1.0;
        auto localization     = radar_lidar::localization::LocalizationStage(*map_result, cfg);

        radar_lidar::localization::LocalizationStageTestPeer::apply_result(
            localization, rejected.result);

        EXPECT_FALSE(localization.is_locked());
        EXPECT_EQ(localization.state(), radar_lidar::localization::RegistrationState::REGISTERING);
        EXPECT_FALSE(localization.failure_reason().empty());
        Eigen::Isometry3d expected_initial_pose = Eigen::Isometry3d::Identity();
        expected_initial_pose.translation()     = Eigen::Vector3d(1.0, 2.0, 3.0);
        EXPECT_TRUE(
            radar_lidar::localization::LocalizationStageTestPeer::previous_pose(localization)
                .isApprox(expected_initial_pose));
    }
}

TEST_F(LocalizationTest, KnownRotation) {
    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value()) << map_result.error();
    auto map = *map_result;

    radar_lidar::config::LocalizationConfig cfg;
    cfg.num_threads        = 2;
    cfg.max_iterations     = 100;
    cfg.max_corr_distance  = 2.0;
    cfg.use_spherical_grid = false;
    cfg.accumulate_frames  = 0;

    auto localization = radar_lidar::localization::LocalizationStage(map, cfg);

    Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();
    init_pose.linear() =
        Eigen::AngleAxisd(deg_to_rad(-15.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    localization.set_initial_pose(init_pose);

    auto frame = make_frame_from_cloud(map->pcl_cloud());

    auto result = localization.process(frame);
    ASSERT_TRUE(result.has_value()) << result.error();

    const auto& R_result = result->t_map_lidar.rotation();
    EXPECT_TRUE(R_result.isApprox(Eigen::Matrix3d::Identity(), 1e-1)) << "Expected localization to "
                                                                         "recover identity "
                                                                         "rotation";
}

TEST_F(LocalizationTest, EmptyScanReturnsError) {
    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value());
    auto map = *map_result;

    radar_lidar::config::LocalizationConfig cfg;
    auto localization = radar_lidar::localization::LocalizationStage(map, cfg);

    radar_lidar::types::Frame empty_frame;
    auto result = localization.process(empty_frame);
    EXPECT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("Empty scan"), std::string::npos);
}

TEST_F(LocalizationTest, ResetStateReflectsMapAvailability) {
    auto map_result = radar_lidar::map_data::MapData::load(map_pcd_, 0.1);
    ASSERT_TRUE(map_result.has_value());
    auto map = *map_result;

    radar_lidar::config::LocalizationConfig cfg;
    auto localization = radar_lidar::localization::LocalizationStage(map, cfg);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation()     = Eigen::Vector3d(1.0, 2.0, 3.0);
    localization.set_initial_pose(pose);

    localization.reset();
    EXPECT_FALSE(localization.is_locked());
    EXPECT_EQ(localization.state(), radar_lidar::localization::RegistrationState::REGISTERING);

    auto mapless = radar_lidar::localization::LocalizationStage(nullptr, cfg);
    mapless.reset();
    EXPECT_FALSE(mapless.is_locked());
    EXPECT_EQ(mapless.state(), radar_lidar::localization::RegistrationState::INITIALIZING);
}
