#include <filesystem>
#include <gtest/gtest.h>

#include "radar_camera/projector.hpp"

namespace {

auto make_camera_cfg() -> radar_camera::camera_config::CameraConfig {
    radar_camera::camera_config::CameraConfig cfg;
    cfg.enemy_color             = "blue";
    cfg.hero_class_id           = 1;
    cfg.engine_class_id         = 3;
    cfg.infantry_3_class_id     = 5;
    cfg.infantry_4_class_id     = 7;
    cfg.sentry_class_id        = 9;
    cfg.drone_class_id          = 11;
    cfg.camera_matrix           = { 500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0 };
    cfg.distortion_coefficients = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    // Camera at (0,0,4) looking down -Z toward ground plane z=0 in map frame:
    // identity rotation keeps optical +Z forward; use pitch so ray hits plane.
    cfg.rotation    = { 0.0, 0.0, 0.0 };
    cfg.translation = { 0.0, 0.0, 4.0 };
    return cfg;
}

auto fixture_mesh() -> std::string {
    return (std::filesystem::path(RADAR_CAMERA_TEST_SOURCE_DIR) / "fixtures" / "ground_plane.obj")
        .string();
}

} // namespace

TEST(ProjectorInitTest, RejectsBadCameraMatrix) {
    radar_camera::projection::Projector projector;
    auto cfg          = make_camera_cfg();
    cfg.camera_matrix = { 1.0, 2.0 };
    auto ret          = projector.proj_init_camera(cfg);
    ASSERT_FALSE(ret.has_value());
    EXPECT_NE(ret.error().find("camera_matrix"), std::string::npos);
}

TEST(ProjectorInitTest, AcceptsValidCamera) {
    radar_camera::projection::Projector projector;
    auto ret = projector.proj_init_camera(make_camera_cfg());
    ASSERT_TRUE(ret.has_value()) << ret.error();
}

TEST(ProjectorPostprocessTest, MapsClassIdsToSlots) {
    radar_camera::projection::Projector projector;
    ASSERT_TRUE(projector.proj_init_camera(make_camera_cfg()));

    std::vector<radar_camera::detection::Detection> dets {
        { .center = { 100, 100 }, .id = 1, .confidence = 0.91f },
        { .center = { 200, 200 }, .id = 3, .confidence = 0.82f },
        { .center = { 300, 300 }, .id = 11, .confidence = 0.77f },
    };
    std::vector<std::optional<cv::Point2d>> projected {
        cv::Point2d { 1.0, 2.0 },
        cv::Point2d { 3.0, 4.0 },
        cv::Point2d { 5.0, 6.0 },
    };

    auto pose = projector.proj_postprocess(projected, dets);
    ASSERT_TRUE(pose.has_value()) << pose.error();
    EXPECT_NEAR(pose->hero_position.x, 1.0, 1e-9);
    EXPECT_NEAR(pose->hero_position.y, 2.0, 1e-9);
    EXPECT_NEAR(pose->hero_confidence, 0.91, 1e-5);
    EXPECT_NEAR(pose->engine_position.x, 3.0, 1e-9);
    EXPECT_NEAR(pose->engine_position.y, 4.0, 1e-9);
    EXPECT_NEAR(pose->engine_confidence, 0.82, 1e-5);
    EXPECT_NEAR(pose->drone_position.x, 5.0, 1e-9);
    EXPECT_NEAR(pose->drone_position.y, 6.0, 1e-9);
    EXPECT_NEAR(pose->drone_confidence, 0.77, 1e-5);
    EXPECT_NEAR(pose->sentry_confidence, 0.0, 1e-9);
}

TEST(ProjectorPostprocessTest, LastDuplicateClassWins) {
    radar_camera::projection::Projector projector;
    ASSERT_TRUE(projector.proj_init_camera(make_camera_cfg()));

    std::vector<radar_camera::detection::Detection> dets {
        { .center = { 0, 0 }, .id = 1, .confidence = 0.5f },
        { .center = { 0, 0 }, .id = 1, .confidence = 0.9f },
    };
    std::vector<std::optional<cv::Point2d>> projected {
        cv::Point2d { 1.0, 1.0 },
        cv::Point2d { 9.0, 9.0 },
    };
    auto pose = projector.proj_postprocess(projected, dets);
    ASSERT_TRUE(pose.has_value()) << pose.error();
    EXPECT_NEAR(pose->hero_position.x, 9.0, 1e-9);
    EXPECT_NEAR(pose->hero_confidence, 0.9, 1e-5);
}

TEST(ProjectorMeshTest, LoadsGroundPlaneAndIntersects) {
    radar_camera::projection::Projector projector;
    auto cam = make_camera_cfg();
    // Look straight down: roll/pitch so optical axis points toward -Z in map.
    // With identity R and t=(0,0,4), optical +Z in camera = +Z map, ray goes up.
    // Pitch +pi so camera looks toward -Z (down onto z=0 plane).
    cam.rotation = { 0.0, 3.141592653589793, 0.0 };
    ASSERT_TRUE(projector.proj_init_camera(cam)) << "camera init";

    radar_camera::projection_config::ProjectionConfig proj_cfg;
    proj_cfg.mesh_path = fixture_mesh();
    ASSERT_TRUE(std::filesystem::exists(proj_cfg.mesh_path)) << proj_cfg.mesh_path;
    auto map_ret = projector.proj_init_map(proj_cfg);
    ASSERT_TRUE(map_ret.has_value()) << map_ret.error();

    // Principal point should ray-hit near origin of ground plane
    auto ray = projector.proj_pixel_to_ray(cv::Point2d { 320.0, 240.0 });
    ASSERT_TRUE(ray.has_value()) << ray.error();
    auto hit = projector.proj_map_intersect(*ray);
    ASSERT_TRUE(hit.has_value()) << hit.error();
    EXPECT_NEAR(hit->x, 0.0, 0.5);
    EXPECT_NEAR(hit->y, 0.0, 0.5);
}

TEST(ProjectorMeshTest, MissingMeshFails) {
    radar_camera::projection::Projector projector;
    radar_camera::projection_config::ProjectionConfig proj_cfg;
    proj_cfg.mesh_path = "/tmp/radar_camera_missing_mesh.obj";
    auto ret           = projector.proj_init_map(proj_cfg);
    ASSERT_FALSE(ret.has_value());
    EXPECT_NE(ret.error().find("not found"), std::string::npos);
}
