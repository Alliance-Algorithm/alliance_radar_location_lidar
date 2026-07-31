#pragma once

#include <cstdint>

#include <Eigen/Core>

namespace radar_fusion::fusion_config {

enum class FusionMode {
    RADAR_ONLY,
    RADAR_CAMERA,
    DEGRADED,
};

struct FusionConfig {
    double gate_distance         = 1.0;
    double track_timeout_sec     = 1.5;
    int min_hits_to_confirm      = 3;
    int max_misses_before_delete = 2;
    int max_tracks               = 20;
    bool enable_camera_fusion    = false;
    double camera_timeout_sec    = 1.5;
    double camera_lidar_consistency_distance = 1.0;
    double identity_retention_sec = 1.5;
    double map_to_rm_offset_x    = 14.0;
    double map_to_rm_offset_y    = 7.5;
};

} // namespace radar_fusion::fusion_config

namespace radar_fusion::camera_observation {

enum class Team : std::uint8_t {
    UNKNOWN = 0,
    RED = 1,
    BLUE = 2,
};

enum class SemanticClass : std::uint8_t {
    UNKNOWN = 0,
    HERO = 1,
    ENGINEER = 2,
    INFANTRY_3 = 3,
    INFANTRY_4 = 4,
    AERIAL = 5,
    SENTRY = 6,
};

struct CameraObservation {
    double x          = 0.0;
    double y          = 0.0;
    double z          = 0.0;
    double confidence = 0.0;
    Team team = Team::UNKNOWN;
    SemanticClass semantic_class = SemanticClass::UNKNOWN;
};

} // namespace radar_fusion::camera_observation

namespace radar_fusion::kalman_tracker {

enum class TrackLifecycle {
    TENTATIVE,
    CONFIRMED,
    DELETED,
};

struct KalmanState {
    Eigen::Vector4d x = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity();

    int64_t last_update_ns   = 0;
    int track_id             = -1;
    int hit_count            = 0;
    int miss_count           = 0;
    camera_observation::Team team = camera_observation::Team::UNKNOWN;
    camera_observation::SemanticClass semantic_class = camera_observation::SemanticClass::UNKNOWN;
    TrackLifecycle lifecycle = TrackLifecycle::TENTATIVE;

    [[nodiscard]] auto position() const -> Eigen::Vector2d { return x.head<2>(); }
    [[nodiscard]] auto velocity() const -> Eigen::Vector2d { return x.tail<2>(); }
    [[nodiscard]] auto is_stale(int64_t now_ns, double timeout_sec) const -> bool;
    [[nodiscard]] auto is_deleted() const -> bool { return lifecycle == TrackLifecycle::DELETED; }
    [[nodiscard]] auto is_confirmed() const -> bool {
        return lifecycle == TrackLifecycle::CONFIRMED;
    }
};

} // namespace radar_fusion::kalman_tracker

namespace radar_fusion::location {

auto meters_to_cm(double meters, double offset_m) -> std::uint16_t;

} // namespace radar_fusion::location
