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
    double map_to_rm_offset_x    = 14.0;
    double map_to_rm_offset_y    = 7.5;
};

} // namespace radar_fusion::fusion_config

namespace radar_fusion::camera_observation {

struct CameraObservation {
    double x          = 0.0;
    double y          = 0.0;
    double z          = 0.0;
    double confidence = 0.0;
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
    int color                = -1;
    int number               = -1;
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
