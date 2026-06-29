#include "radar_fusion/kalman_tracker.hpp"

#include <Eigen/Cholesky>

namespace radar::fusion {

KalmanTracker::KalmanTracker(int track_id) { state_.track_id = track_id; }

void KalmanTracker::predict(int64_t now_ns) {
    double dt = static_cast<double>(now_ns - state_.last_update_ns) / 1e9;
    if (dt < 0) dt = 0;

    // 状态转移矩阵 F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2)           = dt;
    F(1, 3)           = dt;

    state_.x = F * state_.x;

    // 过程噪声 Q
    double q1         = process_noise_pos_ * process_noise_pos_;
    double q2         = process_noise_vel_ * process_noise_vel_;
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0, 0)           = q1 * dt * dt * dt / 3.0;
    Q(0, 2)           = q1 * dt * dt / 2.0;
    Q(1, 1)           = q1 * dt * dt * dt / 3.0;
    Q(1, 3)           = q1 * dt * dt / 2.0;
    Q(2, 0)           = q1 * dt * dt / 2.0;
    Q(2, 2)           = q2 * dt;
    Q(3, 1)           = q1 * dt * dt / 2.0;
    Q(3, 3)           = q2 * dt;

    state_.P = F * state_.P * F.transpose() + Q;
}

void KalmanTracker::update(
    const Eigen::Vector2d& measurement, int64_t now_ns, int min_hits_to_confirm) {
    // 观测矩阵 H = [[1,0,0,0],[0,1,0,0]]
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0)                       = 1.0;
    H(1, 1)                       = 1.0;

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * measurement_noise_;

    Eigen::Vector2d y             = measurement - H * state_.x;
    Eigen::Matrix2d S             = H * state_.P * H.transpose() + R;
    const Eigen::Matrix2d S_inv   = S.ldlt().solve(Eigen::Matrix2d::Identity());
    Eigen::Matrix<double, 4, 2> K = state_.P * H.transpose() * S_inv;

    state_.x              = state_.x + K * y;
    state_.P              = (Eigen::Matrix4d::Identity() - K * H) * state_.P;
    state_.last_update_ns = now_ns;
    state_.hit_count++;
    state_.miss_count = 0;
    if (state_.hit_count >= min_hits_to_confirm) {
        state_.lifecycle = TrackLifecycle::Confirmed;
    }
}

void KalmanTracker::mark_missed(int max_misses_before_delete) {
    state_.miss_count++;
    if (state_.lifecycle == TrackLifecycle::Tentative
        || state_.miss_count >= max_misses_before_delete) {
        state_.lifecycle = TrackLifecycle::Deleted;
    }
}

auto KalmanTracker::distance_squared_to(const Eigen::Vector2d& measurement) const -> double {
    return (state_.x.head<2>() - measurement).squaredNorm();
}

auto KalmanState::is_stale(int64_t now_ns, double timeout_sec) const -> bool {
    double age = static_cast<double>(now_ns - last_update_ns) / 1e9;
    return age > timeout_sec;
}

} // namespace radar::fusion
