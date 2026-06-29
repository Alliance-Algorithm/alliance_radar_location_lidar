#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace radar::fusion {

/// 2D 常速卡尔曼滤波器状态: [x, y, vx, vy]
struct KalmanState {
    Eigen::Vector4d x = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity();

    /// ROS message timestamp (ns from msg->header.stamp). Used for dt calculation and staleness.
    int64_t last_update_ns = 0;
    int track_id           = -1;
    int hit_count          = 0;
    int miss_count         = 0;
    int color              = -1; // 0=blue, 2=red, -1=unknown
    int number             = -1; // robot number 0-5

    [[nodiscard]] auto position() const -> Eigen::Vector2d { return x.head<2>(); }
    [[nodiscard]] auto velocity() const -> Eigen::Vector2d { return x.tail<2>(); }
    [[nodiscard]] auto is_stale(int64_t now_ns, double timeout_sec) const -> bool;
};

/// 单个目标的 2D 常速卡尔曼滤波器
class KalmanTracker {
public:
    explicit KalmanTracker(int track_id);

    /// Predict state to the given ROS message time (ns from msg->header.stamp).
    /// No-op when now_ns is before last_update_ns.
    auto predict(int64_t now_ns) -> bool;

    /// Update state with a measurement at the given ROS message time.
    void update(const Eigen::Vector2d& measurement, int64_t now_ns);

    [[nodiscard]] auto distance_squared_to(const Eigen::Vector2d& measurement) const -> double;

    [[nodiscard]] auto state() const -> const KalmanState& { return state_; }

    void set_color(int color) { state_.color = color; }
    void set_number(int number) { state_.number = number; }

private:
    KalmanState state_;

    // 过程噪声
    double process_noise_pos_ = 0.1;
    double process_noise_vel_ = 0.5;
    // 观测噪声
    double measurement_noise_ = 0.25;
};

} // namespace radar::fusion
