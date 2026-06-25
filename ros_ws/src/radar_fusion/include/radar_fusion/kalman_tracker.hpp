#pragma once

#include <chrono>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace radar::fusion {

/// 2D 常速卡尔曼滤波器状态: [x, y, vx, vy]
struct KalmanState {
    Eigen::Vector4d x = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P = Eigen::Matrix4d::Identity();

    std::chrono::steady_clock::time_point last_update;
    int track_id   = -1;
    int hit_count  = 0;
    int miss_count = 0;
    int color      = -1; // 0=blue, 2=red, -1=unknown
    int number     = -1; // robot number 0-5

    [[nodiscard]] auto position() const -> Eigen::Vector2d { return x.head<2>(); }
    [[nodiscard]] auto velocity() const -> Eigen::Vector2d { return x.tail<2>(); }
    [[nodiscard]] auto is_stale(double timeout_sec) const -> bool;
};

/// 单个目标的 2D 常速卡尔曼滤波器
class KalmanTracker {
public:
    explicit KalmanTracker(int track_id);

    /// 预测到指定时间
    void predict(std::chrono::steady_clock::time_point now);

    /// 用观测更新
    void update(const Eigen::Vector2d& measurement, std::chrono::steady_clock::time_point now);

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
