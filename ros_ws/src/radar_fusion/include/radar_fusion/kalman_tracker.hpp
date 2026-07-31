#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "radar_fusion/data_format.hpp"

namespace radar_fusion::kalman_tracker {

/// 单个目标的 2D 常速卡尔曼滤波器
class KalmanTracker {
public:
    explicit KalmanTracker(int track_id);

    /// 预测到指定时间
    void predict(int64_t now_ns);

    /// 用观测更新
    void update(const Eigen::Vector2d& measurement, int64_t now_ns, int min_hits_to_confirm);

    void mark_missed(int max_misses_before_delete);

    [[nodiscard]] auto distance_squared_to(const Eigen::Vector2d& measurement) const -> double;

    auto state() const -> const KalmanState& { return state_; }

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

} // namespace radar_fusion::kalman_tracker
