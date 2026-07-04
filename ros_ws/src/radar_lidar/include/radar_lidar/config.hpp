#pragma once

#include <string>

namespace radar::config {

/// @brief 轴对齐包围盒 (AABB) ROI 参数
/// 被 LocalizationConfig 和 DynamicCloudConfig 共享嵌入,避免裁剪逻辑重复
struct RoiBounds {
    bool use_roi = false;
    double x_min = 0, x_max = 30;
    double y_min = -15, y_max = 15;
    double z_min = 0, z_max = 7;
};

struct LocalizationConfig {
    double voxel_leaf_size   = 0.1;
    double max_corr_distance = 1.0;
    int max_iterations       = 48;
    double rotation_eps      = 0.03;
    double translation_eps   = 0.1;
    int num_threads          = 4;
    bool verbose             = false;

    // 球面网格预处理（source cloud）
    bool use_spherical_grid   = true; // false = 直接传原始点给 GICP
    double spherical_grid_deg = 0.1;  // 角度分辨率（度）
    int accumulate_frames     = 20;   // 滑动窗口帧数（0=不累积）

    // 一次性锁定（fitness < lock_fitness 后停止配准）
    bool use_lock_strategy = false; // 固定雷达场景可启用
    double lock_fitness    = 0.2;   // fitness 低于此值后锁定

    // ROI 裁剪（source cloud，坐标系同 scan）
    RoiBounds roi;
};

} // namespace radar::config
