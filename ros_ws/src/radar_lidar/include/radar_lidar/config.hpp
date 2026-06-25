#pragma once

#include <string>

namespace radar::config {

struct LocalizationConfig {
    double voxel_leaf_size   = 0.1;
    double max_corr_distance = 1.0;
    int max_iterations       = 48;
    double rotation_eps      = 0.001745;
    double translation_eps   = 0.001;
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
    bool use_roi     = false;
    double roi_x_min = 0, roi_x_max = 30;
    double roi_y_min = -15, roi_y_max = 15;
    double roi_z_min = 0, roi_z_max = 7;
};

} // namespace radar::config
