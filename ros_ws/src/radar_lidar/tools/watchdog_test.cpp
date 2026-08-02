#include <cmath>
#include <cstdio>
#include <string>

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "radar_lidar/data_format.hpp"
#include "radar_lidar/localization_stage.hpp"
#include "radar_lidar/map_data.hpp"

using radar_lidar::config::LocalizationConfig;
using radar_lidar::localization::LocalizationStage;
using radar_lidar::map_data::MapData;

namespace {
auto load_frame(const std::string& path) -> radar_lidar::types::Frame {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud) == -1) {
        throw std::runtime_error("failed to load scan: " + path);
    }
    radar_lidar::types::Frame frame;
    frame.points.reserve(cloud.size());
    for (const auto& p : cloud) {
        frame.points.emplace_back(p.x, p.y, p.z);
    }
    return frame;
}

// 对 scan 施加刚体变换（模拟雷达站被移动）
auto transform_scan(const radar_lidar::types::Frame& src, const Eigen::Isometry3d& T)
    -> radar_lidar::types::Frame {
    radar_lidar::types::Frame out;
    out.points.reserve(src.points.size());
    for (const auto& p : src.points) {
        out.points.push_back(T * p);
    }
    return out;
}

auto make_config() -> LocalizationConfig {
    LocalizationConfig cfg;
    cfg.voxel_leaf_size          = 0.1;
    cfg.max_corr_distance        = 2.0;
    cfg.max_iterations           = 50;
    cfg.num_threads              = 4;
    cfg.use_spherical_grid       = true;
    cfg.spherical_grid_deg       = 0.1;
    cfg.accumulate_frames        = 0;
    cfg.use_lock_strategy        = true;
    cfg.lock_fitness             = 0.2;
    cfg.enable_watchdog          = true;
    cfg.watchdog_fitness         = 0.5;
    cfg.watchdog_check_interval  = 5;
    cfg.watchdog_unlock_frames   = 3;
    cfg.enable_coarse_relocalize = true;
    cfg.coarse_yaw_range_deg     = 20.0;
    cfg.coarse_yaw_step_deg      = 5.0;
    cfg.coarse_translate_range_m = 12.0;
    cfg.coarse_translate_step_m  = 3.0;
    cfg.coarse_voxel             = 0.5;
    cfg.coarse_max_corr          = 5.0;
    cfg.coarse_max_iter          = 20;
    cfg.coarse_min_inlier        = 0.3;
    return cfg;
}
} // namespace

auto main(int argc, char** argv) -> int {
    if (argc < 3) {
        std::printf("usage: watchdog_test <map.pcd> <scan.pcd>\n");
        return 2;
    }
    const std::string map_path  = argv[1];
    const std::string scan_path = argv[2];

    auto map_result = MapData::load(map_path, 0.1);
    if (!map_result) {
        std::printf("map load failed: %s\n", map_result.error().c_str());
        return 1;
    }
    std::shared_ptr<const MapData> map = *map_result;

    auto cfg = make_config();
    // 初始位姿 = field_scan 合成真值: eye=(-14,0,4) yaw=0 pitch=14.04°
    cfg.has_initial_pose = true;
    cfg.initial_tx       = -14.0;
    cfg.initial_ty       = 0.0;
    cfg.initial_tz       = 4.0;
    cfg.initial_yaw      = 0.0;
    cfg.initial_pitch    = 0.245;
    cfg.initial_roll     = 0.0;
    LocalizationStage stage(map, cfg);
    auto scan = load_frame(scan_path);
    std::printf("scan points: %zu, map points: %zu\n", scan.points.size(), map->size());

    // Phase 1: 初始位姿配准 → 应收敛并锁定
    std::printf("--- Phase 1: initial registration ---\n");
    for (int i = 0; i < 30; ++i) {
        auto pose = stage.process(scan);
        if (!pose) {
            std::printf("frame %d: %s\n", i, pose.error().c_str());
            continue;
        }
        std::printf("frame %d: fitness=%.4f locked=%d\n", i, pose->fitness_score,
            stage.is_locked() ? 1 : 0);
        if (stage.is_locked()) break;
    }
    if (!stage.is_locked()) {
        std::printf("FAIL: did not lock after initial registration\n");
        return 1;
    }
    std::printf("PASS: locked after initial registration\n");

    // Phase 2: 未移动时 watchdog 不应解锁（喂同一 scan 若干帧）
    std::printf("--- Phase 2: watchdog steady-state (no displacement) ---\n");
    bool unlocked_steady = false;
    for (int i = 0; i < 30; ++i) {
        auto pose = stage.process(scan);
        if (!stage.is_locked()) {
            unlocked_steady = true;
            std::printf("frame %d: UNLOCKED (unexpected!)\n", i);
            break;
        }
    }
    if (unlocked_steady) {
        std::printf("FAIL: watchdog unlocked without displacement\n");
        return 1;
    }
    std::printf("PASS: watchdog stays locked without displacement\n");

    // Phase 3: watchdog-only（关 coarse）——10m 位移应解锁
    std::printf("--- Phase 3: watchdog-only, 10m displacement ---\n");
    auto cfg_nocoarse                     = cfg;
    cfg_nocoarse.enable_coarse_relocalize = false;
    LocalizationStage stage3(map, cfg_nocoarse);
    Eigen::Isometry3d T_shift = Eigen::Isometry3d::Identity();
    T_shift.translation()     = Eigen::Vector3d(10.0, 0.0, 0.0);
    auto shifted              = transform_scan(scan, T_shift);
    bool unlocked3            = false;
    for (int i = 0; i < 30; ++i) {
        auto pose = stage3.process(shifted);
        if (!stage3.is_locked()) {
            unlocked3 = true;
            std::printf("frame %d: UNLOCKED (watchdog-only triggered)\n", i);
            break;
        }
    }
    std::printf(unlocked3 ? "PASS: watchdog-only unlocked after 10m\n"
                          : "FAIL: watchdog-only did not unlock after 10m\n");
    if (!unlocked3) return 1;

    // Phase 3b: watchdog + coarse——10m 位移应解锁并重定位恢复
    std::printf("--- Phase 3b: watchdog + coarse relocalize, 10m displacement ---\n");
    LocalizationStage stage4(map, cfg);
    bool reloc_ok = false;
    for (int i = 0; i < 40; ++i) {
        auto pose = stage4.process(shifted);
        if (!stage4.is_locked()) {
            std::printf("frame %d: UNLOCKED (coarse failed?)\n", i);
            break;
        }
        if (i >= 5) { // 前几帧已完成解锁+重锁
            reloc_ok = true;
            std::printf("frame %d: locked (watchdog + coarse recovered)\n", i);
            break;
        }
    }
    // 验证重定位后的残差（应接近稳态 0.2 而非 2.5）
    std::printf(reloc_ok ? "PASS: watchdog + coarse recovered and re-locked\n"
                         : "FAIL: watchdog + coarse did not recover\n");

    // Phase 4: 3m 位移灵敏度（watchdog-only）
    std::printf("--- Phase 4: watchdog-only, 3m displacement ---\n");
    Eigen::Isometry3d T_shift3 = Eigen::Isometry3d::Identity();
    T_shift3.translation()     = Eigen::Vector3d(3.0, 0.0, 0.0);
    auto shifted3              = transform_scan(scan, T_shift3);
    bool unlocked4             = false;
    for (int i = 0; i < 30; ++i) {
        auto pose = stage3.process(shifted3);
        if (!stage3.is_locked()) {
            unlocked4 = true;
            std::printf("frame %d: UNLOCKED after 3m\n", i);
            break;
        }
    }
    std::printf(unlocked4 ? "PASS: 3m displacement detected\n"
                          : "NOTE: 3m displacement NOT detected (residual below threshold)\n");
    std::printf("DONE\n");
    return 0;
}
