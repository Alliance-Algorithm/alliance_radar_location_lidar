# 设计：坐标完全信相机（点云仅配准）

日期：2026-08-05
分支：`feat/camera-only`
状态：已批准

## 背景

当前 fusion 输出坐标有两条路径：相机池（`tracks_`）与雷达聚类池（`lidar_tracks_`），雷达池在被相机贴类别后会**覆盖**相机池位置。比赛链路要求：官方坐标 100% 相信相机检测（相机输出即为最终坐标），雷达（点云）只用于 GICP 配准定位，聚类不再参与坐标输出。

## 改动范围

### radar_fusion（融合端）

1. `radar_fusion_node.cpp`：
   - 删除 `/lidar/cluster` 订阅（`sub_cluster_`）与 `on_cluster` 回调
   - 删除 `process_lidar_clusters`、`publish_lidar_tracks`
   - `on_camera_detection` 删除"相机给雷达聚类贴类别"循环
   - `publish_lidar_location` 只填相机池 `tracks_`（删除 `fill_slots(lidar_tracks)`）
   - 10Hz location_timer_ 回调删除 `publish_lidar_tracks(lidar_tracks_, stamp)`
2. `radar_fusion_node.hpp`：删除 `lidar_tracks_`、`next_lidar_track_id_`、`sub_cluster_` 及相关方法声明
3. `test_fusion_node.cpp`：删除/更新雷达聚类相关用例，验证聚类消息不再影响坐标输出

### radar_lidar（雷达端）

4. `runtime.yaml`：新增配置开关 `enable_cluster: false`
5. `radar_lidar_node.cpp`：开关为 false 时跳过 `cluster_stage_.process`，不发布 `/lidar/cluster`（代码保留，便于恢复）
6. radar_lidar 相关测试适配（如有 cluster 用例）

### 不改动

- 相机链路（radar_camera）：检测 → 坐标转换 → `/radar_camera/robot_pose` 原样
- 雷达定位（GICP 配准、开局验证锁定、watchdog 永久锁定）：原样
- `publish_lidar_location` 的默认位置兜底（`fill_default_positions`）：原样
- `update_fusion_mode` / 状态消息：原样（相机 stale 时 DEGRADED 语义不变）

## 行为变化

| 场景 | 之前 | 之后 |
|------|------|------|
| 相机+雷达都有 | 雷达池覆盖相机位置 | 相机坐标为准 |
| 只有雷达聚类 | 贴类别后进坐标 | 聚类不再进坐标 |
| 相机无检测 | 默认位置兜底 | 默认位置兜底（不变） |
| 雷达仅配准 | — | 聚类计算停止，省 CPU |

## 验证

1. `colcon build --packages-select radar_fusion radar_lidar` + 相关测试通过
2. 模拟/实测：相机检测输出即坐标；无聚类消息时坐标不受影响；雷达配准定位正常
