# Odin1 直出聚类目标检测调参节点（odin_tune_node）设计

日期：2026-08-02
状态：已批准

## 背景与目标

比赛主链路 `radar_lidar_node` 的聚类目标检测依赖：
1. 地图（`map_path` 强制加载，缺失则 Fatal）
2. 基于地图的 `DynamicCloudStage` 动态点提取

真机联调场景下**没有地图可用**，且需要为聚类目标检测**调优参数**。本设计提供一个独立的赛前测试节点 `odin_tune_node`，订阅 Odin1 直出原始点云（`/odin1/cloud_raw`），用**背景模型差分**（基于 odometry 对齐的多帧累积背景）替代基于地图的动态点提取，复用现有 `ClusterStage` 欧氏聚类，全部参数通过 ROS2 动态参数实时调整，配合 Foxglove 人工观察评估效果。

**核心约束：绝不影响比赛主链路。** 不改动 `radar_lidar_node`、`dynamic_cloud_stage`、`cluster_stage` 的任何代码；该节点只在赛前真机调参时手动启动，比赛 launch 不加载它。

## 架构

新节点 `odin_tune_node`，位于 `ros_ws/src/radar_lidar/tools/`（与现有 `offline_detection_node` 同级），组件：

| 组件 | 来源 | 说明 |
|------|------|------|
| `PoseBuffer` | 新开发 | 按时间戳缓存 odometry 位姿，支持时间查询 |
| `BackgroundModel` | 新开发（核心） | 滑动窗口累积 N 帧，用 odometry 对齐到当前帧坐标系 |
| `FrameDifferencer` | 新开发（核心） | 当前帧 vs 背景模型 KdTree 最近邻差分，距离 > 阈值判为动态点 |
| `cluster::ClusterStage` | 复用现有 | 欧氏聚类 + 质心 + AABB |
| ROI 过滤逻辑 | 复用现有 | 场地 ROI：x:[-11,14] y:[-7.5,7.5] z:[0,1.4]，排除角区/坡道 |

### 订阅

- `/odin1/cloud_raw`：Odin1 直出原始点云（自定义格式 x,y,z,intensity,confidence,offset_time）
- `/odin1/odometry`：nav_msgs/Odometry，用于把历史帧对齐到当前帧坐标系

驱动配置复用现有 `radar_bringup/config/lidar/odin_driver.yaml`（需开启 `senddtof` 与 `sendodom`）。

### 发布

- `/odin_tune/dynamic`：动态点云（PointCloud2）
- `/odin_tune/background`：背景模型点云（PointCloud2，调试用）
- `/odin_tune/clusters`：聚类质心（PointCloud2）
- `/odin_tune/cluster_viz`：聚类 AABB 边框（MarkerArray）
- `/odin_tune/diag`：诊断（dynamic 点数 / cluster 数 / 耗时）

## 参数（全部 ROS2 动态参数，`ros2 param set` 实时生效）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `bg_num_frames` | 10 | 背景模型累积帧数（滑动窗口） |
| `diff_threshold` | 0.3 | 帧差距离阈值 (m)，大于此值判为动态点 |
| `conf_threshold` | 35 | cloud_raw confidence 过滤（与 Odin 驱动推荐值一致） |
| `voxel_leaf` | 0.05 | 帧内体素下采样（0 = 关闭） |
| `roi_enabled` | true | 是否启用场地 ROI 过滤 |
| `cluster_tolerance` | 0.25 | 欧氏聚类容差 (m) |
| `min_cluster_size` | 5 | 聚类最小点数 |
| `max_cluster_size` | 1000 | 聚类最大点数 |
| `odom_topic` | `/odin1/odometry` | 姿态源话题（可选 `/odin1/odometry_high`） |

## 数据流

```
cloud_raw ──→ confidence 过滤 ──→ 体素下采样 ──→ 查 PoseBuffer 得当前帧位姿
                                                       │
历史帧(带位姿) ←────── odometry 缓存 ──────────────────┘
      │ odometry 对齐到当前帧坐标系
      ▼
背景模型(滑动窗口 N 帧累积) ──→ KdTree 最近邻差分 ──→ 动态点
      ──→ ROI 过滤(可选) ──→ ClusterStage 聚类 ──→ /odin_tune/* 发布
```

## 错误处理

- odometry 缺失 / 时间戳不匹配 → 跳过该帧并告警，不崩溃
- 背景模型不足 N 帧 → 正常输出但不产生动态点（等待累积）
- cloud_raw 解析失败 → 跳过该帧，不影响主链路

## 测试与验证

**单元测试**（`radar_lidar/test/`，仿照现有 `test_radar_lidar_node.cpp` 风格）：
- `PoseBuffer`：时间戳查询、边界时间戳处理
- `BackgroundModel`：帧对齐累积、窗口滑动正确性
- `FrameDifferencer`：静态点不判动态、移动点判动态、阈值边界

**离线冒烟测试**：用现有 bag→pcd 工具或录一段 bag，先离线跑通再上真机。

**真机验收**：
- 静止场景 → 动态点为 0，聚类为空
- 目标走动 → 聚类稳定输出，AABB 框跟住目标
- 调参：`ros2 param set /odin_tune_node cluster_tolerance 0.15` 实时观察效果

## 交付物

1. `ros_ws/src/radar_lidar/tools/odin_tune_node.cpp` + 对应头文件
2. `ros_ws/src/radar_lidar/CMakeLists.txt` 添加构建目标（不影响主节点）
3. launch 文件 `odin_tune.launch.py`（放在 `ros_ws/src/radar_bringup/launch/`，与现有 `odin_localization.launch.py` 同约定，独立于比赛 launch）
4. 参数默认值 YAML：`ros_ws/src/radar_lidar/config/odin_tune.yaml`
5. 单元测试
6. 简短使用说明（README 或 docs/ 下）

## 明确不做（YAGNI）

- 不做帧差结果持久化/录制
- 不做自动调参/网格扫描（人工观察即可）
- 不改动 `radar_lidar_node`、`dynamic_cloud_stage`、`cluster_stage` 代码
- 不提供比赛期运行路径
