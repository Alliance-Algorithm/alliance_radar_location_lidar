# Alliance Radar Overall Architecture

## Overview

Alliance Radar 采用 **ROS2 component + 独立驱动进程 + YAML bringup** 的分层架构。
以 LiDAR 为主链路（`radar_lidar` → `radar_fusion` → `/localization/pose`），
视觉观测（`radar_camera`）作为补充观测源，`radar_bridge` 把最终位姿写入共享内存供 GUI 读取，
`radar_calibration` 负责离线相机-雷达标定。系统对外统一输出 `/localization/pose`。

- 主链路传感器：Odin 为主，Mid-70 通过 `sensor:=mid70` 切换。
- 进程拓扑：`radar_lidar` + `radar_fusion` 合并进单一 component 容器（intra-process 零拷贝）。

## Design Principles

- 雷达驱动（Odin / Livox / WS-30）保持独立进程，不并入算法容器。
- `radar_fusion` 独占 `/localization/pose` 出口。
- 离线模型转换由 `tools/model_to_map/` 负责，模型文件存放于 `model/`。
- `radar_bringup` 只负责 launch、参数、remap 和组件编排，不写业务 C++。
- `radar_camera` 作为补充观测源，不绑死 LiDAR 主链路。
- 命名统一收敛到 `radar::` 顶层命名空间（`radar::fusion::` / `radar::camera::` 等子命名空间）。
- **零自定义 ROS 消息**：只用标准消息（`geometry_msgs` / `diagnostic_msgs` / `vision_msgs`）；
  唯一的跨进程数据契约是 `radar_bridge` 的共享内存 struct。不设消息接口包。

## Package Architecture

```text
packages/
├── radar_lidar        ← LiDAR 预处理 + 配准定位
├── radar_fusion       ← 系统统一位姿出口 + 多目标跟踪
├── radar_camera       ← 视觉观测生成
├── radar_calibration  ← 离线相机-雷达标定 + 模型预处理
├── radar_bridge       ← ROS2 → 共享内存桥接
└── radar_bringup      ← launch / yaml / remap / compose
```

## Package Boundaries

### radar_lidar

订阅单雷达点云，完成有效点过滤、（可选）ROI 裁剪、球面网格下采样、多帧累积、
GICP 配准定位，并在配准结果基础上做动态点提取与欧氏聚类。只做 LiDAR 感知。

核心库 `radar_lidar_core`（纯算法、无 ROS 依赖）：

| 类 / 命名空间 | 文件 | 职责 |
|---|---|---|
| `radar::MapData` | `map_data.{hpp,cpp}` | PCD 加载，构建 small_gicp + PCL KdTree；静态工厂 `Load() -> std::expected<>` |
| `radar::LocalizationStage` | `localization.{hpp,cpp}` | GICP scan-to-map：ROI 裁剪 → 帧累积 → 球面网格 → 配准 |
| `radar::SphericalGrid` | `spherical_grid.{hpp,cpp}` | 方位角/俯仰角分箱下采样 |
| `radar::FrameAccumulator` | `frame_accumulator.{hpp,cpp}` | 滑动窗口多帧累积 |
| `radar::DynamicCloudStage` | `dynamic_cloud.{hpp,cpp}` | KdTree 最近邻动态障碍提取（OpenMP 并行） |
| `radar::ClusterStage` | `cluster.{hpp,cpp}` | 欧氏聚类 → 质心 + AABB |
| `radar::geom::` | `geometry_utils.hpp` | ROI 裁剪 / 位姿构造 / look-at / 有效点过滤 |
| `radar::config::` | `config.hpp` | `LocalizationConfig` / `RoiBounds` 参数结构 |
| `radar::types::` | `types.hpp` | `Point` / `Frame` / `PoseEstimate` 核心数据类型 |

ROS 组件 `radar::LidarPipeline`（`rclcpp_components`，进入 `radar_algorithm_container`）。
同包离线工具：`registration_tool`（CLI PCD-to-PCD 配准）、`offline_test_node`（Foxglove 可视化）。

### radar_fusion

系统唯一的 `/localization/pose` 出口。订阅 LiDAR 观测，做数据关联 + 多目标卡尔曼跟踪。
不做感知，不做 IO 桥接。

| 类 | 文件 | 职责 |
|---|---|---|
| `radar::fusion::FusionNode` | `fusion_node.{hpp,cpp}` | 订阅/发布 + 数据关联 + 轨迹生命周期管理 |
| `radar::fusion::KalmanTracker` | `kalman_tracker.{hpp,cpp}` | 单目标 2D 常速 KF：`[x,y,vx,vy]` 状态 |
| `radar::fusion::KalmanState` | `kalman_tracker.hpp` | KF 状态 + 轨迹元数据（color/number/lifecycle） |
| `radar::fusion::FusionConfig` | `fusion_node.hpp` | 门限/超时/确认命中数等参数 |

ROS 组件，与 `radar_lidar` 同容器零拷贝。

### radar_camera

补充观测源，独立于 LiDAR 主链路。订阅相机图像与内参，完成去畸变、目标检测 / 视觉定位，
输出视觉观测供 `radar_fusion` 融合。只产出观测，不做融合，不进入 LiDAR 主链路容器。

| 文件 | 职责 |
|---|---|
| `include/radar_camera/types.hpp` | `radar::camera::Detection` / `CameraFrame` 数据类型 |
| `include/radar_camera/config.hpp` | `radar::camera::CameraConfig`——内参/外参路径、检测阈值、topic 名 |
| `include/radar_camera/camera_model.hpp` | 去畸变 + 2D→3D 投影（纯函数，无 ROS） |
| `include/radar_camera/detector.hpp` | 目标检测接口 `detect(image) -> std::vector<Detection>` |
| `src/camera_node.{hpp,cpp}` | ROS 薄封装：订阅图像 → detector + camera_model → 发布观测 |
| `src/runtime.cpp` | `main()` → spin |

依赖 `radar_calibration` 输出的相机内参/外参 YAML。独立进程或独立 container。

### radar_calibration

手动触发的一次性离线标定工具。采集相机图像 + 雷达点云，计算相机-雷达外参，
生成 `radar_camera` 启动所需的 YAML 参数。非 spin 节点（`main()` 跑完退出），
不常驻主运行图，不参与实时定位链路。

| 文件 | 职责 |
|---|---|
| `include/radar_calibration/model_preprocess.hpp` | `radar::calibration::ModelProcess`（PIMPL）：FBX/PCD 模型加载 |
| `include/radar_calibration/camera_lidar_calibration.hpp` | 相机-雷达外参标定核心：`solve() -> std::expected<Extrinsic, string>` |
| `include/radar_calibration/pointcloud_capture.hpp` | 标定点云采集 |
| `include/radar_calibration/image_preprocess.hpp` | 标定图像预处理 |
| `src/running.cpp` | CLI 入口：读 `config/setting.yaml` → 标定 → 写外参 YAML |
| `config/setting.yaml` | 相机内参/外参、雷达外参、相机-雷达外参、模型/地图路径 |

### radar_bridge

订阅 `radar_fusion` 的输出，合并写入共享内存供非 ROS GUI（egui）零拷贝读取。
轻量 IO 桥接，不做任何算法。系统唯一的跨进程数据契约（shm struct）由本包定义。

| 文件 | 职责 |
|---|---|
| `include/radar_bridge/shm_layout.hpp` | 共享内存 struct（pose + state + fitness），egui 侧同定义 |
| `include/radar_bridge/shm_writer.hpp` | `radar::bridge::ShmWriter`：mmap + Seqlock 无锁写（无 ROS） |
| `src/bridge_node.{hpp,cpp}` | 订阅 pose + status → 转 struct → 调 writer |
| `src/runtime.cpp` | `main()` → spin |

独立轻量进程，不要求 component。

### radar_bringup

系统唯一编排入口。负责 launch、参数装配、topic remap、不同传感器机型配置、组件编排。
纯 YAML + launch，无业务 C++。

```text
config/
├── common/          ← 公共参数 (topic, frame, QoS, 滤波, 地图路径)
├── lidar/
│   ├── odin.yaml    ← Odin 专用参数（主链路）
│   └── mid70.yaml   ← Mid-70 专用参数
└── system/
    └── *.yaml       ← 组件组合配置

launch/
├── localization.launch.py       ← 单雷达定位入口 (sensor:=odin|mid70)
├── localization_gui.launch.py   ← + bridge + GUI 入口
└── full_system.launch.py        ← 完整系统入口
```

`ComposableNodeContainer` 承载 `radar_lidar` + `radar_fusion`（intra-process），
`IncludeLaunchDescription` 拉起独立驱动进程。

## Topic & Interface Contracts

```text
topics/
├── /odin1/cloud_raw  或  /livox/lidar
│   ├── type: sensor_msgs/msg/PointCloud2
│   ├── producer: 驱动进程 (odin_ros_driver / livox_ros_driver2)
│   └── consumer: radar_lidar
├── /lidar/pose
│   ├── type: geometry_msgs/msg/PoseWithCovarianceStamped
│   ├── producer: radar_lidar
│   ├── consumer: radar_fusion
│   └── role: LiDAR 主观测
├── /lidar/dynamic
│   ├── type: sensor_msgs/msg/PointCloud2
│   ├── producer: radar_lidar
│   └── role: 地图系动态点
├── /lidar/cluster
│   ├── type: sensor_msgs/msg/PointCloud2
│   ├── producer: radar_lidar
│   ├── consumer: radar_fusion
│   └── role: 聚类质心（数据关联输入）
├── /lidar/cluster_viz
│   ├── type: visualization_msgs/msg/MarkerArray
│   ├── producer: radar_lidar
│   └── role: AABB + 质心可视化
├── /diagnostics
│   ├── type: diagnostic_msgs/msg/DiagnosticStatus
│   ├── producer: radar_lidar
│   └── role: fitness / 耗时 / 帧号
├── /camera/image_raw, /camera/camera_info
│   ├── type: sensor_msgs/msg/Image, sensor_msgs/msg/CameraInfo
│   ├── producer: 相机驱动 (ros2-hikcamera)
│   └── consumer: radar_camera
├── /camera/pose  或  /camera/detection
│   ├── type: geometry_msgs/msg/PoseWithCovarianceStamped  或  vision_msgs/msg/Detection3DArray
│   ├── producer: radar_camera
│   └── consumer: radar_fusion
├── /localization/pose
│   ├── type: geometry_msgs/msg/PoseStamped
│   ├── producer: radar_fusion
│   ├── consumers: radar_bridge, 其他 ROS 消费者
│   └── role: 系统对外稳定位姿契约
├── /localization/status
│   ├── type: diagnostic_msgs/msg/DiagnosticStatus
│   ├── producer: radar_fusion
│   ├── consumers: radar_bridge, 其他 ROS 消费者
│   └── role: 定位状态 (state / fitness / converged)
└── /fusion/tracks
    ├── type: visualization_msgs/msg/MarkerArray
    ├── producer: radar_fusion
    └── role: confirmed 轨迹可视化

shared_memory/
└── /dev/shm/lidar_pose
    ├── type: shm_layout.hpp 中的 C++ struct (pose + state + fitness), 非 ROS 消息
    ├── producer: radar_bridge  (订阅 /localization/pose + /localization/status 后合并写入)
    ├── consumer: egui
    └── role: 系统唯一跨进程数据契约, Seqlock 无锁读写
```

## Data Flow

```text
驱动进程 (odin_ros_driver / livox_ros_driver2)
  └─ /odin1/cloud_raw | /livox/lidar : PointCloud2
       └─> radar_lidar (LidarPipeline)
             ├─ /lidar/pose      : PoseWithCovarianceStamped ─┐
             ├─ /lidar/cluster   : PointCloud2 (质心) ────────┤
             ├─ /lidar/dynamic   : PointCloud2                │
             ├─ /lidar/cluster_viz : MarkerArray              │
             └─ /diagnostics     : DiagnosticStatus           │
                                                              v
        /camera/pose|detection ────────────────> radar_fusion (FusionNode)
        (radar_camera)                             ├─ /localization/pose   : PoseStamped
                                                   ├─ /localization/status : DiagnosticStatus
                                                   └─ /fusion/tracks       : MarkerArray
                                                        │ pose + status
                                                        v
                                                   radar_bridge
                                                     └─ /dev/shm/lidar_pose (shm struct)
                                                          └─> egui (只读共享内存)
```

## Process Topology

```text
runtime/
├── 雷达驱动 (odin_ros_driver / livox_ros_driver2)   独立进程, 发布点云 topic
├── radar_algorithm_container                          component container
│   ├── radar_lidar   (component)
│   └── radar_fusion  (component)                       intra-process 零拷贝
├── radar_bridge                                        独立进程, /localization/* -> 共享内存
├── egui                                                非 ROS 进程, 只读共享内存
└── radar_camera                                        独立视觉观测进程
```

## Assumptions & Defaults

- 系统以 LiDAR 时间戳为主时钟。
- 标定精度（相机外参）为准，点云动态目标检测是辅助手段。
- `PoseStamped` 只作为系统对外轻量契约；融合内部默认使用 `PoseWithCovarianceStamped`。
- 主链路默认 Odin；Mid-70 通过 `sensor:=mid70` 切换。
