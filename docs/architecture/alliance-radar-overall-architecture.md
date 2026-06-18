# Alliance Radar Overall Architecture

## Project Tree

```text
alliance_radar_location_lidar/
│
├── .clang-format                              ← C++ 代码格式化规则
├── .clangd                                    ← Clangd LSP 配置
├── .coderabbit.yaml                           ← CodeRabbit 代码审查配置
├── .dockerignore                              ← Docker 构建忽略列表
├── .env                                       ← 环境变量
├── .gitmodules                                ← Git 子模块声明
├── Dockerfile                                 ← 容器镜像定义
├── README.md                                  ← 项目说明
│
├── .devcontainer/                             ← 开发容器配置
│   ├── devcontainer.json                      ← VS Code Dev Container 定义
│   ├── docker-compose.yml                     ← 多容器编排
│   └── post-create.sh                         ← 容器创建后初始化脚本
│
├── .github/
│   └── workflows/
│       ├── build.yml                          ← CI 构建流水线
│       └── build-image.yml                    ← Docker 镜像构建流水线
│
├── .script/                                   ← 开发脚本集
│   ├── banner                                 ← 终端 banner
│   ├── build-all                              ← 全量构建
│   ├── build-radar                            ← radar 包构建
│   ├── clean-radar                            ← radar 清理
│   ├── format-radar                           ← radar 代码格式化
│   ├── foxglove                               ← Foxglove 可视化启动
│   ├── gen_mvs_cmake.sh                       ← MVS CMake 生成
│   ├── ow_logo.txt                            ← Logo 文本
│   ├── run-radar                              ← radar 运行启动
│   └── template/
│       ├── env_setup.bash                     ← Bash 环境模板
│       └── env_setup.zsh                      ← Zsh 环境模板
│
├── .vscode/                                   ← VS Code 工作区配置
│
├── model/                                    ← 离线资产
│   └── models/
│       └── RMUC2026.fbx                       ← 赛场 3D 模型资产
│
├── docs/                                      ← 文档
│   ├── architecture/
│   │   └── alliance-radar-overall-architecture.md  ← 本文档
│   └── SLAM-learn/                            ← SLAM 学习资料
│       ├── 高翔_视觉SLAM十四讲-完整14讲版.pdf
│       └── slambook2-master/                  ← slambook2 代码示例
│           ├── ch2/ ~ ch13/                   ← 各章节实现
│           ├── coordinateTransform            ← 坐标变换示例
│           ├── eigenGeometry                  ← Eigen 几何模块示例
│           ├── linearEqSolution               ← 线性方程求解
│           ├── matrixExtractAssign            ← 矩阵操作示例
│           ├── useGeometry                    ← 几何库使用
│           ├── useSophus                      ← Sophus 李群库使用
│           ├── slambook2_笔记.docx
│           └── slambook2_习题解答.docx
│
├── lidar_ros_driver/                          ← 雷达驱动层（独立于算法）
│   ├── livox_ros_driver2/                     ← Livox Mid-70 ROS2 驱动
│   │   ├── CMakeLists.txt
│   │   ├── config/                            ← 驱动参数
│   │   ├── launch_ROS2/                       ← ROS2 launch 文件
│   │   ├── msg/                               ← 自定义消息
│   │   └── src/                               ← 驱动源码
│   │
│   ├── livox_sdk2/                            ← Livox SDK2 底层库
│   │   ├── CMakeLists.txt
│   │   ├── 3rdparty/                          ← 第三方依赖
│   │   ├── include/                           ← SDK 头文件
│   │   ├── samples/                           ← SDK 示例
│   │   └── sdk_core/                          ← SDK 核心实现
│   │
│   ├── odin_ros_driver/                       ← Odin 雷达驱动
│   │   ├── CMakeLists.txt
│   │   ├── config/
│   │   ├── launch_ROS2/
│   │   ├── src/
│   │   ├── lib/                               ← 预编译库
│   │   ├── RELOCALIZATION_GUIDE.md            ← 重定位指南
│   │   └── script/
│   │
│   └── ws_30pcd_et3_ros2/                     ← WS-30 雷达驱动
│       ├── CMakeLists.txt
│       ├── include/
│       ├── launch/
│       └── src/
│
└── ros_ws/                                    ← ROS2 工作空间
    ├── build/                                 ← 构建产物
    ├── install/                               ← 安装产物
    ├── log/                                   ← 运行日志
    │
    ├── src/
    │   └── radar_localization_lidar/          ← 当前核心算法单包
    │       ├── CMakeLists.txt
    │       ├── package.xml
    │       ├── config/
    │       │   ├── setting.yaml               ← 算法参数
    │       │   └── hikcamera_config.yaml      ← 海康相机参数
    │       ├── model/
    │       │   └── RMUC2026.fbx               ← 赛场 3D 模型资产
    │       └── src/
    │           ├── running.cpp                ← 主节点入口
    │           ├── pointcloud_process.cpp/hpp ← 点云处理管线
    │           ├── pointcloud_capture.cpp/hpp ← 点云采集
    │           ├── image_preprocess.cpp/hpp   ← 图像预处理
    │           ├── image_capturer.cpp         ← 图像采集
    │           ├── model_preprocess.cpp/hpp   ← 模型预处理
    │           └── camera_lidar_calibration.cpp ← 相机-雷达标定
    │
    ├── third-party/                           ← 第三方算法库
    │   ├── direct_visual_lidar_calibration/   ← 视觉-LiDAR 直接标定
    │   ├── hikcamera_sdk/                    ← 海康相机 ROS2 驱动
    │   └── small_gicp/                        ← 轻量 GICP 配准库
    │
    └── tool/
        └── logger/
            ├── logger.hpp                     ← 日志工具头文件
            └── logger_config.hpp              ← 日志配置
```

## Overview

Alliance Radar 采用 ROS2 component + 独立驱动进程 + YAML bringup 的分层架构。

核心目标是以 LiDAR 为主链路，先建立稳定的单雷达定位闭环，再逐步扩展视觉观测、共享内存桥接和离线标定能力。系统对外统一输出 `/localization/pose`，内部则保留传感器观测与融合观测的分层接口。

## Design Principles

- Livox 驱动保持独立进程，不并入算法容器。
- LiDAR 主链路优先稳定，作为系统基线。
- `radar_fusion` 从第一阶段开始独占 `/localization/pose`。
- 离线模型转换由 `tools/model_to_map/` 负责，模型文件存放于 `model/`。
- `radar_bringup` 只负责 launch、参数、remap 和组件编排，不写业务 C++。
- `radar_camera` 作为补充观测源，不绑死 LiDAR 主链路。

## Package Architecture

```text
packages/
├── radar_lidar        ← LiDAR 预处理 + 配准定位
├── radar_camera       ← 视觉观测生成
├── radar_fusion       ← 系统统一位姿出口
├── radar_bridge       ← ROS2 -> 共享内存桥接
├── radar_interfaces   ← 业务消息接口定义
├── radar_calibration  ← 手动触发的离线相机-雷达标定工具
└── radar_bringup      ← launch / yaml / remap / compose
```

### radar_lidar

- 职责：订阅 Mid-70 点云，完成缓冲、滤波、下采样、ROI、初始位姿估计与 GICP / NDT 配准。
- 输入：`/livox/lidar`。
- 输出：`/lidar/pose_raw`、`/lidar/pose`。
- 运行形态：ROS2 component，进入主算法 container。

### radar_camera

- 职责：图像订阅、去畸变、特征 / VO / 目标定位，输出视觉位姿观测。
- 输入：相机图像、相机参数、标定结果。
- 输出：`/camera/pose`。
- 运行形态：独立进程或独立 container，不进入 LiDAR 主链路。

### radar_fusion

- 职责：统一拥有 `/localization/pose`，Phase 1 先单输入透传，后续升级为多传感器融合。
- 输入：`/lidar/pose`，后续可加入 `/camera/pose`。
- 输出：`/localization/pose`。
- 运行形态：ROS2 component，进入主算法 container。

### radar_bridge

- 职责：订阅 `/localization/pose`，写入共享内存供 GUI 零拷贝读取。
- 输入：`/localization/pose`。
- 输出：`/dev/shm/lidar_pose`。
- 运行形态：独立轻量进程，不要求 component。

### radar_calibration

- 职责：手动触发一次性标定计算，生成相机启动所需 YAML 参数。
- 输出：标定 YAML。
- 运行形态：手动执行的工具包，不在主运行图中常驻。

### radar_interfaces

- 职责：定义跨模块通信所需的业务消息接口。
- 输入：无运行时输入。
- 输出：`LocalizationStatus.msg` 等自定义 ROS2 消息。
- 运行形态：纯接口定义包，不承载业务节点。

### radar_bringup

- 职责：launch、参数装配、topic remap、不同传感器机型配置。
- 内容：纯 YAML + launch 文件，无业务 C++。
- 运行形态：系统唯一编排入口。

## Process Topology

```text
runtime/
├── process_1: livox_ros_driver2
│   └── 负责 Mid-70 驱动与 /livox/lidar 发布
├── process_2: radar_algorithm_container
│   ├── radar_lidar
│   └── radar_fusion
├── process_3: radar_bridge
│   └── 负责 /localization/pose -> /dev/shm/lidar_pose
├── process_4: egui
│   └── 只读共享内存
└── process_5: radar_camera
    └── 独立视觉观测进程
```

1. `livox_ros_driver2`
   - 独立进程
   - 只负责 Mid-70 驱动和 ROS2 发布

2. `radar_algorithm_container`
   - ROS2 component container
   - Phase 1：`radar_lidar` + `radar_fusion`

3. `radar_bridge`
   - Phase 2 引入
   - 负责 ROS2 到共享内存桥接

4. `egui`
   - 非 ROS 进程
   - 只读 `/dev/shm/lidar_pose`

5. `radar_camera`
   - Phase 3 引入
   - 独立进程或独立 container

## Current Architecture Mapping

```text
Current Architecture
│
├── Driver Layer
│   └── lidar_ros_driver/
│       ├── livox_ros_driver2         ← Mid-70 点云输入
│       ├── odin_ros_driver           ← Odin 点云输入
│       └── ws_30pcd_et3_ros2         ← WS-30 点云输入
│
├── Core Algorithm Layer
│   └── ros_ws/src/radar_localization_lidar/
│       ├── running.cpp               ← 主节点入口
│       ├── pointcloud_process.*      ← 点云处理 / 配准 / 定位
│       ├── pointcloud_capture.*      ← 点云采集
│       ├── image_preprocess.*        ← 图像预处理
│       ├── image_capturer.cpp        ← 图像采集
│       ├── model_preprocess.*        ← FBX 模型处理
│       └── camera_lidar_calibration.cpp
│                                      ← 相机-雷达标定
│
└── Dependency Layer
    └── ros_ws/third-party/
        ├── small_gicp                ← GICP 配准
        ├── hikcamera_sdk             ← 海康相机驱动
        └── direct_visual_lidar_calibration
                                         ← 视觉-LiDAR 标定
```

## Target Project Tree

```text
alliance_radar_location_lidar/
│
├── lidar_ros_driver/                          ← 驱动层，独立于算法包
│   ├── livox_ros_driver2/
│   ├── livox_sdk2/
│   ├── odin_ros_driver/
│   └── ws_30pcd_et3_ros2/
│
├── model/                                    ← 离线资产
│   └── model/
│       ├── RMUC2026.fbx                       ← 赛场 3D 模型资产
│       └── generated/                         ← 离线生成的地图点云资产
│
├── tools/                                     ← 离线工具
│   ├── model_to_map/
│   └── calibration/
│
└── ros_ws/
    └── src/
        ├── radar_lidar/                       ← LiDAR 预处理 + 配准定位
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── include/
        │   ├── src/
        │   ├── config/
        │   └── README.md
        │
        ├── radar_camera/                      ← 视觉观测生成
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── include/
        │   ├── src/
        │   ├── config/
        │   └── README.md
        │
        ├── radar_fusion/                      ← 系统统一位姿出口
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── include/
        │   ├── src/
        │   ├── config/
        │   └── README.md
        │
        ├── radar_bridge/                      ← ROS2 -> 共享内存桥接
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── include/
        │   ├── src/
        │   ├── shm/
        │   └── README.md
        │
        ├── radar_interfaces/                  ← 业务消息接口定义
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── msg/
        │   └── README.md
        │
        ├── radar_calibration/                 ← 手动触发的离线标定工具
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── include/
        │   ├── src/
        │   ├── tools/
        │   └── README.md
        │
        └── radar_bringup/                     ← launch / yaml / remap / compose
            ├── CMakeLists.txt
            ├── package.xml
            ├── launch/
            │   ├── mid70_localization.launch.py
            │   ├── mid70_localization_gui.launch.py
            │   └── full_system.launch.py
            ├── config/
            │   ├── common/
            │   ├── lidar/
            │   │   └── mid70.yaml
            │   └── system/
            │       ├── phase1.yaml
            │       ├── phase2.yaml
            │       └── phase3.yaml
            └── README.md
```

## Target Architecture Mapping

```text
Target Architecture
│
├── Driver Layer
│   ├── lidar_ros_driver/
│   │   ├── livox_ros_driver2         ← 发布 /livox/lidar
│   │   ├── odin_ros_driver           ← 发布对应雷达点云 topic
│   │   └── ws_30pcd_et3_ros2         ← 发布对应雷达点云 topic
│   └── ros2-hikcamera                ← 发布 /camera/image_raw, /camera/camera_info
│
├── Asset / Tool Layer
│   ├── model/model                  ← 模型 / 赛场资产
│   ├── tools/model_to_map            ← 模型 / 赛场资产 -> 地图点云资产
│   └── radar_calibration             ← 手动标定数据 -> 外参 YAML
│
├── Perception Layer
│   ├── radar_lidar                   ← /livox/lidar -> /lidar/pose_raw, /lidar/pose
│   └── radar_camera                  ← /camera/image_raw, /camera/camera_info -> /camera/pose
│
├── Fusion / Localization Layer
│   └── radar_fusion                  ← /lidar/pose + /camera/pose
│                                      -> /localization/pose, /localization/status
│
├── Interface Layer
│   └── radar_bridge                  ← /localization/pose
│                                      -> /dev/shm/lidar_pose
│
├── Interface Definition Layer
│   └── radar_interfaces              ← LocalizationStatus.msg 等业务接口
│
└── Composition Layer
    └── radar_bringup                 ← launch / config / remap / phase selection
```

## Topic and Interface Contracts

```text
topics/
├── /livox/lidar
│   ├── type: sensor_msgs/msg/PointCloud2
│   ├── producer: livox_ros_driver2
│   └── consumer: radar_lidar
├── /camera/image_raw
│   ├── type: sensor_msgs/msg/Image
│   ├── producer: ros2-hikcamera
│   └── consumer: radar_camera
├── /camera/camera_info
│   ├── type: sensor_msgs/msg/CameraInfo
│   ├── producer: ros2-hikcamera
│   └── consumer: radar_camera
├── /lidar/pose_raw
│   ├── type: geometry_msgs/msg/PoseWithCovarianceStamped
│   ├── producer: radar_lidar
│   ├── consumer: debug / diagnostics
│   └── role: 未融合的 LiDAR 原始估计与协方差
├── /lidar/pose
│   ├── type: geometry_msgs/msg/PoseWithCovarianceStamped
│   ├── producer: radar_lidar
│   ├── consumer: radar_fusion
│   └── role: LiDAR 主观测
├── /camera/pose
│   ├── type: geometry_msgs/msg/PoseWithCovarianceStamped
│   ├── producer: radar_camera
│   ├── consumer: radar_fusion
│   └── role: 视觉位姿观测
├── /localization/pose
│   ├── type: geometry_msgs/msg/PoseStamped
│   ├── producer: radar_fusion
│   ├── consumers: radar_bridge, other ROS consumers
│   └── role: 系统对外稳定位姿契约
└── /localization/status
    ├── type: radar_interfaces/msg/LocalizationStatus
    ├── producer: radar_fusion
    ├── consumers: radar_bridge, other ROS consumers
    └── role: 系统定位状态输出

shared_memory/
└── /dev/shm/lidar_pose
    ├── producer: radar_bridge
    ├── consumer: egui
    └── role: GUI 读取的 Seqlock 共享内存接口
```

## Layered ROS Topic Flow

```text
ros_graph/
├── Driver Layer
│   ├── livox_ros_driver2
│   │   └── /livox/lidar : sensor_msgs/msg/PointCloud2 -> radar_lidar
│   └── ros2-hikcamera
│       ├── /camera/image_raw : sensor_msgs/msg/Image -> radar_camera
│       └── /camera/camera_info : sensor_msgs/msg/CameraInfo -> radar_camera
├── Perception Layer
│   ├── radar_lidar
│   │   ├── /lidar/pose_raw : geometry_msgs/msg/PoseWithCovarianceStamped -> debug / diagnostics
│   │   └── /lidar/pose : geometry_msgs/msg/PoseWithCovarianceStamped -> radar_fusion
│   └── radar_camera
│       └── /camera/pose : geometry_msgs/msg/PoseWithCovarianceStamped -> radar_fusion
├── Fusion / Localization Layer
│   └── radar_fusion
│       ├── /localization/pose : geometry_msgs/msg/PoseStamped -> radar_bridge
│       ├── /localization/pose : geometry_msgs/msg/PoseStamped -> other ROS consumers
│       ├── /localization/status : radar_interfaces/msg/LocalizationStatus -> radar_bridge
│       └── /localization/status : radar_interfaces/msg/LocalizationStatus -> other ROS consumers
└── Interface Layer
    └── radar_bridge
        └── /dev/shm/lidar_pose : Seqlock shared memory -> egui
```

## Phase 1 Minimal Loop

```text
phase1/
├── livox_ros_driver2
│   └── /livox/lidar : sensor_msgs/msg/PointCloud2 -> radar_lidar
├── radar_lidar
│   └── /lidar/pose : geometry_msgs/msg/PoseWithCovarianceStamped -> radar_fusion
├── radar_fusion
│   ├── /localization/pose : geometry_msgs/msg/PoseStamped -> radar_bridge
│   ├── /localization/pose : geometry_msgs/msg/PoseStamped -> other ROS consumers
│   ├── /localization/status : radar_interfaces/msg/LocalizationStatus -> radar_bridge
│   └── /localization/status : radar_interfaces/msg/LocalizationStatus -> other ROS consumers
└── radar_bridge
    └── /dev/shm/lidar_pose : Seqlock shared memory -> egui
```

## Bringup Design

```text
config/
├── common/          ← 公共参数 (topic, frame, QoS, 滤波, 地图路径)
├── lidar/
│   └── mid70.yaml   ← Mid-70 专用参数
└── system/
    ├── phase1.yaml  ← 仅: driver + lidar + fusion
    ├── phase2.yaml  ← phase1 + bridge
    └── phase3.yaml  ← phase2 + camera + calibration

launch/
├── mid70_localization.launch.py      ← Mid-70 单雷达定位入口
├── mid70_localization_gui.launch.py  ← Mid-70 + bridge + GUI 入口
└── full_system.launch.py             ← 完整系统入口
```

- `config/common/*.yaml`
  - 公共 topic、frame、QoS、滤波参数、地图路径

- `config/lidar/mid70.yaml`
  - Mid-70 专用参数
  - 驱动 topic、frame、点云滤波、配准参数

- `config/system/phase1.yaml`
  - 仅启 `livox_ros_driver2`、`radar_lidar`、`radar_fusion`

- `config/system/phase2.yaml`
  - 在 Phase 1 基础上加 `radar_bridge`

- `config/system/phase3.yaml`
  - 在 Phase 2 基础上加 `radar_camera`、`radar_calibration`

## Assumptions and Defaults

- 系统以 LiDAR 时间戳为主时钟。
- Phase 1 不做 ApproximateTimeSync。
- 相机不是主触发源，只做补充观测。
- `PoseStamped` 只作为系统对外轻量契约；融合内部默认使用 `PoseWithCovarianceStamped`。
- `radar_bridge` 与共享内存 ABI 暂不在 Phase 1 锁死，只预留位置。
