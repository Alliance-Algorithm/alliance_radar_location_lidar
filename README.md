# RADAR-LOCATION-LIDAR

基于纯固态 / 非重复扫描 / 融合式激光雷达的定位方案。

---

## 快速开始（Dev Container，推荐）

### 前置要求

- [Docker](https://docs.docker.com/get-docker/)
- [VSCode](https://code.visualstudio.com/) + [Dev Containers 扩展](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### 步骤

```bash
git clone --recurse-submodules https://github.com/HarryPotter1tech/alliance_radar_location_lidar.git
cd alliance_radar_location_lidar
code .
```

在 VSCode 中按 `Ctrl+Shift+P`，选择 **Dev Containers: Reopen in Container**。

容器会自动：
- 构建基于 `ros:jazzy` 的开发环境（GCC 14 + Clang 22 + CMake 4.2）
- 安装 Hik MVS SDK、Livox SDK2、GTSAM 4.2a9、Iridescence
- 挂载宿主机的 `~/.opencode`、`~/.codex` 配置
- 运行 `post-create.sh` 初始化子模块和编译第三方包
- 显示 ALLIANCE RADAR ASCII art banner

### 宿主机 opencode 配置复用

容器自动挂载以下宿主机路径（任何设备通用）：

| 宿主机路径 | 容器路径 | 说明 |
|------------|----------|------|
| `~/.opencode` | `/home/ubuntu/.opencode` | opencode 配置、skills、插件 |
| `~/.codex` | `/home/ubuntu/.codex` | codex 配置 |
| `~/.mimocode` | `/home/ubuntu/.mimocode` | mimocode 配置 |

> 不需要硬编码路径，任何设备的 opencode 配置都会自动同步到容器内。

---

## Docker 环境（手动）

### 方式一：拉取预构建镜像（推荐）

```bash
docker pull ghcr.io/alliance-algorithm/alliance_radar_location_lidar:develop
docker tag ghcr.io/alliance-algorithm/alliance_radar_location_lidar:develop radar:develop
```

### 方式二：本地构建镜像

```bash
git submodule update --init --recursive
docker build -t radar:develop .
```

### 创建容器

```bash
docker run -it -d --name RADAR --privileged --restart=always \
  -v /dev:/dev \
  -v $(pwd):/workspace \
  -v $HOME/.opencode:/home/ubuntu/.opencode:cached \
  -v $HOME/.codex:/home/ubuntu/.codex:cached \
  --network host \
  radar:develop
```

| 参数 | 说明 |
|------|------|
| `--privileged` | 授予容器特权模式，允许直接访问 LiDAR 等硬件设备 |
| `-v /dev:/dev` | 挂载宿主机设备目录 |
| `-v $(pwd):/workspace` | 挂载项目目录到容器 `/workspace` |
| `-v $HOME/.opencode:...` | 挂载宿主机 opencode 配置 |
| `--network host` | 容器直接使用宿主机网络栈 |

### 进入容器

```bash
docker exec -it RADAR zsh
```

---

## 构建 & 运行

### 快捷脚本

| 命令 | 说明 |
|------|------|
| `build-all [Release\|Debug]` | 编译所有包（third-party + 全部 radar 包，含 `direct_visual_lidar_calibration`） |
| `build-radar [Release\|Debug]` | 快速编译 radar 包 + 雷达驱动（跳过 `direct_visual_lidar_calibration`） |
| `calibrate-camera <image.jpg> --camera_model ... --camera_intrinsics ... --camera_distortion_coeffs ...` | 相机-雷达外参自动标定（需先 `build-all`），见下方「相机外参标定」 |
| `offline-test <scan.pcd> --map <map.pcd>` | 启动离线配准可视化节点，发布 `/offline/*` 话题给 Foxglove |
| `run-radar` | 运行标定定位节点 |
| `format-radar` | 格式化 C++ 源文件 |
| `banner` | 显示 ALLIANCE RADAR ASCII art |

### clangd 配置

- 容器内 VSCode clangd 使用 `/workspace/ros_ws/build/radar_lidar/compile_commands.json`
- 宿主机 Neovim clangd 先运行 `export-compile-commands`，在项目根生成 `compile_commands.json`
- `export-compile-commands` 会把编译数据库里的 `/workspace/...` 路径自动改写成宿主机项目路径，供宿主机 clangd 直接读取

---

## Foxglove Bridge

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### 离线配准可视化

```bash
offline-test /path/to/scan.pcd --map /path/to/map.pcd
```

或直接使用 launch：

```bash
ros2 launch radar_bringup offline_registration.launch.py \
  map_path:=/path/to/map.pcd \
  scan_path:=/path/to/scan.pcd
```

Foxglove 里主要查看这些话题：

- `/offline/map`
- `/offline/scan_raw`
- `/offline/scan_aligned`
- `/offline/overlay`
- `/offline/pose`

---

## 相机外参标定

一次性离线标定，用已有场地地图点云 + 一张相机图像自动算出相机外参
`t_map_camera`，全程无人工介入。依赖 `direct_visual_lidar_calibration`
（`ros_ws/third-party/`），需先跑 `build-all`（`build-radar` 会跳过它）。

```bash
calibrate-camera /path/to/photo.jpg \
  --camera_model plumb_bob \
  --camera_intrinsics fx,fy,cx,cy \
  --camera_distortion_coeffs k1,k2,p1,p2,k3
```

内部流程：`preprocess_map`（地图 + 图像 → `calib.json`）→ 注入
`ros_ws/src/radar_calibration/config/initial_guess.yaml` 里的粗略外参猜测
（雷达站相机安装几何估算值）→ `calibrate`（NID 直接配准精化）。结果在
`ros_ws/src/radar_calibration/calibration_data/calib.json` 的
`results.T_lidar_camera`（第三方库字段名，本项目内部读作 `t_map_camera`）。
用 `radar_calibration_node extract-result` 导出成 `radar_camera` 可读的 YAML：

```bash
ros2 run radar_calibration radar_calibration_node extract-result \
  ros_ws/src/radar_calibration/calibration_data/calib.json \
  ros_ws/src/radar_camera/config/extrinsic.yaml
```

---

## LiDAR 驱动

### 1. WS_30PCD_ET3 — 机智人科技

```bash
cd /workspace/lidar_ros_driver/ws_30pcd_et3_ros2
colcon build --packages-select ws_30pcd_et3_ros2
ros2 launch ws_30pcd_et3_ros2 scan_frame.launch.py
```

### 2. Livox Mid-70 / HAP / MID360 — 大疆创新

```bash
cd /workspace/lidar_ros_driver/livox_ros_driver2
cp package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/
./build.sh humble
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `publish_freq` | `10.0` | 点云发布频率（Hz），最大 100.0 |
| `multi_topic` | `0` | 多雷达独立 topic：0=共用，1=独立 |
| `xfer_format` | `0` | 点云格式：0=Livox 自定义，1=定制格式，2=标准 PCL |

> **注意**：官方 `build.sh` 不支持 `jazzy` 参数，需使用 `humble`。构建前需先复制 ROS2 配置文件。

### 3. Odin — 留形科技

> **注意**：Odin 驱动为 git submodule，需先初始化。初始化后参考子模块内 README 进行构建。

```bash
cd /workspace/lidar_ros_driver/odin_ros_driver
# 查看子模块 README 获取具体构建命令
cat README.md
```

#### Odin1 内置重定位（可选）

`radar_lidar` 默认用自身 GICP 做 scan-to-map 配准。若已有 Odin1 SLAM 建的
`.bin` 地图，可切到 Odin1 内置重定位为主位姿源，GICP 自动降级为重定位未收敛
时的回退，不需要手动切换。

```bash
# 1. 赛前离线走图一次，产出 .bin 地图
ros2 launch radar_bringup odin_slam_mapping.launch.py
# 走图完成后，另开终端执行 set_param.sh save_map 1（见子模块 README）

# 2. 用保存的地图做重定位 + GICP 回退联合定位
ros2 launch radar_bringup odin_relocalization_localization.launch.py \
  map_path:=/workspace/model/generated/map_zup.pcd \
  relocalization_map_path:=/path/to/saved_map.bin
```

`custom_init_pos`（红/蓝方安装位置估算值）需在
`ros_ws/src/radar_bringup/config/lidar/odin_driver_relocalization.yaml`
中按实际部署位置填入，默认全零仅供占位。

---

## 目录结构

```text
RADAR-LOCATION-LIDAR/
├── Dockerfile              # 基于 ros:jazzy 的开发环境镜像
├── .devcontainer/          # VSCode Dev Container 配置
│   ├── devcontainer.json
│   └── docker-compose.yml
├── .script/                # 开发脚本
│   ├── build-radar         # 快速编译 radar 包（跳过 DVLC）
│   ├── build-all           # 编译所有包（含 DVLC）
│   ├── calibrate-camera    # 相机-雷达外参自动标定
│   ├── run-radar           # 运行节点
│   ├── format-radar        # 格式化代码
│   ├── banner              # ASCII art banner
│   ├── ow_logo.txt         # Overwatch 风格 logo
│   └── template/           # 环境变量模板
├── model/                  # 模型文件 (FBX, ONNX, PCD)               # 离线资产
├── ros_ws/                 # ROS2 工作空间
│   ├── src/
│   │   ├── radar_lidar/       # LiDAR 配准定位
│   │   ├── radar_camera/      # 视觉位姿观测
│   │   ├── radar_fusion/      # 多传感器融合定位
│   │   ├── radar_bridge/      # ROS2 ↔ ZMQ 桥接 + SHM 视频推流
│   │   ├── radar_calibration/ # 相机-雷达标定+定位
│   │   └── radar_bringup/     # Launch / YAML / 组件编排
│   └── third-party/        # small_gicp, hikcamera_sdk（含 SHM）, direct_visual_lidar_calibration
├── lidar_ros_driver/       # LiDAR 驱动（git submodules）
│   └── * Livox 驱动使用 fork 版本以支持 Mid-70（上游 SDK2 暂未提供），维护者 @yukikaze223344
├── docs/                   # SLAM 学习资料
└── README.md
```
