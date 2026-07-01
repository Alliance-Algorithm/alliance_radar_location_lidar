# RADAR-LOCATION-LIDAR

基于纯固态 / 非重复扫描 / 融合式激光雷达的定位方案。

支持 **WS_30PCD_ET3**（机智人科技）、**Livox**（大疆创新 / Mid-70 / HAP / MID360）、**Odin**（留形科技）三种传感器。

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
docker pull ghcr.io/harrypotter1tech/radar:develop
docker tag ghcr.io/harrypotter1tech/radar:develop radar:develop
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
| `build-all [Release\|Debug]` | 编译所有包（third-party + 全部 radar 包） |
| `build-radar [Release\|Debug]` | 编译 radar_calibration |
| `offline-registration <scan.pcd> --map <map.pcd>` | 启动离线配准可视化节点，发布 `/offline/*` 话题给 Foxglove |
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
offline-registration /path/to/scan.pcd --map /path/to/map.pcd
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

---

## 目录结构

```text
RADAR-LOCATION-LIDAR/
├── Dockerfile              # 基于 ros:jazzy 的开发环境镜像
├── .devcontainer/          # VSCode Dev Container 配置
│   ├── devcontainer.json
│   └── docker-compose.yml
├── .script/                # 开发脚本
│   ├── build-radar         # 编译标定包
│   ├── build-all           # 编译所有包
│   ├── run-radar           # 运行节点
│   ├── format-radar        # 格式化代码
│   ├── banner              # ASCII art banner
│   ├── ow_logo.txt         # Overwatch 风格 logo
│   └── template/           # 环境变量模板
├── ├── model/                 # 模型文件 (FBX, ONNX, PCD)                 # 离线资产
│   └── model/              # 模型文件 (FBX, ONNX, PCD)
├── ros_ws/                 # ROS2 工作空间
│   ├── src/
│   │   ├── radar_lidar/       # LiDAR 配准定位
│   │   ├── radar_camera/      # 视觉位姿观测
│   │   ├── radar_fusion/      # 多传感器融合定位
│   │   ├── radar_bridge/      # ROS2 → 共享内存桥接
│   │   ├── radar_calibration/ # 相机-雷达标定+定位
│   │   └── radar_bringup/     # Launch / YAML / 组件编排
│   └── third-party/        # small_gicp, hikcamera_sdk, direct_visual_lidar_calibration
├── lidar_ros_driver/       # LiDAR 驱动（git submodules）
│   └── * Livox 驱动使用 fork 版本以支持 Mid-70（上游 SDK2 暂未提供），维护者 @Yukikaze2233
├── docs/                   # SLAM 学习资料
└── README.md
```
