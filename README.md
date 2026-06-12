# RADAR-LOCATION-LIDAR

基于纯固态 / 非重复扫描 / 融合式激光雷达的定位方案。

支持 **WS_30PCD_ET3**（机智人科技）、**Livox**（大疆创新 / Mid-70 / HAP / MID360）、**Odin**（留形科技）三种传感器。

---

## Docker 环境

### 构建镜像
```bash
git clone --recurse-submodules https://github.com/HarryPotter1tech/alliance_radar_location_lidar.git
cd alliance_radar_location_lidar
docker build -t radar:full .
```
> `docker build` 成功 = 所有 5 个包编译验证通过。
> 若网络受限，Dockerfile 内置了 ghproxy / tuna 国内镜像兜底。

### 创建容器
```bash
docker run -it -d --name RADAR --privileged --restart=always \
  -v /dev:/dev \
  -v $(pwd):/workspace \
  --network host \
  radar:full
```

### 命令解释

| 参数 | 说明 |
|------|------|
| `-it` | 保持 stdin 打开并分配 TTY，支持 `docker attach` 交互 |
| `-d` | 后台运行（detach mode） |
| `--name RADAR` | 容器命名为 `RADAR` |
| `--privileged` | 授予容器特权模式，允许直接访问宿主机硬件设备（LiDAR 等） |
| `--restart=always` | 容器退出后自动重启（含 Docker 守护进程重启时） |
| `-v /dev:/dev` | 挂载宿主机设备目录，使容器可直接访问 `/dev/ttyUSB*` 等串口与传感器 |
| `-v $(pwd):/workspace` | 挂载当前项目目录到容器 `/workspace`，代码改动即时生效 |
| `--network host` | 容器直接使用宿主机网络栈，无需端口映射，低延迟通信 |
| `radar:full` | 本仓库 Dockerfile 构建的镜像，预装 GCC 13 / CMake 4.3 / clang-22 / Livox SDK |

---

## Foxglove Bridge 配置

### 安装
```bash
sudo apt update && sudo apt install ros-$ROS_DISTRO-foxglove-bridge -y
```

### 启动
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

| 参数 | 说明 |
|------|------|
| `ros2 launch` | ROS2 启动工具，用于启动 launch 文件 |
| `foxglove_bridge` | Foxglove 桥接功能包名 |
| `foxglove_bridge_launch.xml` | 启动配置文件，加载桥接节点 |
| `port:=8765` | 指定 WebSocket 服务端口为 `8765`，Foxglove Studio 通过此端口连接 |

---

## LiDAR 驱动

项目集成了三种激光雷达的 ROS2 驱动，位于 `lidar_ros_driver/` 目录。

### 1. WS_30PCD_ET3 — 深圳市机智人科技

纯固态面阵激光雷达。

```bash
# 编译
cd /workspace/lidar_ros_driver/ws_30pcd_et3_ros2
colcon build --packages-select ws_30pcd_et3_ros2

# RVIZ 显示
ros2 run ws_30pcd_et3_ros2 main_node
ros2 run rviz2 rviz2

# 或通过 launch 启动
ros2 launch ws_30pcd_et3_ros2 scan_frame.launch.py
```

| 资源 | 地址 |
|------|------|
| 产品手册 | https://gitee.com/shenzhen-smart-people/product_manual.git |
| 结构图纸 | https://gitee.com/shenzhen-smart-people/structural_drawings.git |

---

### 2. Livox Mid-70 / HAP / MID360 — 大疆创新

非重复扫描固态激光雷达。已集成 `livox_ros_driver2`，支持 Mid-70 / HAP / MID360 系列。

```bash
# 编译
cd /workspace/lidar_ros_driver/livox_ros_driver2
./build.sh humble

# 启动（MID360 示例）
ros2 launch livox_ros_driver2 rviz_MID360_launch.py

# 启动（HAP 示例）
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `publish_freq` | `10.0` | 点云发布频率（Hz），最大 100.0 |
| `multi_topic` | `0` | 多雷达独立 topic：0=共用，1=独立 |
| `xfer_format` | `0` | 点云格式：0=Livox 自定义，1=定制格式，2=标准 PCL |

---

### 3. Odin — 留形科技

嵌入式视觉-激光雷达融合传感器，支持 Odometry / SLAM / 重定位三种模式。

```bash
# 编译
cd /workspace/lidar_ros_driver/odin_ros_driver
./script/build_ros2.sh

# 启动
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

| Topic | 说明 |
|-------|------|
| `odin1/cloud_raw` | 原始点云 |
| `odin1/cloud_render` | RGB 渲染点云 |
| `odin1/cloud_slam` | SLAM 点云 |
| `odin1/odometry` | 里程计 |
| `odin1/image` | RGB 图像（bgr8） |

| 模式 | `custom_map_mode` | 说明 |
|------|-------------------|------|
| Odometry | `0` | 纯里程计，map 与 odom 同姿 |
| SLAM | `1` | 完整 SLAM（含回环检测和地图保存） |
| 重定位 | `2` | 基于预建地图的重定位 |

> 使用前需配置 Udev 规则，详见 `lidar_ros_driver/odin_ros_driver/README.md`。

---

## 容器内开发

### 进入容器
```bash
docker exec -it RADAR bash
```

### 编译 & 运行

项目提供了一套脚本简化编译运行流程，位于 `/workspace/script/`，支持 `Release` / `Debug` 两种构建类型：

| 命令 | 说明 |
|------|------|
| `radar --build-pure [Release\|Debug]` | 清理后重新编译 |
| `radar --build-run [Release\|Debug]` | 增量编译并运行节点 |
| `radar --run` | 仅运行节点（不编译） |
| `radar --format` | 格式化 radar_localization_lidar/src/ 下所有 .cpp .hpp |
| `radar --help` | 查看帮助 |

构建类型默认为 `Release`：
- **Release**: `-O3 -march=native -flto -DNDEBUG -DEIGEN_NO_DEBUG`
- **Debug**: `-O3 -march=native -flto -DNDEBUG -DEIGEN_NO_DEBUG -g`（与 Release 优化完全一致，仅多调试符号）

也可以直接调用底层脚本：

```bash
# Release 纯净编译
bash /workspace/script/build_pure.sh Release

# Debug 编译 + 运行
bash /workspace/script/build_run.sh Debug
```

### clangd 配置

构建脚本已内置 `-DCMAKE_EXPORT_COMPILE_COMMANDS=1`，每次编译后会在 `/workspace/ros_ws/build/` 下生成 `compile_commands.json`，clangd 会自动读取。若需手动生成：

```bash
cd /workspace/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select radar_localization_lidar --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

> 项目根目录的 `.clangd` 提供 fallback 编译标志，覆盖 `lidar_ros_driver/` 下未构建的驱动包。

### 手动编译
```bash
cd /workspace/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select radar_localization_lidar
source /workspace/ros_ws/install/setup.bash
ros2 run radar_localization_lidar radar_localization_lidar_node
```

---

## 目录结构

```
RADAR-LOCATION-LIDAR/
├── ros_ws/               # ROS2 工作空间（含 radar_localization_lidar 及第三方依赖）
├── lidar_ros_driver/     # LiDAR 驱动（Livox / WS_30PCD_ET3 / Odin）
├── script/               # 编译运行脚本
├── docs/                 # SLAM 学习资料
└── README.md
```
