# tools — 离线预处理工具

这里存放**离线开发工具**：不参与 `colcon build`、不是 ROS2 运行时节点，
只在准备数据 / 生成资产 / 测试时手动运行。

> ROS2 运行时节点与会被 colcon 构建的工具在 `ros_ws/src/<pkg>/tools/`
> （例如 `radar_lidar/tools/offline_test_node`）。此处只放独立离线工具。

## model_to_map

FBX 场地模型 → PCD 点云地图（C++ / assimp / PCL）。表面均匀采样 + 可选体素降采样，
法向量按 `|nx|,|nz|,|ny|` 打包成 RGB。

```bash
tools/model_to_map/build/model_to_map <input.fbx> <output.pcd> [options]
#   --density <float>   每 m^2 采样点数 (默认 10000)
#   --scale <float>     坐标缩放 (cm→m 用 0.01, 默认 1.0)
#   --voxel <float>     VoxelGrid 叶大小, 0=不降采样 (默认 0)
#   --roi <6 floats>    x_min,x_max,y_min,y_max,z_min,z_max
#   --y-up              FBX 为 Y-up, 转成工作系 Z-up (x'=x, y'=-z, z'=y)
```

生成 RMUC 工作系规范地图（中心原点 / 米 / Z-up）：

```bash
tools/model_to_map/build/model_to_map model/RMUC2026_l.fbx \
    model/generated/map_zup.pcd --y-up --voxel 0.05
```

构建：`cmake -S tools/model_to_map -B tools/model_to_map/build && cmake --build tools/model_to_map/build`

## bag_to_pcd

ros2 bag 中的 PointCloud2 topic → PCD（纯 numpy，不依赖 open3d）。
主要用于把 Odin1 SLAM 模式的 `/odin1/cloud_slam` 导出为场地地图。

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/bag_to_pcd/bag_to_pcd.py <bag_dir> \
    [--topic /odin1/cloud_slam] [--output map.pcd] \
    [--per-frame --output-dir frames/]
```

## make_synth_scan

从规范 Z-up 地图合成一个**已知真值部署位姿**的 Odin scan，用于验证离线配准工具能否恢复
`T_map_lidar`。按 Odin1 真实规格建模：FOV 120°×90°、量程 70m、方位-俯仰 z-buffer 遮挡、
2cm 高斯噪声。真值位姿故意偏离标称初值，以检验 yaw 搜索 + 精配准的鲁棒性。

```bash
python3 tools/make_synth_scan/make_synth_scan.py \
    model/generated/map_zup.pcd model/generated/synth_scan_odin.pcd
```

生成后用 `offline-test <synth_scan.pcd> --map <map_zup.pcd>`（见项目根 README「离线配准可视化」）配准，比对输出 pose 与脚本打印的真值。

## logger

轻量级 header-only 日志工具（`Radar::Logger`），提供 `LOG_INFO/LOG_WARN/LOG_ERROR/LOG_DEBUG`
宏与编译期日志级别。当前为共享实现，尚未在各包中启用（各包暂用 ROS 原生 `RCLCPP_*`）。

```cpp
#include "logger/logger.hpp"

Radar::Logger::Logger::Init("/var/log/radar");
LOG_INFO("module_name", "message");
```
