# livo2 一键建图 + 退出自动回环/后处理/彩色导出 Design

## Goal

把「建图 → 回环优化 → 后处理 → 彩色地图导出」收敛为**一条命令**：启动 `.script/livo2-build`
后自动录 bag + 启动建图 launch，Ctrl-C 退出时自动完成：

1. 保存几何地图（已有，launch 退出钩子）；
2. 保存在线彩色地图（LIO 位姿着色，新增 rgb_colorizer 析构存图）；
3. 停 bag 录制；
4. 回环优化（`optimize_pose_graph.py`）→ `map_optimized.pcd`；
5. 颜色迁移（新 `color_transfer.py`）→ 彩色 + 优化几何；
6. 后处理（`pcd_postprocess.py`，保留 rgb）→ 最终导出 `model/generated/livo2_final_rgb.pcd`。

最终地图 = **回环修正后的几何 + RGB 颜色**。

## Decisions

- **不录制图像**。bag 只含 `/fast_livo2/odom`、`/fast_livo2/cloud_world`、`/fast_livo2/path`
  （几百 MB）。不做「离线重着色」（B 方案）——实测 40m 走圈漂移 19mm、voxel 10cm，
  离线重着色相对颜色迁移的精度收益在视觉上不可见，而录图像 10 分钟产生 20-30 GB bag，
  代价不成比例。B 记为后续可选项（YAGNI）。
- 彩色来源 = `rgb_colorizer_node` 在线累积的彩色体素图（LIO 位姿投影，多视角
  `insert_if_better` 质量融合）。退出时节点析构保存，镜像 fast_livo2 的
  `~LivMapperNode() → save_pcd()` 模式（析构在 `rclcpp::shutdown` 后的正常上下文
  执行，非 signal handler，PCL 文件 I/O 安全）。
- 颜色迁移 = 网格哈希最近邻，纯 numpy，不引 scipy/KDTree（延续
  `tools/loop_closure`、`tools/pcd_postprocess` 的零依赖风格）。漂移 cm 级 << 阈值，
  最近邻迁移足够。
- 管线编排 = 新的 shell 脚本 `.script/livo2-build`，位于现有 `.script/*` 工具族中。
- 现有手动入口保留：`.script/lio-optimize record/run`、`.script/odin-map-export` 不动。

## 组件

### 1. `rgb_colorizer_node` 析构存图（小改动）

`~RgbColorizerNode()` 在现有 timer cancel / stop_camera_input 之后追加：

- 若彩色图非空：`ColorVoxelMap::save_binary_pcd(pcd_save_dir/cloud_rgb_shutdown_<sec>_<nsec>.pcd)`
- 日志输出路径与体素数
- 与 `pcd_trigger` 手动触发机制共存，互不影响

### 2. `tools/loop_closure/color_transfer.py`（新，纯 numpy）

```
用法: python3 color_transfer.py <colored.pcd> <geometry.pcd> --output out.pcd
      [--threshold 0.15] [--grid 0.05]
```

算法：

1. 读两个 PCD（ASCII/binary 都支持，复用 `pcd_postprocess` 的读取思路）；
   `<colored.pcd>` 提取 xyz+rgb；`<geometry.pcd>` 提取 xyz。
2. 彩色点建网格哈希索引（`grid` 边长，默认 0.05m，floor-divide + 哈希表）；
3. 对几何图每个点，取所在 3×3×3 邻域网格候选，找欧氏最近彩色点；
4. 距离 ≤ `threshold`（默认 0.15m）→ 复制 rgb；否则丢弃该点（保持与几何一致且
   无空洞色块——未着色区域不产出假色）；
5. 输出 `PointXYZRGB` binary PCD。

### 3. `.script/livo2-build`（新，bash）

```
用法: .script/livo2-build [--skip-record] [--no-postprocess]
```

流程：

1. `set -euo pipefail` + `set -m`（后台录包独立进程组）；`trap` 处理 INT：
   wrapper 自身忽略 SIGINT，让终端 Ctrl-C 只终止前台 launch 进程组；
2. 后台 `ros2 bag record -o model/generated/lio_bag_<ts> /fast_livo2/odom
   /fast_livo2/cloud_world /fast_livo2/path --max-cache-size 500000000`；
3. 前台 `ros2 launch radar_bringup odin_fast_livo2_mapping.launch.py`（默认
   `pcd_save_en:=true`，即上一步已改的 launch 默认值）；
4. launch 退出后：SIGINT 停录并等待 bag 收尾；
5. 按「数据流」跑管线，每步日志追加 `model/generated/optimization/pipeline.log`；
6. 打印汇总（各产物路径、点数、漂移报告）。

## 数据流

```
终端 Ctrl-C
 ├─ fast_livo2 析构 → lio_map.pcd（原始几何，已有机制）
 ├─ rgb_colorizer 析构 → cloud_rgb_shutdown_*.pcd（新）
 └─ livo2-build 接管:
    1. 停 bag
    2. optimize_pose_graph.py --bag <bag> --output-dir optimization/
       （--keyframe-dist 0.3 --loop-radius 2.0 --loop-min-skip 30，同 lio-optimize run）
       → optimization/map_optimized.pcd
    3. color_transfer.py cloud_rgb_shutdown_*.pcd map_optimized.pcd
       → optimization/map_optimized_rgb.pcd
    4. pcd_postprocess.py map_optimized_rgb.pcd --voxel-size 0.05
       → model/generated/livo2_final_rgb.pcd（最终导出）
    5. 汇总报告
```

## 错误处理

| 场景 | 行为 |
|---|---|
| bag 缺失/为空/录制失败 | 跳过回环与迁移，直出 `cloud_rgb_shutdown_*.pcd` 副本并标注 WARNING |
| 彩色图缺失（建图秒退等） | 跳过迁移，仅导出 `map_optimized.pcd`，红色告警提示原因 |
| launch 启动即失败（<60s 且退出码非 0） | 判定异常，跳过整条管线，提示查看 pipeline.log |
| 单步失败 | 不中断后续步骤；pipeline.log 记录退出码；汇总报告逐项 ✓/✗ |
| 已存在同名最终产物 | 覆盖前备份 `livo2_final_rgb.pcd.bak` |

## 测试

1. **color_transfer.py 单测**：合成两片已知颜色/位置的点云 → 断言颜色迁移正确、
   阈值剔除生效（超出 threshold 的点被丢弃）、不同 grid 下结果一致。
2. **rgb_colorizer 析构存图**：`colcon build --packages-select radar_fast_livo2_rgb`，
   短暂启动 launch 后 Ctrl-C，确认 `cloud_rgb_shutdown_*.pcd` 生成且点数 > 0。
3. **集成验证**：`pcd_postprocess.py` 对彩色 PCD 跑通（rgb 字段保留，已确认
   `voxel_downsample_with_extra` 支持 extra 字段）。
4. **端到端**：真实建图走一圈（或复用已有 `lio_bag_*` 回放数据），跑
   `.script/livo2-build`，人工检查 `livo2_final_rgb.pcd`（CloudCompare/Foxglove）：
   颜色与几何对齐、无空洞色块、回环漂移报告合理。

## 不做的事（YAGNI）

- 不录图像 topic；不做 B 方案离线重着色（`rgb_recolorize` 不在本期范围）；
- 不改 `optimize_pose_graph.py` / `extract_poses.py` / `pcd_postprocess.py` 核心逻辑；
- 不加 systemd/service 常驻，不碰 `lio-optimize` / `odin-map-export` 现有手动入口；
- 不做颜色插值/曝光补偿等质量增强（收益边际递减）。
