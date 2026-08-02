# mp4 视频回放 → camera 检测 → fusion 坐标连续性测试

日期: 2026-08-01

## 背景

需要验证 radar_camera（L1/L2/L3 检测 + 投影）输出的坐标经过 radar_fusion 融合后，
在 `/lidar/location` 上是否连续（轨迹无中断、无跳变、无闪烁）。

测试素材：`/home/yukikaze/Downloads/26-May-30-香港大学-2.mp4`（5472×3648 @25fps，698 秒，2.4GB）。

## 探索发现

1. **管线**：`radar_camera` 从 `/hikcamera_shm` 读帧 → L1/L2/L3 检测 → 投影
   （内参 + 外参 + mesh）→ 发布 `/radar_camera/robot_pose`
   （`radar_interfaces::msg::CameraDetectionPose`，6 类位置 + 置信度）→
   `radar_fusion` 订阅后跟踪融合 → 发布 `/lidar/location`
   （`LidarLocation`，24 字段 uint16 坐标）+ `/fusion/tracks`（MarkerArray）。

2. **fusion camera 路径独立**：`on_camera_detection()` 不依赖 lidar 输入
   （`update_fusion_mode` + `process_measurements` + publish），无 lidar 也能跑纯 camera 跟踪。

3. **内参已确认**：`ros_ws/src/radar_bringup/config/camera/radar_camera.yaml` 有
   5472×3648 全分辨率标定：
   ```
   camera_matrix: [6753.698616, 0.0, 2620.748274,
                   0.0, 6737.450110, 1924.062270,
                   0.0, 0.0, 1.0]
   distortion_coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]
   rotation: [0.0, 0.0, 0.0]     # 外参默认值（用户确认）
   translation: [0.0, 0.0, 0.0]
   mesh_path: "/workspace/model/generated/field_zup.obj"
   ```
   外参全 0（用户确认用默认值），投影出来的绝对坐标仅作相对连续性评估。

4. **颜色**：我方蓝方 → 敌方红方（检测 red 0-5 类；fusion `enemy_color: red`）。

5. **radar_camera 只从 SHM 读帧**，无视频输入；`SharedFrameWriter` 是 C++ API，
   Python 无绑定 → 回放器需用 C++ 实现（OpenCV 解码 + hikcamera SHM 写）。

6. 项目已有工具模式参考：`tools/video_zmq/zmq_mjpeg_player.py`（Python + ZMQ）、
   `tools/armor_verify/CMakeLists.txt`（独立 CMake 目标，引用 radar_camera 源码）。

## 目标

mp4 视频回放进 SHM → radar_camera 检测投影 → radar_fusion 融合 → 记录
`/lidar/location` 时间序列 → 分析输出坐标连续性。

## 架构

```
mp4 → mp4_replay (C++/OpenCV) → /hikcamera_shm
     → radar_camera (L1/L2/L3 + 投影) → /radar_camera/robot_pose
     → radar_fusion (跟踪融合, enable_camera_fusion) → /lidar/location + /fusion/tracks
     → location_recorder.py → CSV 时间序列
     → analyze → 连续性报告
```

## 组件与改动

| 组件 | 状态 | 内容 |
|---|---|---|
| `tools/video_zmq/mp4_replay.cpp`（新） | 新增 | C++ 回放器：OpenCV `VideoCapture` 解码 mp4 → 帧写入 `/hikcamera_shm`（v2 `SharedFrameWriter`，BGR8）→ 按视频帧率定时。参数：`--video <path>`、`--shm <name>`（默认 /hikcamera_shm）、`--speed <倍速>`（默认 1.0）、`--width/--height`（默认从视频取）、`--max-frames`（可选，测试用） |
| `tools/video_zmq/CMakeLists.txt`（新） | 新增 | 独立 CMake：链接 OpenCV + hikcamera SDK（参照 armor_verify 模式，RADAR_SRC 指向 /workspace/ros_ws/src/radar_camera，hikcamera 头/库从 install 取） |
| `tools/video_zmq/location_recorder.py`（新） | 新增 | 订阅 `/lidar/location`（LidarLocation）、`/radar_camera/robot_pose`（CameraDetectionPose）、`/fusion/tracks`（MarkerArray），按时间戳写 CSV（header.stamp、6 类 x/y、置信度） |
| `tools/video_zmq/analyze_continuity.py`（新） | 新增 | 读 CSV，输出：每类机器人轨迹中断次数/最长中断帧数、相邻帧跳变 > 阈值（默认 3m）的次数、跟踪创建→删除闪烁计数、每类轨迹时间线摘要（min/max/mean x/y、活跃帧比例） |
| `ros_ws/src/radar_bringup/config/camera/radar_camera.yaml` | 不改文件 | `enemy_color: "red"` 通过 `--ros-args -p enemy_color:=red` 覆盖（避免污染部署配置）；内参已在文件中 |
| `ros_ws/src/radar_fusion/config/*.yaml`（查） | 读取 | 测试时通过 `-p enable_camera_fusion:=true -p enemy_color:=red` 覆盖，不改部署文件 |
| 测试启动脚本 `tools/video_zmq/run_continuity_test.sh`（新） | 新增 | 一键：启动 radar_camera（推理 + 真实内参 + `-p enemy_color:=red`）→ radar_fusion（`-p enable_camera_fusion:=true` + `-p enemy_color:=red`）→ mp4_replay（可选 --max-frames）→ location_recorder → 停止 → 分析。 |

## 数据流与接口

### mp4_replay.cpp

```cpp
// 用法: mp4_replay --video <mp4> [--shm /hikcamera_shm] [--speed 1.0] [--max-frames N]
// 帧率: 由 VideoCapture 实际帧率决定（不 sleep 按 0 speed，1.0 按帧率）
// 写帧: hikcamera::SharedFrameWriter::create(name) → writer.write(meta, bgr_data)
//   FrameMetadata { committed_sequence, frame_id, host_monotonic_ns,
//                   width, height, stride_bytes=width*3, committed_bytes=width*height*3,
//                   pixel_format=BGR8 }
// 视频帧: cv::imdecode 由 VideoCapture 解码 → 确保 CV_8UC3 BGR → reshape 为 1D span
```

关键点：
- SHM 段创建用 `SharedFrameWriter::create`（O_EXCL），**必须先确保旧段不存在**
  （或启动前 `rm -f /dev/shm/hikcamera_shm`）；
- 写端必须维持 `committed_sequence` 单调递增（从 1 开始）；
- 帧率控制：`--speed 1.0` 时按 `1/fps` 间隔写；`--speed 0` 时尽快写（测速用）；
- 结束：写完后进程退出（writer 析构 unlink SHM）——**注意**：若 SHM 被 unlink，
  radar_camera 读侧会失败，测试流程需在 writer 退出前停止 recorder。

### location_recorder.py

```python
# 输出 CSV 列（每帧一行）:
# stamp_ns, hero_x, hero_y, hero_conf, eng_x, eng_y, eng_conf,
# inf3_x, inf3_y, inf3_conf, inf4_x, inf4_y, inf4_conf,
# sentry_x, sentry_y, sentry_conf, drone_x, drone_y, drone_conf,
# n_tracks（fusion MarkerArray marker 数）
# 订阅: /lidar/location (LidarLocation) — 24 字段 uint16 对手 6 类 x/y
#       /radar_camera/robot_pose (CameraDetectionPose) — 6 类 float x/y/z + conf
#       /fusion/tracks (MarkerArray)
# 参数: --out <csv> --duration <秒>（默认 60）
```

### analyze_continuity.py

```python
# 输入: location_recorder 输出的 CSV
# 输出:
#   per-class 摘要: 活跃帧数/总帧数, 首个/最后活跃帧, x/y min/max/mean/std
#   中断: 活跃帧间 gap > gap_threshold（默认 10 帧）计数 + 最大 gap
#   跳变: 相邻活跃帧位移 > jump_threshold（默认 3.0 m，lidar location 单位待确认）计数
#   闪烁: 活跃→消失→活跃 切换次数
# 终端打印 + 可选 --plot 保存 PNG 轨迹图（matplotlib）
```

## 连续性度量定义

| 指标 | 定义 | 默认阈值 |
|---|---|---|
| 中断 | 某类连续无有效输出帧数 | gap > 10 帧记一次中断 |
| 跳变 | 相邻有效帧坐标位移（欧氏） | > 3.0 m（uint16 单位待核实后换算） |
| 闪烁 | 活跃→消失→活跃 状态切换 | 计数（越低越好） |
| 活跃率 | 有有效坐标的帧 / 总录制帧 | 百分比 |

注意：`LidarLocation` 是 uint16 字段（与 radar_bridge zmq 格式一致，单位未注明，
可能是 cm 或 mm——分析脚本需先确认单位，报告里说明）。

## 错误处理

| 场景 | 行为 |
|---|---|
| 视频打不开 | mp4_replay 打印错误退出，返回非 0 |
| SHM 已存在（旧段残留） | mp4_replay 报错提示先删段（或自动 unlink——视权限） |
| radar_camera 读不到 SHM（writer 未启动） | radar_camera 启动时 shm_open 失败抛异常退出——测试脚本先启动 writer |
| recorder 订阅无消息 | recorder 打印 WARN 并继续；分析脚本对空文件友好退出 |

## 测试

1. **单元/冒烟**：mp4_replay 用 `--max-frames 10 --speed 0` 跑通，`ls /dev/shm/hikcamera_shm`
   存在，`hikcamera::SharedFrameReader`（test 工具或 radar_camera）能读到帧。
2. **链路测试**：短跑 60 秒（或 --max-frames 1500 = 60s@25fps）：
   启动 fusion → camera → replay → recorder → 停止 → 分析。
   预期：CSV 有数据，分析报告输出 6 类摘要；红方（0-5）应有活跃轨迹。
3. **连续性回归**：同一视频片段跑两次，报告指标一致（确定性验证）。

## 不做的事（YAGNI）

- 不做实时相机回放（本次只用视频）；
- 不修改 radar_camera / radar_fusion 的 C++ 逻辑（纯测试工具）；
- 不做 egui/ZMQ 推流接入；
- 不做多视频批量测试（脚本留 --video 参数即可扩展）。
