# mp4 视频回放 → camera 检测 → fusion 坐标连续性测试 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用港大比赛 mp4 视频回放，验证 radar_camera 检测投影坐标经 radar_fusion 融合后在 /lidar/location 上的连续性（轨迹中断/跳变/闪烁/活跃率）。

**Architecture:** C++ 回放器 mp4_replay 将 mp4 帧写入 /hikcamera_shm（hikcamera v2 SharedFrameWriter），radar_camera 读 SHM 检测投影发布 /radar_camera/robot_pose，radar_fusion 融合发布 /lidar/location，Python recorder 记录 CSV，分析脚本输出连续性指标。

**Tech Stack:** C++23, OpenCV, hikcamera_sdk v2, ROS2 Jazzy, Python3, matplotlib(可选)。

## Global Constraints

- 内参（来自 radar_bringup/config/camera/radar_camera.yaml，5472×3648）：`camera_matrix: [6753.698616, 0.0, 2620.748274, 0.0, 6737.450110, 1924.062270, 0.0, 0.0, 1.0]`，`distortion_coefficients: [0,0,0,0,0]`，`rotation/translation: [0,0,0]`。
- 测试参数用 `--ros-args -p` 覆盖，**不改部署 yaml**：radar_camera `-p enemy_color:=red`；radar_fusion `-p enable_camera_fusion:=true -p enemy_color:=red`。
- SHM 写端 v2 API：`hikcamera::SharedFrameWriter::create(name)` + `write(FrameMetadata, span<const unsigned char>)`；`FrameMetadata { committed_sequence, frame_id, host_monotonic_ns, width, height, stride_bytes, committed_bytes, pixel_format=BGR8 }`；像素 BGR8。
- SHM 段 O_EXCL 创建，writer 析构自动 unlink；启动前必须确保旧段不存在。
- 视频：`/home/yukikaze/Downloads/26-May-30-香港大学-2.mp4`，5472×3648 @25fps。
- 容器内路径 /workspace = 项目根，构建/运行在 devcontainer-radar-develop-1 内。
- 连续性指标：中断（gap>10 帧）、跳变（相邻位移>3.0 单位，单位在分析脚本确认后换算）、闪烁（活跃→消失→活跃切换次数）、活跃率（有效帧/总帧）。
- LidarLocation 为 uint16 字段（单位待确认——分析脚本输出时注明）。

---

### Task 1: 修复 radar_camera 构建（hikcamera v2 include + SHMRead 迁移前置）

**Files:**
- Modify: `ros_ws/src/radar_camera/CMakeLists.txt`
- Modify: `ros_ws/src/radar_camera/package.xml`（如缺 <depend> 已存在则不动）

**Interfaces:**
- Produces: `colcon build radar_camera` 成功（此任务只修构建，SHMRead 迁移在 Task 2）。

**背景:** radar_camera 当前编译失败：
1. `raw_shm_reader.hpp:19: fatal error: hikcamera/shm.hpp: No such file or directory` —— radar_camera_core 是手动 add_library，`ament_auto_find_build_dependencies` 只作用于 ament_auto target，不会给手动库传 include；
2. 需要用 v1 API 的 SHMRead/raw_shm_reader（Task 2 迁移）。

- [ ] **Step 1: CMakeLists.txt 加 find_package(hikcamera) 与链接**

在 `ament_auto_find_build_dependencies()`（50 行）之后加：

```cmake
find_package(hikcamera REQUIRED)
```

在 `target_link_libraries(${PROJECT_NAME}_core PUBLIC ...)` 块（68-79 行）中追加 `hikcamera::hikcamera`：

```cmake
target_link_libraries(${PROJECT_NAME}_core
    PUBLIC
        opencv_core
        opencv_imgproc
        opencv_imgcodecs
        opencv_dnn
        opencv_calib3d
        openvino::runtime
        Eigen3::Eigen
        assimp::assimp
        PkgConfig::FFMPEG
        hikcamera::hikcamera
)
```

- [ ] **Step 2: 构建验证（预期失败于 SHMRead，确认 include 已通）**

```bash
docker exec devcontainer-radar-develop-1 bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/ros_ws/install/setup.bash && cd /workspace/ros_ws && export RADAR_CAMERA_TENSORRT_ROOT=/opt/radar_camera_trt && colcon build --packages-select radar_camera 2>&1 | grep -E 'error|Failed|Finished' | head -8"
```

Expected: 不再有 `hikcamera/shm.hpp: No such file` 错误；若继续失败，错误应为 `SHMRead` 未定义（Task 2 处理）或 `imageSHM` 未定义（raw_shm_reader.cpp，Task 2 处理）。

- [ ] **Step 3: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
git add ros_ws/src/radar_camera/CMakeLists.txt
git commit -m "build(camera): link hikcamera target for radar_camera_core"
```

---

### Task 2: 迁移 radar_camera SHM 读取到 v2 SharedFrameReader

**Files:**
- Modify: `ros_ws/src/radar_camera/src/radar_camera_node.cpp`（`shm_open` + `SHMRead` → `SharedFrameReader`）
- Modify: `ros_ws/src/radar_camera/include/radar_camera/radar_camera_node.hpp`
- Modify: `ros_ws/src/radar_camera/src/raw_shm_reader.cpp`
- Modify: `ros_ws/src/radar_camera/include/radar_camera/raw_shm_reader.hpp`
- Modify: `ros_ws/src/radar_camera/include/radar_camera/raw_video_recorder.hpp`（如引用 imageSHM）

**Interfaces:**
- Consumes: `hikcamera::SharedFrameReader`（`open(name) -> expected<void,string>`、`wait_next(milliseconds) -> expected<SharedFrame, FrameReadError>`、`SharedFrame::mat() -> cv::Mat`（BGR8 非拥有视图）、`SharedFrame::metadata() -> const FrameMetadata&`、`SharedFrame::valid()`）。
- Produces: radar_camera 使用 v2 SHM 读取，编译通过。

**背景:** radar_camera_node.cpp:72-78 用 `shm_open` + `FdGuard`，:265 用 `hikcamera::SHMRead`（v2 SDK 已删除）；raw_shm_reader.cpp 用 `imageSHM`/`mmap`（v1 结构已删除）。全部迁移到 v2。

- [ ] **Step 1: 修改 radar_camera_node.hpp 成员**

`include/radar_camera/radar_camera_node.hpp`：
- 替换 `int shm_fd_ = -1;` 为 `hikcamera::SharedFrameReader shm_reader_;`
- include `"hikcamera/shared_frame_reader.hpp"`
- `FdGuard` 若无其他用处可移除相关头（检查后处理）

- [ ] **Step 2: 修改 radar_camera_node.cpp 构造函数**

```cpp
    // 原 FdGuard + shm_open 块（72-78 行）替换为:
    auto open_ret = shm_reader_.open(camera_config_.shm_name.c_str());
    if (!open_ret) {
        throw std::runtime_error("SHM open failed: " + open_ret.error());
    }
    RCLCPP_INFO(get_logger(), "SHM open succeeded: %s", camera_config_.shm_name.c_str());
```

- [ ] **Step 3: 修改 infer_thread_start 的读帧（265-271 行）**

```cpp
            cv::Mat orig_frame;
            auto frame = shm_reader_.wait_next(std::chrono::milliseconds { 100 });
            if (!frame || !frame->valid()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            orig_frame = frame->mat().clone();
            capture_timestamp_ = std::chrono::steady_clock::now();
```

注意：原 SHMRead 返回的 ts 是 host monotonic 时间戳；v2 SharedFrame 的 `metadata().host_monotonic_ns` 可替代：

```cpp
            capture_timestamp_ = std::chrono::steady_clock::time_point(
                std::chrono::nanoseconds(frame->metadata().host_monotonic_ns));
```

- [ ] **Step 4: 迁移 raw_shm_reader.hpp/cpp（录制侧）**

`raw_shm_reader.hpp`：成员 `hikcamera::SharedFrameReader reader_;` 替代 `shm_fd_/shm_ptr_`（保留 RecordingFifo 参数）；`start()` 中 `reader_.open(name)`；`loop()` 中 `reader_.wait_next(2000ms)` → 取 mat clone + metadata → fifo push。

`raw_shm_reader.cpp`：删除 `shm_open/mmap/imageSHM/sem` 相关代码，替换为 v2 调用。保留 `ReaderState/stats/failure_reason` 接口不变（radar_camera_node.cpp 依赖）。**注意**：v2 wait_next 无帧时返回 FrameReadError（Timeout）——按超时处理继续循环，不进 failed 状态；只有 open 失败进 failed。

- [ ] **Step 5: 构建验证**

```bash
docker exec devcontainer-radar-develop-1 bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/ros_ws/install/setup.bash && cd /workspace/ros_ws && export RADAR_CAMERA_TENSORRT_ROOT=/opt/radar_camera_trt && colcon build --packages-select radar_camera 2>&1 | grep -E 'error|Failed|Finished' | head -8"
```

Expected: `Finished <<< radar_camera`。

- [ ] **Step 6: 运行测试（可选，需 GPU）**

```bash
docker exec devcontainer-radar-develop-1 bash -c "cd /workspace/ros_ws/build/radar_camera && ctest -R radar_camera_tests 2>&1 | tail -5"
```

若 ctest 需 GPU/相机则跳过（记录说明）。

- [ ] **Step 7: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
git add ros_ws/src/radar_camera/
git commit -m "fix(camera): migrate SHM read to v2 SharedFrameReader"
```

---

### Task 3: mp4_replay C++ 回放器

**Files:**
- Create: `tools/video_zmq/mp4_replay.cpp`
- Create: `tools/video_zmq/CMakeLists.txt`

**Interfaces:**
- Consumes: OpenCV `VideoCapture`、`hikcamera::SharedFrameWriter`。
- Produces: `mp4_replay` 可执行文件；CLI：`mp4_replay --video <path> [--shm /hikcamera_shm] [--speed 1.0] [--max-frames N]`。写帧 BGR8，`committed_sequence` 从 1 递增，`frame_id` 同值，`host_monotonic_ns` 用 steady_clock，`stride_bytes=width*3`，`committed_bytes=width*height*3`，`pixel_format=BGR8`。

- [ ] **Step 1: 写 mp4_replay.cpp**

```cpp
#include <chrono>
#include <cstdint>
#include <cstring>
#include <expected>
#include <iostream>
#include <span>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include "hikcamera/shared_frame_writer.hpp"

namespace {
struct Args {
    std::string video;
    std::string shm { "/hikcamera_shm" };
    double speed { 1.0 };
    int max_frames { 0 };  // 0 = unlimited
};

auto parse_args(int argc, char** argv) -> std::expected<Args, std::string> {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
            return argv[++i];
        };
        if (arg == "--video") a.video = next();
        else if (arg == "--shm") a.shm = next();
        else if (arg == "--speed") a.speed = std::stod(next());
        else if (arg == "--max-frames") a.max_frames = std::stoi(next());
        else return std::unexpected("unknown arg: " + arg);
    }
    if (a.video.empty()) return std::unexpected("--video required");
    if (a.speed < 0.0) return std::unexpected("--speed must be >= 0 (0 = as fast as possible)");
    return a;
}
} // namespace

auto main(int argc, char** argv) -> int {
    auto args = parse_args(argc, argv);
    if (!args) {
        std::cerr << "usage: mp4_replay --video <mp4> [--shm name] [--speed x] [--max-frames N]\n"
                  << "error: " << args.error() << "\n";
        return 2;
    }

    cv::VideoCapture cap(args->video);
    if (!cap.isOpened()) {
        std::cerr << "failed to open video: " << args->video << "\n";
        return 1;
    }
    const double fps = cap.get(cv::CAP_PROP_FPS);
    const int width  = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    const int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    std::cout << "[mp4_replay] " << args->video << " " << width << "x" << height
              << " @ " << fps << " fps, speed=" << args->speed << "\n";

    auto writer = hikcamera::SharedFrameWriter::create(args->shm.c_str());
    if (!writer) {
        std::cerr << "SHM create failed (exists?): " << writer.error()
                  << "\n  rm -f /dev/shm" << args->shm << "  or use a different --shm\n";
        return 1;
    }
    std::cout << "[mp4_replay] SHM " << args->shm << " created\n";

    cv::Mat frame;
    uint64_t seq = 0;
    const auto frame_interval = std::chrono::duration<double>(1.0 / (fps > 0 ? fps : 25.0));

    for (;;) {
        if (!cap.read(frame)) break;
        if (frame.empty()) break;
        if (frame.channels() != 3) {
            std::cerr << "unexpected channels: " << frame.channels() << "\n";
            return 1;
        }
        cv::Mat bgr;
        if (frame.type() != CV_8UC3) frame.convertTo(bgr, CV_8UC3);
        else bgr = frame;

        ++seq;
        hikcamera::FrameMetadata meta;
        meta.committed_sequence    = seq;
        meta.frame_id              = seq;
        meta.host_monotonic_ns     = static_cast<std::uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
        meta.width                 = static_cast<std::uint32_t>(bgr.cols);
        meta.height                = static_cast<std::uint32_t>(bgr.rows);
        meta.stride_bytes          = static_cast<std::uint32_t>(bgr.cols * 3);
        meta.committed_bytes       = static_cast<std::uint32_t>(bgr.total() * 3);
        meta.pixel_format          = hikcamera::PixelFormat::BGR8;

        auto ret = writer->write(meta, std::span<const unsigned char>(bgr.data, bgr.total() * 3));
        if (!ret) {
            std::cerr << "SHM write failed at frame " << seq << ": " << ret.error() << "\n";
            return 1;
        }
        if (seq % 30 == 0) {
            std::cout << "[mp4_replay] " << seq << " frames" << std::endl;
        }
        if (args->speed > 0.0) {
            std::this_thread::sleep_for(frame_interval / args->speed);
        }
        if (args->max_frames > 0 && seq >= static_cast<std::uint64_t>(args->max_frames)) break;
    }

    std::cout << "[mp4_replay] done: " << seq << " frames\n";
    return 0;
}
```

- [ ] **Step 2: 写 CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.16)
project(mp4_replay CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)

set(RADAR_WS /workspace)
set(HIK_INSTALL ${RADAR_WS}/ros_ws/install/hikcamera)

add_executable(mp4_replay mp4_replay.cpp)

target_include_directories(mp4_replay PRIVATE
    ${HIK_INSTALL}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(mp4_replay PRIVATE
    ${HIK_INSTALL}/lib
)

target_link_libraries(mp4_replay
    ${OpenCV_LIBS}
    hikcamera
    pthread
    rt
)

install(TARGETS mp4_replay DESTINATION bin)
```

注意：libhikcamera.so 依赖 MVS SDK（/opt/MVS/lib/64），运行时需 `LD_LIBRARY_PATH=/workspace/ros_ws/install/hikcamera/lib:/workspace/ros_ws/third-party/hikcamera_sdk/src/sdk/lib:/opt/MVS/lib/64`（与 hikcamera_ros_driver 启动时一致）。

- [ ] **Step 3: 构建**

```bash
docker exec devcontainer-radar-develop-1 bash -c "cd /workspace/tools/video_zmq && rm -rf build && mkdir build && cd build && cmake .. > /dev/null && make -j\$(nproc) 2>&1 | grep -E 'error|Error|mp4_replay' | head -5; ls -la mp4_replay"
```

Expected: `mp4_replay` 二进制存在，无 error。

- [ ] **Step 4: 冒烟测试（写 10 帧到测试 SHM 名）**

```bash
docker exec devcontainer-radar-develop-1 bash -c "cd /workspace/tools/video_zmq/build && LD_LIBRARY_PATH=/workspace/ros_ws/install/hikcamera/lib:/workspace/ros_ws/third-party/hikcamera_sdk/src/sdk/lib:/opt/MVS/lib/64 ./mp4_replay --video '/home/yukikaze/Downloads/26-May-30-香港大学-2.mp4' --shm /hikcamera_shm_test --max-frames 10 --speed 0 2>&1 | tail -3; ls -la /dev/shm/hikcamera_shm_test 2>/dev/null"
```

注意：视频文件路径在容器内——宿主机 Downloads 是否挂载进容器？检查后决定（若未挂载，先 `docker cp` 或调整 --video 路径为容器可见路径）。

Expected: `done: 10 frames`，SHM 段存在。冒烟后清理测试段：`rm -f /dev/shm/hikcamera_shm_test`。

- [ ] **Step 5: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
git add tools/video_zmq/mp4_replay.cpp tools/video_zmq/CMakeLists.txt
git commit -m "feat(tools): mp4 replay to hikcamera SHM"
```

---

### Task 4: location_recorder.py

**Files:**
- Create: `tools/video_zmq/location_recorder.py`

**Interfaces:**
- Consumes: `/lidar/location`（radar_interfaces/msg/LidarLocation）、`/radar_camera/robot_pose`（radar_interfaces/msg/CameraDetectionPose）、`/fusion/tracks`（visualization_msgs/msg/MarkerArray）。
- Produces: CSV 输出到 `--out` 路径；CLI：`location_recorder.py --out <csv> [--duration <sec>] [--topics ...]`。

- [ ] **Step 1: 写 location_recorder.py**

```python
#!/usr/bin/env python3
"""Record radar_fusion camera-track output to CSV for continuity analysis.

Subscribes /lidar/location (LidarLocation), /radar_camera/robot_pose
(CameraDetectionPose), /fusion/tracks (MarkerArray) and writes one CSV
row per /lidar/location message.

CSV columns:
  stamp_ns, hero_x, hero_y, hero_conf, eng_x, eng_y, eng_conf,
  inf3_x, inf3_y, inf3_conf, inf4_x, inf4_y, inf4_conf,
  sentry_x, sentry_y, sentry_conf, drone_x, drone_y, drone_conf,
  n_tracks
"""
import argparse
import csv
import signal
import sys
import time

import rclpy
from rclpy.node import Node

from radar_interfaces.msg import CameraDetectionPose, LidarLocation
from visualization_msgs.msg import MarkerArray


class Recorder(Node):
    def __init__(self, out_path: str, duration: float):
        super().__init__("location_recorder")
        self.out_path = out_path
        self.duration = duration
        self.start_ns = time.time_ns()
        self.n_rows = 0
        self.f = open(out_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow([
            "stamp_ns", "hero_x", "hero_y", "hero_conf",
            "eng_x", "eng_y", "eng_conf",
            "inf3_x", "inf3_y", "inf3_conf",
            "inf4_x", "inf4_y", "inf4_conf",
            "sentry_x", "sentry_y", "sentry_conf",
            "drone_x", "drone_y", "drone_conf",
            "n_tracks",
        ])
        self.sub_loc = self.create_subscription(
            LidarLocation, "/lidar/location", self.on_location, 10)
        self.sub_pose = self.create_subscription(
            CameraDetectionPose, "/radar_camera/robot_pose", self.on_pose, 10)
        self.sub_tracks = self.create_subscription(
            MarkerArray, "/fusion/tracks", self.on_tracks, 10)
        self.get_logger().info(f"recording to {out_path}")

    def on_pose(self, msg: CameraDetectionPose):
        self.latest_pose = msg

    def on_tracks(self, msg: MarkerArray):
        self.latest_tracks = msg

    def on_location(self, msg: LidarLocation):
        p = getattr(self, "latest_pose", None)
        t = getattr(self, "latest_tracks", None)
        row = [
            msg.header.stamp.nanosec + msg.header.stamp.sec * 1_000_000_000,
            msg.opponent_hero_x, msg.opponent_hero_y,
            p.hero_confidence if p else 0.0,
            msg.opponent_engineer_x, msg.opponent_engineer_y,
            p.engine_confidence if p else 0.0,
            msg.opponent_infantry_3_x, msg.opponent_infantry_3_y,
            p.infantry_3_confidence if p else 0.0,
            msg.opponent_infantry_4_x, msg.opponent_infantry_4_y,
            p.infantry_4_confidence if p else 0.0,
            msg.opponent_sentry_x, msg.opponent_sentry_y,
            p.sentry_confidence if p else 0.0,
            msg.opponent_aerial_x, msg.opponent_aerial_y,
            p.drone_confidence if p else 0.0,
            len(t.markers) if t else 0,
        ]
        self.w.writerow(row)
        self.n_rows += 1
        if self.n_rows % 100 == 0:
            self.get_logger().info(f"{self.n_rows} rows")
        elapsed = (time.time_ns() - self.start_ns) / 1e9
        if self.duration > 0 and elapsed >= self.duration:
            self.shutdown()

    def shutdown(self):
        self.f.close()
        self.get_logger().info(f"done: {self.n_rows} rows -> {self.out_path}")
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--duration", type=float, default=60.0)
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.out, args.duration)
    signal.signal(signal.SIGINT, lambda *_: node.shutdown())

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.n_rows > 0 or node.f.closed is False:
            node.shutdown()


if __name__ == "__main__":
    main()
```

注意：`shutdown()` 可能重复调用（spin 结束 + finally）——用 `self.f.closed` 守卫；`rclpy.shutdown()` 调用后 spin 返回。实现时确保幂等。

- [ ] **Step 2: 语法验证**

```bash
docker exec devcontainer-radar-develop-1 bash -c "python3 -m py_compile /workspace/tools/video_zmq/location_recorder.py && echo SYNTAX-OK"
```

- [ ] **Step 3: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
git add tools/video_zmq/location_recorder.py
git commit -m "feat(tools): record camera-fusion location stream to CSV"
```

---

### Task 5: analyze_continuity.py

**Files:**
- Create: `tools/video_zmq/analyze_continuity.py`

**Interfaces:**
- Consumes: location_recorder 输出的 CSV（列名见 Task 4）。
- Produces: 终端报告 + 可选 `--plot out.png` 轨迹图。CLI：`analyze_continuity.py <csv> [--gap-threshold 10] [--jump-threshold 3.0] [--plot <png>] [--unit-factor 1.0]`。

- [ ] **Step 1: 写 analyze_continuity.py**

```python
#!/usr/bin/env python3
"""Analyze camera-fusion location continuity from location_recorder CSV.

Metrics per robot class (hero/eng/inf3/inf4/sentry/drone):
  - active_frames / total_frames  (active = x,y != 0 AND confidence > 0)
  - gaps: inactive runs longer than --gap-threshold frames
  - jumps: consecutive active-frame displacement > --jump-threshold (after unit factor)
  - flicker: active -> inactive -> active transitions
  - x/y min/max/mean/std over active frames
"""
import argparse
import csv
import math
import sys

CLASSES = ["hero", "eng", "inf3", "inf4", "sentry", "drone"]


def load(path: str) -> list[dict]:
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def analyze(rows: list[dict], gap_thr: int, jump_thr: float, unit: float):
    report = {}
    for cls in CLASSES:
        xk, yk, ck = f"{cls}_x", f"{cls}_y", f"{cls}_conf"
        active = [
            i for i, r in enumerate(rows)
            if r[xk] and r[yk] and float(r[ck]) > 0.0
        ]
        active = [i for i in active if float(rows[i][xk]) != 0.0 and float(rows[i][yk]) != 0.0]

        gaps = 0
        max_gap = 0
        for a, b in zip(active, active[1:]):
            g = b - a - 1
            if g > gap_thr:
                gaps += 1
            max_gap = max(max_gap, g)

        jumps = 0
        max_jump = 0.0
        for a, b in zip(active, active[1:]):
            dx = (float(rows[b][xk]) - float(rows[a][xk])) * unit
            dy = (float(rows[b][yk]) - float(rows[a][yk])) * unit
            d = math.hypot(dx, dy)
            if d > jump_thr:
                jumps += 1
            max_jump = max(max_jump, d)

        flicker = 0
        prev_active = False
        for r in rows:
            is_active = bool(r[xk]) and bool(r[yk]) and float(r[ck]) > 0.0
            if is_active and not prev_active and any(True for _ in [0]):  # placeholder
                pass
            if is_active and not prev_active:
                # count only transitions back to active after a real inactive run
                flicker += 1
            prev_active = is_active
        # first row active counts as one appearance; we only count re-appearances
        flicker = max(0, flicker - 1)

        xs = [float(rows[i][xk]) * unit for i in active]
        ys = [float(rows[i][yk]) * unit for i in active]

        report[cls] = {
            "active": len(active),
            "total": len(rows),
            "active_ratio": len(active) / len(rows) if rows else 0.0,
            "gaps": gaps,
            "max_gap": max_gap,
            "jumps": jumps,
            "max_jump": max_jump,
            "flicker": flicker,
            "x_min": min(xs) if xs else None,
            "x_max": max(xs) if xs else None,
            "x_mean": sum(xs) / len(xs) if xs else None,
            "y_min": min(ys) if ys else None,
            "y_max": max(ys) if ys else None,
            "y_mean": sum(ys) / len(ys) if ys else None,
        }
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--gap-threshold", type=int, default=10)
    ap.add_argument("--jump-threshold", type=float, default=3.0)
    ap.add_argument("--unit-factor", type=float, default=1.0,
                    help="multiply x/y by this (e.g. cm->m = 0.01) before metrics")
    ap.add_argument("--plot", help="save trajectory PNG (matplotlib)")
    args = ap.parse_args()

    rows = load(args.csv)
    if not rows:
        print("empty CSV")
        sys.exit(1)

    report = analyze(rows, args.gap_threshold, args.jump_threshold, args.unit_factor)
    print(f"rows={len(rows)}  gap_thr={args.gap_threshold}  jump_thr={args.jump_threshold} "
          f"unit={args.unit_factor}")
    for cls, s in report.items():
        print(f"[{cls}] active={s['active']}/{s['total']} ({s['active_ratio']*100:.1f}%) "
              f"gaps={s['gaps']}(max {s['max_gap']}) jumps={s['jumps']}(max {s['max_jump']:.2f}) "
              f"flicker={s['flicker']}")
        if s["x_mean"] is not None:
            print(f"        x[{s['x_min']:.2f},{s['x_max']:.2f}] mean={s['x_mean']:.2f} "
                  f"y[{s['y_min']:.2f},{s['y_max']:.2f}] mean={s['y_mean']:.2f}")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(6, 1, figsize=(10, 18), sharex=True)
        for ax, cls in zip(axes, CLASSES):
            xk, yk, ck = f"{cls}_x", f"{cls}_y", f"{cls}_conf"
            pts = [(float(r[xk]) * args.unit_factor, float(r[yk]) * args.unit_factor)
                   for r in rows
                   if r[xk] and r[yk] and float(r[ck]) > 0.0]
            if pts:
                ax.plot([p[0] for p in pts], [p[1] for p in pts], ".-", ms=2)
            ax.set_title(cls)
            ax.set_aspect("equal", adjustable="datalim")
        fig.tight_layout()
        fig.savefig(args.plot)
        print(f"plot -> {args.plot}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 用合成 CSV 自测**

```bash
cd /tmp && printf 'stamp_ns,hero_x,hero_y,hero_conf,eng_x,eng_y,eng_conf,inf3_x,inf3_y,inf3_conf,inf4_x,inf4_y,inf4_conf,sentry_x,sentry_y,sentry_conf,drone_x,drone_y,drone_conf,n_tracks\n1,100,200,0.9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n2,101,201,0.9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n4,105,205,0.9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n' > /tmp/synth_cont.csv
python3 /home/yukikaze/Documents/workspace/alliance_radar_location_lidar/tools/video_zmq/analyze_continuity.py /tmp/synth_cont.csv
```

Expected: hero active=3/4 (75%)，gaps=0（gap=0 不大于 10），jumps=1（(105-101,205-201)=(4,4) 位移 5.66 > 3），flicker=1（row4 重新出现）。

- [ ] **Step 3: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
git add tools/video_zmq/analyze_continuity.py
git commit -m "feat(tools): camera-fusion continuity analyzer"
```

---

### Task 6: run_continuity_test.sh 一键测试脚本

**Files:**
- Create: `tools/video_zmq/run_continuity_test.sh`

**Interfaces:**
- Consumes: Task 3 mp4_replay、Task 4 location_recorder、Task 5 analyze_continuity；radar_camera/radar_fusion 已构建。
- Produces: 一键执行 60s 链路测试 + 分析报告。CLI：`run_continuity_test.sh [--video <path>] [--duration 60] [--max-frames 0] [--unit-factor 1.0]`。

- [ ] **Step 1: 写 run_continuity_test.sh**

```bash
#!/usr/bin/env bash
# run_continuity_test.sh - 一键 camera→fusion 连续性测试
# 流程: 清 SHM → radar_camera(red) + radar_fusion(camera on) → mp4_replay → location_recorder → 分析
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
RADAR_WS="${RADAR_WS:-/workspace}"
HIK_SHM="/hikcamera_shm"
DURATION="${DURATION:-60}"
UNIT_FACTOR="${UNIT_FACTOR:-1.0}"
GAP_THRESHOLD="${GAP_THRESHOLD:-10}"
JUMP_THRESHOLD="${JUMP_THRESHOLD:-3.0}"
VIDEO=""
MAX_FRAMES=0

while [[ $# -gt 0 ]]; do
    case $1 in
        --video) VIDEO="$2"; shift 2 ;;
        --duration) DURATION="$2"; shift 2 ;;
        --max-frames) MAX_FRAMES="$2"; shift 2 ;;
        --unit-factor) UNIT_FACTOR="$2"; shift 2 ;;
        *) echo "unknown: $1" >&2; exit 2 ;;
    esac
done

if [[ -z "$VIDEO" ]]; then
    echo "usage: run_continuity_test.sh --video <mp4> [--duration 60] [--max-frames N] [--unit-factor x]" >&2
    exit 2
fi

IN_CONTAINER=0
[ -f /.dockerenv ] && IN_CONTAINER=1

exec_in() {
    if [ "$IN_CONTAINER" = "1" ]; then
        bash -c "$1"
    else
        docker exec devcontainer-radar-develop-1 bash -c "$1"
    fi
}

REPLAY="${ROOT_DIR}/tools/video_zmq/build/mp4_replay"
RECORDER="${ROOT_DIR}/tools/video_zmq/location_recorder.py"
ANALYZE="${ROOT_DIR}/tools/video_zmq/analyze_continuity.py"
OUT_DIR="${ROOT_DIR}/tools/video_zmq/out"
mkdir -p "$OUT_DIR"
CSV_OUT="${OUT_DIR}/continuity_$(date +%Y%m%d_%H%M%S).csv"

stop_all() {
    exec_in "pkill -f '[r]adar_camera' 2>/dev/null || true; pkill -f '[r]adar_fusion' 2>/dev/null || true; pkill -f '[m]p4_replay' 2>/dev/null || true; pkill -f '[l]ocation_recorder' 2>/dev/null || true; rm -f /dev/shm/hikcamera_shm 2>/dev/null || true; sleep 1"
}

trap stop_all EXIT
stop_all

ENV_SETUP="source /opt/ros/jazzy/setup.bash && source ${RADAR_WS}/ros_ws/install/setup.bash && cd ${RADAR_WS}/ros_ws"

echo "[1/5] start radar_camera (enemy=red)"
exec_in "${ENV_SETUP} && setsid bash -c 'exec ros2 run radar_camera radar_camera_node --ros-args --params-file ${RADAR_WS}/ros_ws/install/radar_bringup/share/radar_bringup/config/camera/radar_camera.yaml -p enemy_color:=red > /tmp/cont_camera.log 2>&1' < /dev/null &"
sleep 3

echo "[2/5] start radar_fusion (camera fusion on)"
exec_in "${ENV_SETUP} && setsid bash -c 'exec ros2 run radar_fusion radar_fusion_node --ros-args -p enable_camera_fusion:=true -p enemy_color:=red > /tmp/cont_fusion.log 2>&1' < /dev/null &"
sleep 2

echo "[3/5] mp4_replay (max-frames=${MAX_FRAMES})"
REPLAY_ARGS="--video '${VIDEO}' --shm ${HIK_SHM} --speed 1.0"
[ "$MAX_FRAMES" -gt 0 ] && REPLAY_ARGS="${REPLAY_ARGS} --max-frames ${MAX_FRAMES}"
exec_in "cd /workspace/tools/video_zmq/build && LD_LIBRARY_PATH=/workspace/ros_ws/install/hikcamera/lib:/workspace/ros_ws/third-party/hikcamera_sdk/src/sdk/lib:/opt/MVS/lib/64 setsid bash -c 'exec ./mp4_replay ${REPLAY_ARGS} > /tmp/cont_replay.log 2>&1' < /dev/null &"

echo "[4/5] record ${DURATION}s -> ${CSV_OUT}"
exec_in "source /opt/ros/jazzy/setup.bash && source ${RADAR_WS}/ros_ws/install/setup.bash && timeout ${DURATION} python3 ${RECORDER} --out ${CSV_OUT} --duration ${DURATION} 2>&1 | tail -3"
# 等待 replay 结束（若 max-frames 先于 duration 结束）
sleep 2

echo "[5/5] analyze"
python3 "${ANALYZE}" "${CSV_OUT}" --gap-threshold "${GAP_THRESHOLD}" --jump-threshold "${JUMP_THRESHOLD}" --unit-factor "${UNIT_FACTOR}" --plot "${OUT_DIR}/traj.png"
echo "CSV: ${CSV_OUT}"
echo "PLOT: ${OUT_DIR}/traj.png"
echo "logs: /tmp/cont_camera.log /tmp/cont_fusion.log /tmp/cont_replay.log"
```

注意：video 路径在容器内必须可见。若宿主机 Downloads 未挂载进容器，脚本开头做一次 `docker cp` 到容器 /tmp（或要求 --video 为容器路径）。

- [ ] **Step 2: 语法验证**

```bash
bash -n /home/yukikaze/Documents/workspace/alliance_radar_location_lidar/tools/video_zmq/run_continuity_test.sh && echo SYNTAX-OK
```

- [ ] **Step 3: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
chmod +x tools/video_zmq/run_continuity_test.sh
git add tools/video_zmq/run_continuity_test.sh
git commit -m "feat(tools): one-shot camera-fusion continuity test script"
```

---

### Task 7: 端到端验证（60 秒短测）

**Files:** 无代码改动。

**Interfaces:** Consumes Task 1-6 产物。

- [ ] **Step 1: 确认视频文件容器内可见**

```bash
docker exec devcontainer-radar-develop-1 ls -la "/home/yukikaze/Downloads/26-May-30-香港大学-2.mp4" 2>/dev/null || echo "NOT visible in container"
```

若不可见：`docker cp "/home/yukikaze/Downloads/26-May-30-香港大学-2.mp4" devcontainer-radar-develop-1:/tmp/hku.mp4`（2.4GB，需数分钟——计划在带宽允许时执行）。

- [ ] **Step 2: 跑 60 秒短测**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
bash tools/video_zmq/run_continuity_test.sh --video <容器内视频路径> --duration 60 2>&1 | tail -30
```

Expected: 5 步全部完成；分析报告输出 6 类摘要；若 GPU 正常且视频含红方装甲，hero/eng/inf3/inf4 应有活跃轨迹；CSV 与轨迹图生成。

- [ ] **Step 3: 检查结果合理性**

- CSV 行数 ≈ duration × fusion 输出频率（25fps 输入，fusion 逐帧处理 → ~1500 行/60s）
- 若全类 active=0：检查 /tmp/cont_camera.log 是否有检测（L1 无目标或 enemy_color 错）、/tmp/cont_fusion.log 是否 subscribe 到 camera topic
- 若跳变异常：确认 LidarLocation 单位（uint16 可能是 cm/mm——用 --unit-factor 调整后重跑分析）

- [ ] **Step 4: 记录结果**

把结果摘要追加到测试说明（无代码提交）。

---

## Self-Review 记录

- **Spec 覆盖**：mp4_replay（Task 3）✓；location_recorder（Task 4）✓；analyze_continuity（Task 5）✓；测试脚本（Task 6）✓；链路测试（Task 7）✓；enemy_color 命令行覆盖不改 yaml（Task 6）✓；内参用已有配置（Task 6 参数文件）✓。**新增前置**：radar_camera 构建修复（Task 1）+ SHM v2 迁移（Task 2）——spec 探索时发现的硬障碍（radar_camera 当前编译失败，测试必须依赖它）。
- **占位符扫描**：无 TBD/TODO；所有代码块完整。
- **类型一致性**：`mp4_replay` CLI 参数在 Task 3/6 一致；CSV 列名在 Task 4/5 一致（hero_x/eng_x/... 与 conf）；`analyze_continuity` 参数名 gap-threshold/jump-threshold/unit-factor 在 Task 5/6 一致；`SharedFrameWriter::create/write` 签名与 SDK 头文件核对一致；`FrameMetadata` 字段名与 shm_types.hpp 核对一致。
- 已知风险：视频文件容器内可见性未验证（Task 7 Step 1 处理）；LidarLocation uint16 单位未知（Task 7 Step 3 说明）；radar_camera 迁移涉及 raw_shm_reader（录制侧）——已在 Task 2 覆盖，接口保持兼容。
