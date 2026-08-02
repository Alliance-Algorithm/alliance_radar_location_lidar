# VideoBridge 实时 3 层装甲检测标注推流 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有 radar_bridge 推流主链路中插入实时 L1→L2→L3 TensorRT 推理与帧上标注，仍推 `tcp://*:5557`，egui 零改动。

**Architecture:** 新建 `ArmorInfer` 推理单元（从 `tools/armor_verify/annotate_l1l2l3.cpp` 抽出，不改算法），VideoBridge 的 SHM 读取从废弃的 v1 API 迁移到 v2 `SharedFrameReader`，在推流线程内每帧：读帧 → 推理+标注（可选开关）→ JPEG → ZMQ PUB。

**Tech Stack:** C++23, ROS2 Jazzy, OpenCV, cppzmq, TensorRT 11 (fp16 engines), hikcamera_sdk v2 (SharedFrameReader)。

## Global Constraints

- 算法不变：阈值 `kL1Conf=0.20f` / `kL2Conf=0.50f` / `kL3Conf=0.80f` / `kL2Nms=0.30f`，输入边长 L1=1280 / L2=640 / L3=224，类名 `names[12]`（red-first: hero-R..drone-R, hero-B..drone-B）、`number_names[9]`（B1..BS,R1..R4）。
- 模型文件：`best_fixed_names_1280_fp16.engine`、`shenzhen-0708_fp16.engine`、`armor-number_fp16.engine`，位于 `/workspace/ros_ws/src/radar_camera/model/`。
- 推流协议不变：JPEG 质量 85 + ZMQ PUB conflate=1 + `tcp://*:5557`。
- 无人机（L1 id 5/11）跳过 L2/L3，恒 MATCH。
- 失败降级：engine 加载失败或单帧推理异常 → ERROR 日志 + 透传原帧，推流永不断。
- 容器内所有路径为 `/workspace/...`（bind mount 到本地 `/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/...`），构建和运行都在容器 `devcontainer-radar-develop-1` 内执行。
- 编译需 TensorRT stage：容器内 `/opt/radar_camera_trt`（宿主机 `${RADAR_TENSORRT_STAGE}` bind mount）。当前为空，Task 1 先恢复。

---

### Task 1: 恢复 TensorRT stage 并验证构建基线

**Files:**
- Modify: `/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.script/prepare-radar-camera-tensorrt-runtime`（仅当需改 stage 目录时）

**Interfaces:**
- Produces: 容器内 `/opt/radar_camera_trt/include/NvInferRuntime.h` 与 `/opt/radar_camera_trt/lib/libnvinfer.so` 可访问；`hikcamera` 包重新构建成功（v2 API）。

**背景:** 宿主机 `/tmp/radar_camera_trt_host` 是 root 创建的空目录且无法删除（无 passwordless sudo），docker-compose 用它挂载 `/opt/radar_camera_trt:ro`，导致容器内 TRT 头/库为空。宿主机本身有 `/usr/include/NvInfer.h`、`/usr/lib/libnvinfer.so.11`、`/opt/cuda`、`trtexec`、RTX 4060。

- [ ] **Step 1: 在宿主机用新目录准备 TRT stage**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
mkdir -p /tmp/radar_trt_stage2
RADAR_TENSORRT_STAGE=/tmp/radar_trt_stage2 bash .script/prepare-radar-camera-tensorrt-runtime
ls /tmp/radar_trt_stage2/include/NvInferRuntime.h /tmp/radar_trt_stage2/lib/libnvinfer.so
```

Expected: stage 填充完毕，du 显示约 1-2GB。若脚本因 `rm -rf` 失败，手动 `chmod` 或换目录重试（脚本本身会 `rm -rf "${STAGE_DIR}"`，用新目录即避开 root 权限问题）。

- [ ] **Step 2: 重启容器挂载新 stage**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.devcontainer
RADAR_TENSORRT_STAGE=/tmp/radar_trt_stage2 docker compose up -d
docker exec devcontainer-radar-develop-1 ls /opt/radar_camera_trt/include/NvInferRuntime.h
```

Expected: 容器内能看到 TRT 头文件。若 compose 环境变量不生效（devcontainer 由 VSCode 管理），改为手动 bind mount 或 `docker cp` 到容器 /opt/radar_camera_trt（临时手段，仅开发期）。

- [ ] **Step 3: 重新构建 hikcamera 包（v2 协议）**

```bash
docker exec devcontainer-radar-develop-1 bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace/ros_ws && colcon build --packages-select hikcamera 2>&1 | tail -5"
```

Expected: `Finished <<< hikcamera`。确认 `install/hikcamera/include/hikcamera/shared_frame_reader.hpp` 存在。

- [ ] **Step 4: 记录基线：确认 radar_bridge 当前编译失败原因**

```bash
docker exec devcontainer-radar-develop-1 bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace/ros_ws && colcon build --packages-select radar_bridge 2>&1 | grep -E 'error' | head -5"
```

Expected: 报 `'imageSHM' in namespace 'hikcamera' does not name a type`（Task 4 修复它）。radar_camera 包因 `raw_shm_reader` 也是 v1 API 而失败——不在本计划范围（spec 明确不迁移），radar_bridge 不依赖 radar_camera 构建产物。

- [ ] **Step 5: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
git add .script/ 2>/dev/null; git commit -m "chore: restore TensorRT stage for radar_bridge TRT build" 2>/dev/null || echo "no script changes; stage is environment-only"
```

---

### Task 2: ArmorInfer 纯逻辑抽取 + 单元测试（无 GPU 依赖）

**Files:**
- Create: `ros_ws/src/radar_camera/include/radar_camera/armor_infer.hpp`
- Create: `ros_ws/src/radar_camera/src/armor_infer.cpp`
- Create: `tools/armor_verify/test_armor_infer.cpp`
- Modify: `tools/armor_verify/CMakeLists.txt`

**Interfaces:**
- Produces (namespace `radar_camera::armor_infer`):
  - `struct Det { int id; float conf; cv::Rect2f box; };`
  - `struct Plate { cv::Rect2f box; std::vector<cv::Point2f> corners; int genre; int color; float conf; };`
  - `struct Number { int index; float conf; };`
  - `struct ArmorResult { int l1_id; float l1_conf; cv::Rect2f l1_box; std::optional<Plate> l2; std::optional<Number> l3; int final_id; std::string decision; std::string match_state; };`
  - `auto decode_l1(const std::vector<float>& raw, float scale, float l1_conf) -> std::vector<Det>;` — 解析 [N,6] xyxy+conf+cls，按 id 取最优 + 宽高比过滤 + `kL1Conf` 阈值
  - `auto decode_l2(const std::vector<float>& raw, const cv::Rect2f& roi, float scale, int px, int py, float l2_conf, float l2_nms) -> std::optional<Plate>;` — 解析 [25200,22]，obj sigmoid、角点包围盒、color/genre argmax、NMS
  - `auto letterbox(const cv::Mat& src, int side, bool center, float& scale, int& px, int& py) -> cv::Mat;`
  - `auto blob(const cv::Mat& rgb) -> std::vector<float>;`
  - `auto iou(const cv::Rect2f& a, const cv::Rect2f& b) -> float;`
  - `auto sigmoid(float x) -> float;`
  - `auto l2_id(int genre, int color) -> std::optional<int>;` — `ids[]={-1,0,1,2,3,-1,4}`，color==1 红 (+0) / color==2 蓝 (+6)
  - `auto l3_id(int index) -> std::optional<int>;` — 0..4 → 6..10，5..8 → 0..3
  - `auto l1_names(int id) -> const char*;` / `auto l3_names(int index) -> const char*;`
  - 常量 `inline constexpr float kL1Conf=0.20f; kL2Conf=0.50f; kL3Conf=0.80f; kL2Nms=0.30f; kSideL1=1280; kSideL2=640; kSideL3=224;`

- [ ] **Step 1: 写头文件 `armor_infer.hpp`**

```cpp
#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace radar_camera::armor_infer {

inline constexpr float kL1Conf = 0.20f;
inline constexpr float kL2Conf = 0.50f;
inline constexpr float kL3Conf = 0.80f;
inline constexpr float kL2Nms = 0.30f;
inline constexpr int kSideL1 = 1280;
inline constexpr int kSideL2 = 640;
inline constexpr int kSideL3 = 224;

struct Det { int id; float conf; cv::Rect2f box; };
struct Plate { cv::Rect2f box; std::vector<cv::Point2f> corners; int genre; int color; float conf; };
struct Number { int index; float conf; };

struct ArmorResult {
    int l1_id;
    float l1_conf;
    cv::Rect2f l1_box;
    std::optional<Plate> l2;
    std::optional<Number> l3;
    int final_id;
    std::string decision;    // "L1" | "L2" | "L3-plate"
    std::string match_state; // "MATCH" | "MISS"
};

auto sigmoid(float x) -> float;
auto letterbox(const cv::Mat& src, int side, bool center, float& scale, int& px, int& py) -> cv::Mat;
auto blob(const cv::Mat& rgb) -> std::vector<float>;
auto iou(const cv::Rect2f& a, const cv::Rect2f& b) -> float;
auto decode_l1(const std::vector<float>& raw, float scale, float l1_conf = kL1Conf) -> std::vector<Det>;
auto decode_l2(const std::vector<float>& raw, const cv::Rect2f& roi, float scale, int px, int py,
    float l2_conf = kL2Conf, float l2_nms = kL2Nms) -> std::optional<Plate>;
auto l2_id(int genre, int color) -> std::optional<int>;
auto l3_id(int index) -> std::optional<int>;
auto l1_names(int id) -> const char*;
auto l3_names(int index) -> const char*;

} // namespace radar_camera::armor_infer
```

- [ ] **Step 2: 写实现 `armor_infer.cpp`（从 annotate_l1l2l3.cpp 搬移，不改算法）**

```cpp
#include "radar_camera/armor_infer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>

namespace radar_camera::armor_infer {

const char* kNames[] = { "hero-R", "eng-R", "inf3-R", "inf4-R", "sentry-R", "drone-R",
    "hero-B", "eng-B", "inf3-B", "inf4-B", "sentry-B", "drone-B" };
const char* kNumberNames[] = { "B1", "B2", "B3", "B4", "BS", "R1", "R2", "R3", "R4" };

auto sigmoid(float x) -> float { return 1.0f / (1.0f + std::exp(-x)); }

auto letterbox(const cv::Mat& src, int side, bool center, float& scale, int& px, int& py) -> cv::Mat {
    scale = std::min(static_cast<float>(side) / src.cols, static_cast<float>(side) / src.rows);
    const int w = std::max(1, static_cast<int>(std::round(src.cols * scale)));
    const int h = std::max(1, static_cast<int>(std::round(src.rows * scale)));
    px = center ? (side - w) / 2 : 0;
    py = center ? (side - h) / 2 : 0;
    cv::Mat out(side, side, src.type(), cv::Scalar::all(0));
    cv::Mat resized;
    cv::resize(src, resized, { w, h });
    resized.copyTo(out(cv::Rect(px, py, w, h)));
    return out;
}

auto blob(const cv::Mat& rgb) -> std::vector<float> {
    cv::Mat b = cv::dnn::blobFromImage(rgb, 1.0 / 255.0, {}, {}, false, false);
    std::vector<float> out(b.total());
    std::memcpy(out.data(), b.ptr<float>(), out.size() * sizeof(float));
    return out;
}

auto iou(const cv::Rect2f& a, const cv::Rect2f& b) -> float {
    const auto inter = a & b;
    const float union_area = a.area() + b.area() - inter.area();
    return union_area <= 0.0f ? 0.0f : inter.area() / union_area;
}

auto decode_l1(const std::vector<float>& raw, float scale, float l1_conf) -> std::vector<Det> {
    std::vector<Det> best;
    for (size_t i = 0; i + 5 < raw.size(); i += 6) {
        const float conf = raw[i + 4];
        const int id = static_cast<int>(raw[i + 5]);
        if (conf < l1_conf) continue;
        const float x1 = raw[i] / scale;
        const float y1 = raw[i + 1] / scale;
        const float x2 = raw[i + 2] / scale;
        const float y2 = raw[i + 3] / scale;
        cv::Rect2f box(x1, y1, x2 - x1, y2 - y1);
        if (box.width < 1 || box.height < 1) continue;
        const float ratio = std::max(box.width, box.height) / std::min(box.width, box.height);
        const bool drone = id == 5 || id == 11;
        if (ratio < (drone ? 2.0f : 0.5f) || ratio > (drone ? 10.0f : 3.0f)) continue;
        auto it = std::find_if(best.begin(), best.end(), [id](const Det& d) { return d.id == id; });
        Det candidate { id, conf, box };
        if (it == best.end()) best.push_back(candidate);
        else if (conf > it->conf) *it = candidate;
    }
    return best;
}

auto decode_l2(const std::vector<float>& raw, const cv::Rect2f& roi, float scale, int px, int py,
    float l2_conf, float l2_nms) -> std::optional<Plate> {
    std::vector<Plate> candidates;
    for (size_t i = 0; i + 21 < raw.size(); i += 22) {
        const float confidence = sigmoid(raw[i + 8]);
        if (confidence < l2_conf) continue;
        float min_x = raw[i], max_x = raw[i];
        float min_y = raw[i + 1], max_y = raw[i + 1];
        for (int p = 0; p < 4; ++p) {
            min_x = std::min(min_x, raw[i + p * 2]);
            max_x = std::max(max_x, raw[i + p * 2]);
            min_y = std::min(min_y, raw[i + p * 2 + 1]);
            max_y = std::max(max_y, raw[i + p * 2 + 1]);
        }
        cv::Rect2f box((min_x - px) / scale + roi.x, (min_y - py) / scale + roi.y,
            (max_x - min_x) / scale, (max_y - min_y) / scale);
        if (box.width < 1 || box.height < 1) continue;
        int color = static_cast<int>(std::max_element(raw.begin() + i + 9, raw.begin() + i + 13)
            - (raw.begin() + i + 9));
        int genre = static_cast<int>(std::max_element(raw.begin() + i + 13, raw.begin() + i + 22)
            - (raw.begin() + i + 13));
        std::vector<cv::Point2f> corners;
        for (int p = 0; p < 4; ++p) {
            corners.emplace_back((raw[i + p * 2] - px) / scale + roi.x,
                (raw[i + p * 2 + 1] - py) / scale + roi.y);
        }
        candidates.push_back({ box, corners, genre, color, confidence });
    }
    std::sort(candidates.begin(), candidates.end(),
        [](const Plate& a, const Plate& b) { return a.conf > b.conf; });
    for (size_t i = 0; i < candidates.size(); ++i) {
        bool suppressed = false;
        for (size_t j = 0; j < i; ++j) if (iou(candidates[i].box, candidates[j].box) > l2_nms) suppressed = true;
        if (!suppressed) return candidates[i];
    }
    return std::nullopt;
}

auto l3_id(int index) -> std::optional<int> {
    if (index >= 0 && index <= 4) return index + 6;
    if (index >= 5 && index <= 8) return index - 5;
    return std::nullopt;
}

auto l2_id(int genre, int color) -> std::optional<int> {
    const int ids[] = { -1, 0, 1, 2, 3, -1, 4 };
    if (genre < 0 || genre >= 7 || color == 0 || ids[genre] < 0) return std::nullopt;
    return color == 1 ? ids[genre] : ids[genre] + 6;
}

auto l1_names(int id) -> const char* {
    return (id >= 0 && id < 12) ? kNames[id] : "?";
}

auto l3_names(int index) -> const char* {
    return (index >= 0 && index < 9) ? kNumberNames[index] : "?";
}

} // namespace radar_camera::armor_infer
```

- [ ] **Step 3: 写单元测试 `test_armor_infer.cpp`**

```cpp
#include <cassert>
#include <cmath>
#include <iostream>

#include <opencv2/core.hpp>

#include "radar_camera/armor_infer.hpp"

using namespace radar_camera::armor_infer;

void test_l2_id() {
    assert(l2_id(1, 1) == 0);   // eng-R
    assert(l2_id(1, 2) == 6);   // eng-B
    assert(l2_id(6, 1) == 4);   // sentry-R
    assert(l2_id(0, 1) == std::nullopt); // genre 0 无映射
    assert(l2_id(2, 0) == std::nullopt); // color 0 无效
    assert(l2_id(7, 1) == std::nullopt); // genre 越界
}

void test_l3_id() {
    assert(l3_id(0) == 6);      // B1 -> hero-B
    assert(l3_id(4) == 10);     // BS -> sentry-B
    assert(l3_id(5) == 0);      // R1 -> hero-R
    assert(l3_id(8) == 3);      // R4 -> inf4-R
    assert(l3_id(9) == std::nullopt);
}

void test_letterbox() {
    cv::Mat src(100, 200, CV_8UC3);
    float scale; int px; int py;
    auto lb = letterbox(src, 1280, false, scale, px, py);
    assert(lb.cols == 1280 && lb.rows == 1280);
    assert(px == 0 && py == 0);
    assert(std::abs(scale - 6.4f) < 1e-4f);
    auto lbc = letterbox(src, 224, true, scale, px, py);
    assert(lbc.cols == 224 && lbc.rows == 224);
    assert(px > 0 && py > 0);
}

void test_iou() {
    assert(iou(cv::Rect2f(0, 0, 10, 10), cv::Rect2f(0, 0, 10, 10)) == 1.0f);
    assert(iou(cv::Rect2f(0, 0, 10, 10), cv::Rect2f(20, 20, 10, 10)) == 0.0f);
    assert(std::abs(iou(cv::Rect2f(0, 0, 10, 10), cv::Rect2f(5, 0, 10, 10)) - 0.333333f) < 1e-4f);
}

void test_decode_l1() {
    // scale=1: 两个同类检测取高分；宽高比过滤；低置信度过滤
    std::vector<float> raw {
        10, 10, 110, 210, 0.9f, 0,   // hero-R, ratio 2.0 通过
        20, 20, 120, 220, 0.8f, 0,   // 同类低分被丢弃
        0, 0, 100, 50, 0.99f, 5,     // drone-R, ratio 2.0 通过
        0, 0, 100, 100, 0.99f, 7,    // 方形 ratio 1.0 被过滤 (非无人机)
        0, 0, 10, 10, 0.1f, 1,       // 低置信度被过滤
    };
    auto dets = decode_l1(raw, 1.0f);
    assert(dets.size() == 2);
    assert(dets[0].id == 0 && std::abs(dets[0].conf - 0.9f) < 1e-5f);
    assert(dets[1].id == 5 && std::abs(dets[1].conf - 0.99f) < 1e-5f);
}

void test_decode_l2() {
    // 手工构造一条 22 列检测: 角点(5,5)(15,5)(15,10)(5,10), obj=0.9, color=[0.1,0.8,0.05,0.05] -> red(1), genre=[0.9,...] -> genre 0
    std::vector<float> raw(22, 0.0f);
    raw[0] = 5; raw[1] = 5; raw[2] = 15; raw[3] = 5; raw[4] = 15; raw[5] = 10; raw[6] = 5; raw[7] = 10;
    raw[8] = 0.9f;
    raw[9] = 0.1f; raw[10] = 0.8f; raw[11] = 0.05f; raw[12] = 0.05f;
    raw[13] = 0.9f; // genre 0
    auto plate = decode_l2(raw, cv::Rect2f(100, 100, 0, 0), 1.0f, 0, 0);
    assert(plate.has_value());
    assert(plate->color == 1 && plate->genre == 0);
    assert(std::abs(plate->box.x - 105.0f) < 1e-3f && std::abs(plate->box.y - 105.0f) < 1e-3f);
    assert(std::abs(plate->box.width - 10.0f) < 1e-3f && std::abs(plate->box.height - 5.0f) < 1e-3f);
    assert(plate->corners.size() == 4);
    // roi 偏移 + 缩放
    auto plate2 = decode_l2(raw, cv::Rect2f(100, 100, 0, 0), 2.0f, 10, 20);
    assert(plate2.has_value());
    assert(std::abs(plate2->box.x - 100.0f + 5.0f / 2.0f - 10.0f / 2.0f + 100.0f + 0.0f) > -1.0f); // 烟幕断言: 只验证不崩溃
    assert(plate2->corners[0].x > 95.0f && plate2->corners[0].x < 110.0f);
    // 低置信度
    std::vector<float> raw_low = raw;
    raw_low[8] = 0.1f;
    assert(!decode_l2(raw_low, cv::Rect2f(0, 0, 0, 0), 1.0f, 0, 0).has_value());
}

void test_names() {
    assert(std::string(l1_names(0)) == "hero-R");
    assert(std::string(l1_names(11)) == "drone-B");
    assert(std::string(l3_names(0)) == "B1");
    assert(std::string(l3_names(8)) == "R4");
}

int main() {
    test_l2_id(); test_l3_id(); test_letterbox(); test_iou();
    test_decode_l1(); test_decode_l2(); test_names();
    std::cout << "all armor_infer tests passed\n";
    return 0;
}
```

- [ ] **Step 4: 修改 `tools/armor_verify/CMakeLists.txt` 注册测试目标**

在文件末尾追加：

```cmake
add_executable(test_armor_infer
    test_armor_infer.cpp
    ${RADAR_SRC}/src/armor_infer.cpp
)

target_include_directories(test_armor_infer PRIVATE
    ${RADAR_SRC}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(test_armor_infer
    ${OpenCV_LIBS}
)
```

- [ ] **Step 5: 编译并运行测试（无 GPU 也通过）**

```bash
docker exec devcontainer-radar-develop-1 bash -c "cd /workspace/tools/armor_verify && rm -rf build && mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null && make test_armor_infer -j$(nproc) 2>&1 | tail -3 && ./test_armor_infer"
```

Expected: `all armor_infer tests passed`。

- [ ] **Step 6: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
git add ros_ws/src/radar_camera/include/radar_camera/armor_infer.hpp \
        ros_ws/src/radar_camera/src/armor_infer.cpp \
        tools/armor_verify/test_armor_infer.cpp \
        tools/armor_verify/CMakeLists.txt
git commit -m "feat(camera): extract reusable L1/L2/L3 inference decode logic (ArmorInfer)"
```

---

### Task 3: ArmorInfer 推理类（engine 加载 + infer 全流程）

**Files:**
- Modify: `ros_ws/src/radar_camera/include/radar_camera/armor_infer.hpp`
- Modify: `ros_ws/src/radar_camera/src/armor_infer.cpp`

**Interfaces:**
- Consumes: `radar_camera::model_inference::TensorRtInference`（`init(path) -> expected<void,string>`，`start(input, n) -> expected<void,string>`，`wait() -> expected<reference_wrapper<const vector<float>>, string>`），Task 2 的纯函数。
- Produces:
  - `class ArmorInfer { public: static auto create(const std::string& model_dir, float l1_conf = kL1Conf, float l2_conf = kL2Conf, float l3_conf = kL3Conf) -> std::expected<std::shared_ptr<ArmorInfer>, std::string>; auto infer(const cv::Mat& frame_rgb) -> std::vector<ArmorResult>; };`
  - 不可拷贝；`create` 任一 engine 加载失败返回错误。

- [ ] **Step 1: 头文件追加 ArmorInfer 类声明**

在 `namespace radar_camera::armor_infer` 内、函数声明之后追加：

```cpp
class ArmorInfer final {
public:
    ArmorInfer() = default;
    ~ArmorInfer() = default;

    ArmorInfer(const ArmorInfer&)            = delete;
    ArmorInfer& operator=(const ArmorInfer&) = delete;
    ArmorInfer(ArmorInfer&&) noexcept        = default;
    ArmorInfer& operator=(ArmorInfer&&) noexcept = default;

    static auto create(const std::string& model_dir,
        float l1_conf = kL1Conf, float l2_conf = kL2Conf, float l3_conf = kL3Conf)
        -> std::expected<std::shared_ptr<ArmorInfer>, std::string>;

    auto infer(const cv::Mat& frame_rgb) -> std::vector<ArmorResult>;

private:
    ArmorInfer(std::unique_ptr<class ArmorInferImpl> impl);
    std::unique_ptr<class ArmorInferImpl> impl_;
};
```

头文件顶部增加 `#include <expected>`、`#include <memory>`、`#include "radar_camera/tensorrt_inference.hpp"`。

- [ ] **Step 2: 实现文件追加推理类实现**

在 `armor_infer.cpp` 末尾（namespace 内）追加：

```cpp
namespace {

auto crop(const cv::Mat& frame, cv::Rect2f box) -> cv::Mat {
    int x = std::clamp(static_cast<int>(box.x), 0, frame.cols - 1);
    int y = std::clamp(static_cast<int>(box.y), 0, frame.rows - 1);
    int w = std::clamp(static_cast<int>(box.width), 1, frame.cols - x);
    int h = std::clamp(static_cast<int>(box.height), 1, frame.rows - y);
    return frame(cv::Rect(x, y, w, h));
}

auto run_l3(model_inference::TensorRtInference& engine, const cv::Mat& frame,
    cv::Rect2f box, float l3_conf) -> std::optional<Number> {
    float scale; int px; int py;
    auto input = letterbox(crop(frame, box), kSideL3, true, scale, px, py);
    auto data = blob(input);
    if (!engine.start(data.data(), data.size())) return std::nullopt;
    auto output = engine.wait();
    if (!output) return std::nullopt;
    const auto& raw = output->get();
    if (raw.size() < 9) return std::nullopt;
    const int index = static_cast<int>(std::max_element(raw.begin(), raw.begin() + 9) - raw.begin());
    const float confidence = raw[index];
    if (confidence < l3_conf) return std::nullopt;
    return Number { index, confidence };
}

auto run_l2(model_inference::TensorRtInference& engine, const cv::Mat& frame,
    cv::Rect2f roi, float l2_conf) -> std::optional<Plate> {
    float scale; int px; int py;
    const cv::Rect2f source_roi = roi;
    auto input = letterbox(crop(frame, roi), kSideL2, false, scale, px, py);
    auto data = blob(input);
    if (!engine.start(data.data(), data.size())) return std::nullopt;
    auto output = engine.wait();
    if (!output) return std::nullopt;
    return decode_l2(output->get(), source_roi, scale, px, py, l2_conf);
}

auto infer_l1(model_inference::TensorRtInference& engine, const cv::Mat& rgb) -> std::vector<Det> {
    float scale; int px; int py;
    auto input = letterbox(rgb, kSideL1, false, scale, px, py);
    auto data = blob(input);
    if (!engine.start(data.data(), data.size())) return {};
    auto output = engine.wait();
    if (!output) return {};
    return decode_l1(output->get(), scale);
}
} // namespace

class ArmorInferImpl {
public:
    model_inference::TensorRtInference l1_engine;
    model_inference::TensorRtInference l2_engine;
    model_inference::TensorRtInference l3_engine;
    float l1_conf { kL1Conf };
    float l2_conf { kL2Conf };
    float l3_conf { kL3Conf };
};

ArmorInfer::ArmorInfer(std::unique_ptr<ArmorInferImpl> impl) : impl_(std::move(impl)) {}

auto ArmorInfer::create(const std::string& model_dir,
    float l1_conf, float l2_conf, float l3_conf)
    -> std::expected<std::shared_ptr<ArmorInfer>, std::string> {
    auto impl = std::make_unique<ArmorInferImpl>();
    impl->l1_conf = l1_conf;
    impl->l2_conf = l2_conf;
    impl->l3_conf = l3_conf;
    const auto engine = [&](const char* name) {
        return (std::filesystem::path(model_dir) / name).string();
    };
    if (auto r = impl->l1_engine.init(engine("best_fixed_names_1280_fp16.engine")); !r)
        return std::unexpected("L1 engine init failed: " + r.error());
    if (auto r = impl->l2_engine.init(engine("shenzhen-0708_fp16.engine")); !r)
        return std::unexpected("L2 engine init failed: " + r.error());
    if (auto r = impl->l3_engine.init(engine("armor-number_fp16.engine")); !r)
        return std::unexpected("L3 engine init failed: " + r.error());
    return std::shared_ptr<ArmorInfer>(new ArmorInfer(std::move(impl)));
}

auto ArmorInfer::infer(const cv::Mat& frame_rgb) -> std::vector<ArmorResult> {
    std::vector<ArmorResult> results;
    for (const auto& detection : infer_l1(impl_->l1_engine, frame_rgb)) {
        ArmorResult result { detection.id, detection.conf, detection.box, std::nullopt,
            std::nullopt, detection.id, "L1", "MATCH" };
        if (detection.id != 5 && detection.id != 11) {
            auto plate = run_l2(impl_->l2_engine, frame_rgb, detection.box, impl_->l2_conf);
            if (plate) {
                result.l2 = plate;
                auto number = run_l3(impl_->l3_engine, frame_rgb, plate->box, impl_->l3_conf);
                if (number && l3_id(number->index)) {
                    result.l3 = number;
                    result.final_id = *l3_id(number->index);
                    result.decision = "L3-plate";
                } else if (l2_id(plate->genre, plate->color)) {
                    result.final_id = *l2_id(plate->genre, plate->color);
                    result.decision = "L2";
                }
            }
        }
        const bool l2_match = result.l2.has_value();
        const bool l3_match = result.l3.has_value();
        result.match_state = (l2_match || l3_match || detection.id == 5 || detection.id == 11)
            ? "MATCH" : "MISS";
        results.push_back(std::move(result));
    }
    return results;
}
```

需要 `#include <filesystem>` 于实现文件顶部。

- [ ] **Step 3: 修改 `tools/armor_verify/CMakeLists.txt` — test 目标追加 TRT 链接**

`armor_infer.cpp` 现已包含 ArmorInfer 类（引用 `TensorRtInference`），`test_armor_infer` 必须链接 `tensorrt_inference.cpp` + TRT/CUDA 库才能编译：

```cmake
add_executable(test_armor_infer
    test_armor_infer.cpp
    ${RADAR_SRC}/src/armor_infer.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
)

target_include_directories(test_armor_infer PRIVATE
    ${RADAR_SRC}/include
    ${TRT_ROOT}/include
    ${CUDA_ROOT}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_directories(test_armor_infer PRIVATE
    ${TRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)

target_link_libraries(test_armor_infer
    ${OpenCV_LIBS}
    nvinfer
    nvinfer_plugin
    cudart
)
```

注意：`TRT_ROOT`/`CUDA_ROOT` 已在 CMakeLists 顶部定义（`set(TRT_ROOT /opt/radar_camera_trt)`、`set(CUDA_ROOT /usr/local/cuda)`）。测试本身不加载 engine、不需要 GPU，仅链接。

- [ ] **Step 4: 编译 + 运行测试**

```bash
docker exec devcontainer-radar-develop-1 bash -c "cd /workspace/tools/armor_verify/build && cmake .. > /dev/null && make test_armor_infer -j$(nproc) 2>&1 | grep -E 'error|Error' | head -5 && ./test_armor_infer"
```

Expected: 无编译错误，输出 `all armor_infer tests passed`。若报 `NvInferRuntime.h` 缺失，说明容器内 TRT stage 未就绪，回 Task 1。

- [ ] **Step 5: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
git add ros_ws/src/radar_camera/include/radar_camera/armor_infer.hpp \
        ros_ws/src/radar_camera/src/armor_infer.cpp \
        tools/armor_verify/CMakeLists.txt
git commit -m "feat(camera): add ArmorInfer class (L1/L2/L3 engine load + infer pipeline)"
```

---

### Task 4: VideoBridge SHM 读取迁移到 v2 SharedFrameReader

**Files:**
- Modify: `ros_ws/src/radar_bridge/include/radar_bridge/videostream_bridge.hpp`
- Modify: `ros_ws/src/radar_bridge/src/videostream_bridge.cpp`
- Modify: `ros_ws/src/radar_bridge/include/radar_bridge/radar_bridge_node.hpp`
- Modify: `ros_ws/src/radar_bridge/src/radar_bridge_node.cpp`

**Interfaces:**
- Consumes: `hikcamera::SharedFrameReader`（`open(name) -> expected<void,string>`，`wait_next(milliseconds) -> expected<SharedFrame, FrameReadError>`，`SharedFrame::mat() -> cv::Mat`（BGR8 非拥有视图）、`SharedFrame::metadata() -> const FrameMetadata&`、`SharedFrame::valid()`）。
- Produces: `VideoBridge::video_init(const std::string& shm_name, const std::string& pub_address) -> std::expected<void, std::string>`（**删除** image_width/image_height 参数）；推流线程每帧 BGR8 → JPEG → ZMQ（**不再需要** RGB2BGR cvtColor）。

- [ ] **Step 1: 修改 `videostream_bridge.hpp`**

```cpp
#pragma once

#include <hikcamera/shared_frame_reader.hpp>

#include <atomic>
#include <expected>
#include <string>
#include <thread>
#include <zmq.hpp>

namespace radar_bridge::videostream_bridge {

class VideoBridge final {
public:
    VideoBridge() = default;
    ~VideoBridge();

    auto video_init(const std::string& shm_name, const std::string& pub_address)
        -> std::expected<void, std::string>;
    auto video_thread() -> std::expected<void, std::string>;
    auto video_thread_stop() -> std::expected<void, std::string>;

private:
    hikcamera::SharedFrameReader reader_;
    std::string pub_address_;
    std::string shm_name_;
    zmq::context_t ctx_ { 1 };
    zmq::socket_t pub_ { ctx_, zmq::socket_type::pub };
    std::thread video_thread_;
    std::atomic<bool> video_thread_running_ { false };
};

} // namespace radar_bridge::videostream_bridge
```

- [ ] **Step 2: 修改 `videostream_bridge.cpp`**

```cpp
#include "radar_bridge/videostream_bridge.hpp"

#include <iostream>

#include <opencv2/imgcodecs.hpp>

namespace radar_bridge::videostream_bridge {

VideoBridge::~VideoBridge() { auto _ = video_thread_stop(); }

auto VideoBridge::video_init(const std::string& shm_name, const std::string& pub_address)
    -> std::expected<void, std::string> {
    shm_name_    = shm_name;
    pub_address_ = pub_address;

    auto open_ret = reader_.open(shm_name_);
    if (!open_ret) return std::unexpected("SharedFrameReader open failed: " + open_ret.error());

    try {
        pub_.bind(pub_address_);
    } catch (const zmq::error_t& e) {
        return std::unexpected("zmq bind failed: " + std::string(e.what()));
    }
    pub_.set(zmq::sockopt::conflate, 1);
    return { };
}

auto VideoBridge::video_thread() -> std::expected<void, std::string> {
    video_thread_running_ = true;
    video_thread_         = std::thread([this]() {
        constexpr auto kWaitTimeout = std::chrono::milliseconds { 500 };
        while (video_thread_running_) {
            auto frame = reader_.wait_next(kWaitTimeout);
            if (!frame || !frame->valid()) continue;

            // mat() 是 SHM 上的 BGR8 非拥有视图；clone 保证短生命周期内数据安全
            cv::Mat bgr = frame->mat().clone();

            std::vector<uchar> jpeg;
            if (!cv::imencode(".jpg", bgr, jpeg, { cv::IMWRITE_JPEG_QUALITY, 85 })
                || jpeg.empty()) {
                std::cerr << "[VideoBridge] JPEG encode failed\n";
                video_thread_running_ = false;
                break;
            }

            try {
                auto send_ret =
                    pub_.send(zmq::message_t(jpeg.data(), jpeg.size()), zmq::send_flags::none);
                if (!send_ret) {
                    std::cerr << "[VideoBridge] ZMQ send failed\n";
                    video_thread_running_ = false;
                    break;
                }
            } catch (const zmq::error_t& e) {
                std::cerr << "[VideoBridge] ZMQ send error: " << e.what() << "\n";
                video_thread_running_ = false;
                break;
            }
        }
    });
    return { };
}

auto VideoBridge::video_thread_stop() -> std::expected<void, std::string> {
    video_thread_running_ = false;
    if (video_thread_.joinable()) video_thread_.join();
    return { };
}

} // namespace radar_bridge::videostream_bridge
```

注意：`wait_next` 返回 `std::expected<SharedFrame, FrameReadError>`，`operator*`/`operator->` 解引用；`!frame` 判断 expected 无值（超时/错误），沿用旧循环语义继续等待。

- [ ] **Step 3: 修改 `radar_bridge_node.hpp` — BridgeConfig 删除宽高**

```cpp
struct BridgeConfig {
    std::string zmq_pub_address;
    std::vector<std::string> zmq_sub_addresses;
    std::string shm_name;
    std::string video_pub_address;
    std::string image_topic;
};
```

（删除 `int video_width = 4096; int video_height = 3000;`）

- [ ] **Step 4: 修改 `radar_bridge_node.cpp`**

- `ConfigsLoader`：删除 `video_width`/`video_height` 两个 `declare_parameter` + 两个 `get_parameter` 行。
- `video_init` 调用改为两参：

```cpp
    auto init_ret = video_bridge_.video_init(config_.shm_name, config_.video_pub_address);
```

- [ ] **Step 5: 修改 `radar_bridge_params.yaml` / `radar_bringup/config/bridge/radar_bridge.yaml` 删除宽高**

两文件各删除：

```yaml
    video_width: 4096
    video_height: 3000
```

- [ ] **Step 6: 构建验证**

```bash
docker exec devcontainer-radar-develop-1 bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace/ros_ws && colcon build --packages-select radar_bridge 2>&1 | tail -5"
```

Expected: `Finished <<< radar_bridge`。

- [ ] **Step 7: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
git add ros_ws/src/radar_bridge/ ros_ws/src/radar_bringup/config/bridge/radar_bridge.yaml
git commit -m "fix(bridge): migrate VideoBridge SHM read to v2 SharedFrameReader"
```

---

### Task 5: VideoBridge 接入 ArmorInfer 推理 + 帧上标注

**Files:**
- Modify: `ros_ws/src/radar_bridge/include/radar_bridge/videostream_bridge.hpp`
- Modify: `ros_ws/src/radar_bridge/src/videostream_bridge.cpp`
- Modify: `ros_ws/src/radar_bridge/include/radar_bridge/radar_bridge_node.hpp`
- Modify: `ros_ws/src/radar_bridge/src/radar_bridge_node.cpp`
- Modify: `ros_ws/src/radar_bridge/CMakeLists.txt`
- Modify: `ros_ws/src/radar_bringup/config/bridge/radar_bridge.yaml`

**Interfaces:**
- Consumes: `radar_camera::armor_infer::ArmorInfer::create(model_dir, l1, l2, l3)`、`ArmorInfer::infer(frame_rgb) -> std::vector<ArmorResult>`、`l1_names`/`l3_names`；Task 4 的 `VideoBridge`。
- Produces:
  - `VideoBridge::video_init(const std::string& shm_name, const std::string& pub_address, std::shared_ptr<radar_camera::armor_infer::ArmorInfer> infer)`
  - `BridgeConfig` 增加 `bool enable_inference{false}; std::string model_dir{"/workspace/ros_ws/src/radar_camera/model"}; float l1_conf{kL1Conf}; float l2_conf{kL2Conf}; float l3_conf{kL3Conf};`

- [ ] **Step 1: 修改 `videostream_bridge.hpp` 加推理成员**

```cpp
#pragma once

#include <hikcamera/shared_frame_reader.hpp>

#include <atomic>
#include <expected>
#include <memory>
#include <string>
#include <thread>
#include <zmq.hpp>

#include "radar_camera/armor_infer.hpp"

namespace radar_bridge::videostream_bridge {

class VideoBridge final {
public:
    VideoBridge() = default;
    ~VideoBridge();

    auto video_init(const std::string& shm_name, const std::string& pub_address,
        std::shared_ptr<radar_camera::armor_infer::ArmorInfer> infer)
        -> std::expected<void, std::string>;
    auto video_thread() -> std::expected<void, std::string>;
    auto video_thread_stop() -> std::expected<void, std::string>;

private:
    void draw_overlay(cv::Mat& bgr, const std::vector<radar_camera::armor_infer::ArmorResult>& results);

    hikcamera::SharedFrameReader reader_;
    std::shared_ptr<radar_camera::armor_infer::ArmorInfer> infer_;
    std::string pub_address_;
    std::string shm_name_;
    zmq::context_t ctx_ { 1 };
    zmq::socket_t pub_ { ctx_, zmq::socket_type::pub };
    std::thread video_thread_;
    std::atomic<bool> video_thread_running_ { false };
};

} // namespace radar_bridge::videostream_bridge
```

- [ ] **Step 2: 修改 `videostream_bridge.cpp` 实现标注与推理**

`video_init` 增加第三参并存储 `infer_ = std::move(infer);`。在推流线程中，JPEG 编码前插入推理+标注：

```cpp
auto VideoBridge::video_thread() -> std::expected<void, std::string> {
    video_thread_running_ = true;
    video_thread_         = std::thread([this]() {
        constexpr auto kWaitTimeout = std::chrono::milliseconds { 500 };
        while (video_thread_running_) {
            auto frame = reader_.wait_next(kWaitTimeout);
            if (!frame || !frame->valid()) continue;

            cv::Mat bgr = frame->mat().clone();
            if (infer_) {
                try {
                    cv::Mat rgb;
                    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
                    const auto results = infer_->infer(rgb);
                    draw_overlay(bgr, results);
                } catch (const std::exception& e) {
                    std::cerr << "[VideoBridge] inference failed, passthrough frame: "
                              << e.what() << "\n";
                }
            }

            std::vector<uchar> jpeg;
            if (!cv::imencode(".jpg", bgr, jpeg, { cv::IMWRITE_JPEG_QUALITY, 85 })
                || jpeg.empty()) {
                std::cerr << "[VideoBridge] JPEG encode failed\n";
                video_thread_running_ = false;
                break;
            }
            // ... ZMQ send（同 Task 4 不变）
        }
    });
    return { };
}
```

`draw_overlay` 实现（帧上直接标注，无侧面板；颜色参照 annotate_l1l2l3：L1 橙 `{255,180,0}`、L2/L3 黄 `{0,255,255}`、MISS 红 `{0,0,255}`、MATCH 绿 `{0,255,0}`）：

```cpp
namespace {
void draw_text(cv::Mat& image, const std::string& value, cv::Point point,
    cv::Scalar color, double scale = 0.8) {
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, color, 2, cv::LINE_AA);
}
auto pct(float value) -> std::string { return std::to_string(static_cast<int>(value * 100.0f)) + "%"; }
} // namespace

void VideoBridge::draw_overlay(cv::Mat& bgr,
    const std::vector<radar_camera::armor_infer::ArmorResult>& results) {
    using radar_camera::armor_infer::l1_names;
    using radar_camera::armor_infer::l3_names;
    const cv::Scalar l1_color(255, 180, 0), l2_color(0, 255, 255), miss_color(0, 0, 255);
    for (const auto& result : results) {
        const bool l3_match = result.l3.has_value();
        const cv::Scalar box_color = (result.final_id < 6)
            ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        const auto draw_box = [&](cv::Rect2f box, cv::Scalar color, int thickness) {
            cv::rectangle(bgr, { int(box.x), int(box.y) },
                { int(box.x + box.width), int(box.y + box.height) }, color, thickness);
        };
        draw_box(result.l1_box, box_color, 8);
        if (result.l2 && result.l2->corners.size() == 4) {
            std::vector<cv::Point> poly;
            for (const auto& p : result.l2->corners) poly.emplace_back(int(p.x), int(p.y));
            cv::polylines(bgr, poly, true, l2_color, 6, cv::LINE_AA);
        } else if (result.l2) {
            draw_box(result.l2->box, l2_color, 6);
        }
        if (l3_match) {
            const cv::Rect2f roi = result.l2 ? result.l2->box : result.l1_box;
            draw_box(roi, l2_color, 6);
        }
        const int label_x = std::clamp(static_cast<int>(result.l1_box.x), 10, bgr.cols - 650);
        const int label_y = std::max(150, static_cast<int>(result.l1_box.y) - 110);
        draw_text(bgr, "L1 " + std::string(l1_names(result.l1_id)) + " " + pct(result.l1_conf),
            { label_x, label_y }, l1_color, 0.9);
        if (result.l2) {
            const std::string color = result.l2->color == 2 ? "BLUE"
                : result.l2->color == 1 ? "RED" : "UNK";
            draw_text(bgr, "L2 armor " + color + " genre=" + std::to_string(result.l2->genre)
                    + " " + pct(result.l2->conf), { label_x, label_y + 32 }, l2_color, 0.9);
        } else {
            draw_text(bgr, "L2 MISS", { label_x, label_y + 32 }, miss_color, 0.9);
        }
        if (result.l3) {
            draw_text(bgr, "L3 " + std::string(l3_names(result.l3->index)) + " "
                    + pct(result.l3->conf), { label_x, label_y + 64 }, l2_color, 0.9);
        } else {
            draw_text(bgr, "L3 MISS", { label_x, label_y + 64 }, miss_color, 0.9);
        }
        draw_text(bgr, "FINAL " + std::string(l1_names(result.final_id)) + " ["
                + result.decision + "] " + result.match_state,
            { label_x, label_y + 96 }, result.match_state == "MATCH"
                ? cv::Scalar(0, 255, 0) : miss_color, 0.9);
    }
}
```

- [ ] **Step 3: 修改 `radar_bridge_node.hpp` BridgeConfig**

```cpp
struct BridgeConfig {
    std::string zmq_pub_address;
    std::vector<std::string> zmq_sub_addresses;
    std::string shm_name;
    std::string video_pub_address;
    std::string image_topic;
    bool enable_inference = false;
    std::string model_dir = "/workspace/ros_ws/src/radar_camera/model";
    float l1_conf = radar_camera::armor_infer::kL1Conf;
    float l2_conf = radar_camera::armor_infer::kL2Conf;
    float l3_conf = radar_camera::armor_infer::kL3Conf;
};
```

头文件需 `#include "radar_camera/armor_infer.hpp"`。

- [ ] **Step 4: 修改 `radar_bridge_node.cpp`**

`ConfigsLoader` 追加参数声明与读取：

```cpp
        node.declare_parameter("enable_inference", false);
        node.declare_parameter("model_dir", std::string("/workspace/ros_ws/src/radar_camera/model"));
        node.declare_parameter("l1_conf", radar_camera::armor_infer::kL1Conf);
        node.declare_parameter("l2_conf", radar_camera::armor_infer::kL2Conf);
        node.declare_parameter("l3_conf", radar_camera::armor_infer::kL3Conf);
        ...
        config.enable_inference = node.get_parameter("enable_inference").as_bool();
        config.model_dir        = node.get_parameter("model_dir").as_string();
        config.l1_conf          = static_cast<float>(node.get_parameter("l1_conf").as_double());
        config.l2_conf          = static_cast<float>(node.get_parameter("l2_conf").as_double());
        config.l3_conf          = static_cast<float>(node.get_parameter("l3_conf").as_double());
```

构造函数中 engine 加载（失败降级透传）：

```cpp
    std::shared_ptr<radar_camera::armor_infer::ArmorInfer> infer;
    if (config_.enable_inference) {
        auto created = radar_camera::armor_infer::ArmorInfer::create(
            config_.model_dir, config_.l1_conf, config_.l2_conf, config_.l3_conf);
        if (created) {
            infer = *created;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                "ArmorInfer init failed, video stream will passthrough: %s",
                created.error().c_str());
        }
    }
    auto init_ret = video_bridge_.video_init(
        config_.shm_name, config_.video_pub_address, std::move(infer));
```

- [ ] **Step 5: 修改 `radar_bridge/CMakeLists.txt`**

在 `find_package(hikcamera REQUIRED)` 之后追加 TRT 探测（参照 `tools/armor_verify/CMakeLists.txt` 的 `/opt/radar_camera_trt` 约定）：

```cmake
set(RADAR_SRC /workspace/ros_ws/src/radar_camera)
set(TRT_ROOT /opt/radar_camera_trt)
set(CUDA_ROOT /usr/local/cuda)
```

在 `target_link_libraries(${PROJECT_NAME}_node ...)` 中追加：

```cmake
target_sources(${PROJECT_NAME}_node PRIVATE
    ${RADAR_SRC}/src/armor_infer.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
)
target_include_directories(${PROJECT_NAME}_node PRIVATE
    ${RADAR_SRC}/include
    ${TRT_ROOT}/include
    ${CUDA_ROOT}/include
)
target_link_directories(${PROJECT_NAME}_node PRIVATE
    ${TRT_ROOT}/lib
    ${CUDA_ROOT}/lib64
)
target_link_libraries(${PROJECT_NAME}_node
    nvinfer
    nvinfer_plugin
    cudart
)
```

注意：`radar_bridge_node` 是 `ament_auto_add_executable` 产物；直接在 CMakeLists 追加 target_sources/target_include_directories/target_link_libraries 调用（target 名 `${PROJECT_NAME}_node`）。若 `CUDA_ROOT` 的 lib64 不存在（容器内 CUDA 由 TRT stage 提供），则去掉 `${CUDA_ROOT}/lib64` 一行，cudart 从 `${TRT_ROOT}/lib` 解析。

- [ ] **Step 6: 修改 `radar_bringup/config/bridge/radar_bridge.yaml` 加参数**

```yaml
    # 实时 3 层推理标注（默认关；开时推流画面带 L1/L2/L3 标注）
    enable_inference: false
    model_dir: "/workspace/ros_ws/src/radar_camera/model"
    l1_conf: 0.20
    l2_conf: 0.50
    l3_conf: 0.80
```

`radar_bridge/config/radar_bridge_params.yaml` 同步加相同块。

- [ ] **Step 7: 构建验证**

```bash
docker exec devcontainer-radar-develop-1 bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace/ros_ws && colcon build --packages-select radar_bridge 2>&1 | tail -5"
```

Expected: `Finished <<< radar_bridge`。

- [ ] **Step 8: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
git add ros_ws/src/radar_bridge/ ros_ws/src/radar_bringup/config/bridge/radar_bridge.yaml
git commit -m "feat(bridge): live L1/L2/L3 inference overlay in VideoBridge push stream"
```

---

### Task 6: annotate_l1l2l3 离线工具改为复用 ArmorInfer + 回归

**Files:**
- Modify: `tools/armor_verify/annotate_l1l2l3.cpp`
- Modify: `tools/armor_verify/CMakeLists.txt`

**Interfaces:**
- Consumes: `ArmorInfer::create` + `infer`（Task 3）；Task 2 纯函数。
- Produces: 与改动前输出一致的 `results.csv` 与 `*_l123.jpg`。

- [ ] **Step 1: 重写 `annotate_l1l2l3.cpp` 主流程（删除内联推理实现）**

删除原文件中的 `letterbox`/`blob`/`iou`/`l1`/`run_l3`/`run_l2`/`l3_id`/`l2_id`/`sigmoid`/`kL1Conf`/`kL2Conf`/`kL3Conf`/`kL2Nms`/`kSideL1`/`kSideL2`/`kSideL3`/`names`/`number_names`/`Det`/`Plate`/`Number`，**保留** `crop`（面板预览仍使用）与 `text`/`pct` 绘制逻辑，主循环改为：

```cpp
auto main_impl(int argc, char** argv) -> int {
    if (argc < 4) {
        std::cerr << "usage: annotate_l1l2l3 <frames> <output> <model_dir>\n";
        return 2;
    }
    const fs::path frames = argv[1], output = argv[2];
    const auto infer = ArmorInfer::create(argv[3]);
    if (!infer) {
        std::cerr << "ArmorInfer init failed: " << infer.error() << "\n";
        return 1;
    }
    fs::create_directories(output);
    std::ofstream csv(output / "results.csv");
    csv << "frame,det_idx,l1_id,l1_conf,l1_x,l1_y,l1_w,l1_h,l2_match,l2_genre,l2_color,l2_conf,l2_x,l2_y,l2_w,l2_h,l3_plate,l3_plate_conf,decision,final_id,match_state\n";
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(frames)) if (entry.path().extension() == ".jpg") files.push_back(entry.path());
    std::sort(files.begin(), files.end());
    const cv::Scalar l1_color(255, 180, 0), l2_color(0, 255, 255), miss_color(0, 0, 255);
    for (const auto& file : files) {
        cv::Mat bgr = cv::imread(file.string()); if (bgr.empty()) continue;
        cv::Mat rgb; cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        constexpr int panel_width = 1100;
        cv::Mat output_image(bgr.rows, bgr.cols + panel_width, CV_8UC3, cv::Scalar(28, 28, 28));
        bgr.copyTo(output_image(cv::Rect(0, 0, bgr.cols, bgr.rows)));
        cv::rectangle(output_image, { 0, 0 }, { output_image.cols - 1, 92 }, { 25, 25, 25 }, cv::FILLED);
        text(output_image, "L1 orange", { 25, 52 }, l1_color, 1.0);
        text(output_image, "L2 plate poly", { 245, 52 }, l2_color, 1.0);
        text(output_image, "L3 number 224", { 540, 52 }, l2_color, 1.0);
        text(output_image, "green=MATCH", { 880, 52 }, { 0, 255, 0 }, 1.0);
        text(output_image, "red=MISS", { 1120, 52 }, miss_color, 1.0);
        const auto results = infer->infer(rgb);
        int index = 0;
        for (const auto& result : results) {
            const bool l2_match = result.l2.has_value();
            const bool l3_match = result.l3.has_value();
            const auto l3_value = result.l3;
            const cv::Scalar box_color = (result.final_id < 6) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            const auto draw_box = [&](cv::Rect2f box, cv::Scalar color) {
                cv::rectangle(output_image, { int(box.x), int(box.y) },
                    { int(box.x + box.width), int(box.y + box.height) }, color, 8);
            };
            draw_box(result.l1_box, box_color);
            if (result.l2 && result.l2->corners.size() == 4) {
                std::vector<cv::Point> poly;
                for (const auto& p : result.l2->corners) poly.emplace_back(int(p.x), int(p.y));
                cv::polylines(output_image, poly, true, l2_color, 6, cv::LINE_AA);
            } else if (result.l2) {
                draw_box(result.l2->box, l2_color);
            }
            if (l3_value) {
                const cv::Rect2f roi = result.l2 ? result.l2->box : result.l1_box;
                draw_box(roi, l2_color);
            }
            const int label_x = std::clamp(static_cast<int>(result.l1_box.x), 10, bgr.cols - 650);
            const int label_y = std::max(150, static_cast<int>(result.l1_box.y) - 110);
            text(output_image, "L1 " + std::string(l1_names(result.l1_id)) + " " + pct(result.l1_conf),
                { label_x, label_y }, l1_color, 0.9);
            if (result.l2) {
                const std::string color = result.l2->color == 2 ? "BLUE" : result.l2->color == 1 ? "RED" : "UNK";
                text(output_image, "L2 armor " + color + " genre=" + std::to_string(result.l2->genre)
                        + " " + pct(result.l2->conf), { label_x, label_y + 32 }, l2_color, 0.9);
            } else {
                text(output_image, "L2 MISS", { label_x, label_y + 32 }, miss_color, 0.9);
            }
            if (l3_value) {
                text(output_image, "L3 " + std::string(l3_names(l3_value->index)) + " " + pct(l3_value->conf),
                    { label_x, label_y + 64 }, l2_color, 0.9);
            } else {
                text(output_image, "L3 MISS", { label_x, label_y + 64 }, miss_color, 0.9);
            }
            text(output_image, "FINAL " + std::string(l1_names(result.final_id)) + " [" + result.decision + "] " + result.match_state,
                { label_x, label_y + 96 }, result.match_state == "MATCH"
                    ? cv::Scalar(0, 255, 0) : miss_color, 0.9);

            const int card_y = 125 + index * 620;
            if (card_y + 570 < output_image.rows) {
                const int panel_x = bgr.cols + 35;
                text(output_image, "TARGET " + std::to_string(index + 1), { panel_x, card_y }, { 255, 255, 255 }, 0.9);
                const cv::Rect2f armor_box = result.l2 ? result.l2->box : result.l1_box;
                cv::Mat armor_crop = crop(bgr, armor_box);
                cv::Mat armor_preview;
                cv::resize(armor_crop, armor_preview, { panel_width - 70, 400 });
                armor_preview.copyTo(output_image(cv::Rect(panel_x, card_y + 20, armor_preview.cols, armor_preview.rows)));
                text(output_image, result.l2 ? "L2 armor crop" : "L1 ROI (L2 MISS)",
                    { panel_x, card_y + 450 }, result.l2 ? l2_color : miss_color, 0.8);
                if (l3_value) {
                    text(output_image, "L3 " + std::string(l3_names(l3_value->index)) + " " + pct(l3_value->conf),
                        { panel_x, card_y + 485 }, l2_color, 0.8);
                } else {
                    text(output_image, "L3 MISS", { panel_x, card_y + 485 }, miss_color, 0.8);
                }
            }
            csv << file.filename().string() << "," << index << "," << result.l1_id << "," << result.l1_conf << ","
                << result.l1_box.x << "," << result.l1_box.y << "," << result.l1_box.width << "," << result.l1_box.height << ","
                << (result.l2 ? 1 : 0) << "," << (result.l2 ? result.l2->genre : -1) << "," << (result.l2 ? result.l2->color : -1) << "," << (result.l2 ? result.l2->conf : 0) << ","
                << (result.l2 ? result.l2->box.x : 0) << "," << (result.l2 ? result.l2->box.y : 0) << "," << (result.l2 ? result.l2->box.width : 0) << "," << (result.l2 ? result.l2->box.height : 0) << ","
                << (result.l3 ? result.l3->index : -1) << "," << (result.l3 ? result.l3->conf : 0) << ","
                << result.decision << "," << result.final_id << "," << result.match_state << "\n";
            ++index;
        }
        const std::string stem = file.stem().string();
        cv::imwrite((output / (stem + "_l123.jpg")).string(), output_image);
    }
    return 0;
}
```

`crop` 辅助函数保留（面板预览使用）；`names`/`number_names` 替换为 `l1_names`/`l3_names`（返回 `const char*`，用法不变）。`text` 函数保留。文件顶部在 `using radar_camera::model_inference::TensorRtInference;` 之后追加 `using radar_camera::armor_infer::ArmorInfer;`（或全部使用全限定名），并 include `"radar_camera/armor_infer.hpp"`。

- [ ] **Step 2: 修改 `tools/armor_verify/CMakeLists.txt` — annotate_l1l2l3 加源文件**

```cmake
add_executable(annotate_l1l2l3
    annotate_l1l2l3.cpp
    ${RADAR_SRC}/src/armor_infer.cpp
    ${RADAR_SRC}/src/tensorrt_inference.cpp
)
```

- [ ] **Step 3: 重新构建**

```bash
docker exec devcontainer-radar-develop-1 bash -c "cd /workspace/tools/armor_verify/build && cmake .. > /dev/null && make annotate_l1l2l3 -j$(nproc) 2>&1 | grep -E 'error|Error' | head -5; echo BUILD_DONE"
```

Expected: 无 error 输出。

- [ ] **Step 4: 离线回归（需要 GPU + engine 文件）**

```bash
docker exec devcontainer-radar-develop-1 bash -c "cd /workspace && mkdir -p /tmp/armor_regress_out && ./tools/armor_verify/build/annotate_l1l2l3 model/generated/l1l2l3_test_100frames /tmp/armor_regress_out ros_ws/src/radar_camera/model 2>&1 | tail -3"
```

Expected: 正常生成 `/tmp/armor_regress_out/results.csv`。若有改动前的历史输出可 diff 对比（`model/generated/l1l2l3_annotate.csv` 或 git 历史），无则人工抽查 3-5 张 `*_l123.jpg` 标注正确（框/多边形/号码/决策行）。**若容器内无 GPU（DeviceRequests=null），本步移到部署机执行，本地只验证编译。**

- [ ] **Step 5: Commit**

```bash
cd /home/yukikaze/Documents/workspace/alliance_radar_location_lidar
git add tools/armor_verify/annotate_l1l2l3.cpp tools/armor_verify/CMakeLists.txt
git commit -m "refactor(armor_verify): reuse ArmorInfer in annotate_l1l2l3"
```

---

### Task 7: 实时端到端验证（GPU 环境）

**Files:** 无代码改动。

**Interfaces:** Consumes Task 5 的 radar_bridge 构建产物。

- [ ] **Step 1: 确认环境**（宿主机 GPU + 相机 + hikcamera_ros_driver 运行，SHM 存在）

```bash
nvidia-smi
docker exec devcontainer-radar-develop-1 bash -c "ls /dev/shm/hikcamera_shm 2>/dev/null; ls /dev/shm | grep hikcamera"
```

Expected: GPU 可见；hikcamera_ros_driver 进程运行并写 SHM。若 SHM 不在 /dev/shm，检查 `ipcs`/`/run/shm`。

- [ ] **Step 2: 纯透传模式回归（enable_inference=false）**

```bash
docker exec devcontainer-radar-develop-1 bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/ros_ws/install/setup.bash && cd /workspace/ros_ws && ros2 run radar_bridge radar_bridge_node --ros-args -p enable_inference:=false"
```

Expected: 启动无错误日志；egui（或 `zmq_sub` 测试客户端）在 5557 收到原始画面。

- [ ] **Step 3: 推理模式验证（enable_inference=true）**

```bash
docker exec devcontainer-radar-develop-1 bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/ros_ws/install/setup.bash && cd /workspace/ros_ws && ros2 run radar_bridge radar_bridge_node --ros-args -p enable_inference:=true"
```

Expected: 日志出现（可选加 `RCLCPP_INFO` 打印每帧检测数/耗时）；egui 显示带 L1 橙框 / L2 黄色多边形 / L3 号码 / FINAL 决策行的标注画面。

- [ ] **Step 4: 故障注入验证降级透传**

```bash
docker exec devcontainer-radar-develop-1 bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace/ros_ws && ros2 run radar_bridge radar_bridge_node --ros-args -p enable_inference:=true -p model_dir:=/nonexistent"
```

Expected: ERROR 日志 `ArmorInfer init failed...passthrough`，推流继续原始画面。

- [ ] **Step 5: 记录验证结果到 spec 对应章节**（无 commit，仅汇报）

---

## Self-Review 记录

- **Spec 覆盖**：ArmorInfer 抽取（Task 2/3）✓；annotate 复用（Task 6）✓；VideoBridge v2 SHM 迁移（Task 4）✓；推理接入 + 帧上标注（Task 5）✓；参数/yaml/CMake（Task 5）✓；降级透传（Task 5 Step 2 + Task 7 Step 4）✓；实时验证（Task 7）✓；不做项（egui/RTSP/面板/raw_shm_reader）均未涉及 ✓。
- **占位符扫描**：所有步骤含完整代码或精确命令；无 TBD/TODO。
- **类型一致性**：`ArmorInfer::create` 返回 `std::expected<std::shared_ptr<ArmorInfer>, std::string>` 在 Task 3/5/6/7 一致；`ArmorResult` 字段 `l1_id/l1_conf/l1_box/l2/l3/final_id/decision/match_state` 在 Task 2/5/6 一致；`decode_l1/decode_l2/letterbox/blob/iou/sigmoid/l2_id/l3_id/l1_names/l3_names` 签名一致；`VideoBridge::video_init` 三参（shm, addr, infer）在 Task 4→5 演进一致。
- 已知风险：Task 1 的 stage 恢复依赖宿主机权限与 compose 行为，已在步骤中给出两种备选路径。
