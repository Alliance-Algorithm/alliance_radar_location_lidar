# VideoBridge 实时 3 层装甲检测标注推流

日期: 2026-08-01

## 背景

现有 3 层检测管线（L1 装甲 YOLO 1280 → L2 灯条多边形 640 → L3 号码分类 224）已在
`tools/armor_verify/annotate_l1l2l3.cpp`（TensorRT）和 `l1l2l3_verify.py`（onnxruntime CPU）
实现，但只支持**离线图片文件夹**：输入 `*.jpg` 帧目录，输出带注释的图片 + CSV。

用户需要同样的 3 层标注**实时作用在相机推流上**：相机图像（而非照片）→ 检测标注 →
继续推流给 egui，端口不变（5557），egui 零改动。

## 探索发现的关键事实

1. **3 层模型文件已存在**（`ros_ws/src/radar_camera/model/`）：
   - `best_fixed_names_1280_fp16.engine`（L1，[1,300,6] xyxy+conf+cls，12 类 red-first）
   - `shenzhen-0708_fp16.engine`（L2，[1,25200,22] 4 角点 + obj + color + genre）
   - `armor-number_fp16.engine`（L3，[1,9] softmax B1..BS,R1..R4）
   - 同名 `.onnx` 亦存在。

2. **VideoBridge 已实现推流**（`ros_ws/src/radar_bridge/src/videostream_bridge.cpp`）：
   SHM 读帧 → `cvtColor RGB2BGR` → JPEG(85) → ZMQ PUB (conflate=1) 绑定 `tcp://*:5557`。

3. **⚠️ 当前 radar_bridge 编译失败**：`videostream_bridge.cpp`/`videostream_bridge.hpp` 使用
   v1 SHM API（`hikcamera::imageSHM`、`sem_timedwait`、`SHMGetPtr`、`SHMInit(name, size)`），
   而 `hikcamera_sdk` 已升级 v2 协议：`SharedFrameReader`（`open(name)` + `wait_next()` 返回
   `SharedFrame`，含 `mat()`/`metadata()`/`sequence()`），`imageSHM` 类型已不存在。
   实测 `colcon build --packages-select radar_bridge` 报
   `'imageSHM' in namespace 'hikcamera' does not name a type`。

4. **SHM v2 协议要点**（`hikcamera_sdk/include/hikcamera/shm_types.hpp`）：
   - 4-slot ring，`SharedRingHeader` + `FrameSlot slots[4]`，`latest_sequence` 原子发布；
   - 像素格式 **RGB8**（写端输出 RGB8，推流端 `cvtColor RGB2BGR`，与现状代码一致）；
   - 帧宽高由 `FrameMetadata.width/height/stride_bytes` 携带，不再依赖进程参数
     （radar_bridge.yaml 中 `video_width: 4096 / video_height: 3000` 已无意义，CS200-10UC 为 5472×3648）。

5. **TensorRT 构建环境**：宿主机有 `/usr/lib/libnvinfer.so.11`、`/opt/cuda`、`trtexec`、RTX 4060 8GB。
   容器通过 bind mount `${RADAR_TENSORRT_STAGE:-/tmp/radar_camera_trt_host}:/opt/radar_camera_trt:ro`
   获得 TRT 头文件/库（由 `.script/prepare-radar-camera-tensorrt-runtime` 生成）。
   ⚠️ 当前 `/tmp/radar_camera_trt_host` 为空目录（root 所有），需先恢复 stage 才能编译。

## 目标

在现有推流主链路中插入实时 3 层推理与帧上标注：

- 输入：`/hikcamera_shm`（v2 协议，RGB8，5472×3648）
- 推理：L1→L2→L3 全管线，每帧全推理（复用 `annotate_l1l2l3.cpp` 的推理逻辑）
- 标注：帧上直接绘制（不加右侧面板）
- 输出：JPEG(85) + ZMQ PUB conflate 绑 `tcp://*:5557`（端口不变）
- 失败降级：推理异常或引擎加载失败 → 透传原始帧，推流永不断
- 默认关闭：`enable_inference: false` 时行为与现状一致（修复编译后为纯透传）

## 架构

```
hikcamera_ros_driver ──写──> /hikcamera_shm (v2 ring)
                                │
                        VideoBridge 推流线程 (每帧)
                                │ 1. SharedFrameReader::wait_next() → RGB8 Mat
                                │ 2. [enable_inference=on] L1→L2→L3 推理 + 画标注
                                │ 3. RGB2BGR → JPEG(85) → ZMQ PUB tcp://*:5557
                                ▼
                             egui (零改动)
```

## 组件与改动

| 组件 | 状态 | 内容 |
|---|---|---|
| `include/radar_camera/armor_infer.hpp` + `src/armor_infer.cpp`（新，radar_camera 包内） | 新增 | 从 `tools/armor_verify/annotate_l1l2l3.cpp` 抽出推理逻辑为可复用单元：`ArmorInfer` 类（构造时加载 3 个 engine；`infer(frame_rgb) -> vector<ArmorResult>`），含 letterbox/blob/iou/l1/run_l2/run_l3/l2_id/l3_id 逻辑与阈值常量 |
| `tools/armor_verify/annotate_l1l2l3.cpp` | 改为复用 | 删除内联推理实现，改用 `ArmorInfer`（保持离线标注功能与输出不变） |
| `videostream_bridge.hpp/cpp`（radar_bridge） | 修改 | SHM 读取迁移 v2 `SharedFrameReader`；新增推理步骤 + 标注绘制（`draw_overlay`）；失败透传 |
| `radar_bridge_node.hpp/cpp` | 修改 | 新参数 `enable_inference`（默认 false）、`model_dir`（默认 `/workspace/ros_ws/src/radar_camera/model`）、`l1_conf`/`l2_conf`/`l3_conf`（沿用 annotate 阈值 0.20/0.50/0.80）；移除 `video_width`/`video_height` 参数 |
| `radar_bridge/CMakeLists.txt` | 修改 | 链接 `nvinfer`/`cudart`/`cublas`/`cublasLt` + `armor_infer.cpp` + `tensorrt_inference.cpp`（参照 `tools/armor_verify/CMakeLists.txt`） |
| `radar_bringup/config/bridge/radar_bridge.yaml` | 修改 | 加 `enable_inference`、`model_dir`；删 `video_width`/`video_height` |

## 数据流与接口

### ArmorInfer

```cpp
struct ArmorResult {
    int l1_id;               // 12 类 id (0-11, red-first)
    float l1_conf;
    cv::Rect2f l1_box;
    std::optional<Plate> l2; // 角点/box/genre/color/conf
    std::optional<Number> l3;// index 0..8, conf
    int final_id;
    std::string decision;    // "L1"/"L2"/"L3-plate"
    std::string match_state; // "MATCH"/"MISS"（无人机恒 MATCH）
};

class ArmorInfer {
public:
    // 任一 engine 加载失败 → 返回错误（调用方决定降级透传）
    static auto create(const std::string& model_dir,
                       float l1_conf, float l2_conf, float l3_conf)
        -> std::expected<ArmorInfer, std::string>;
    auto infer(const cv::Mat& frame_rgb) -> std::vector<ArmorResult>;
};
```

推理逻辑、阈值、NMS、id 映射、无人机跳过 L2/L3 均与 `annotate_l1l2l3.cpp` 保持一致
（**不改算法，只搬位置**）。

### VideoBridge 改动

- 构造/init 时接收 `shared_ptr<ArmorInfer>`（可为 null → 纯透传模式）；
- 推流线程每帧：`reader.wait_next()` → 若 infer 非空：`infer(frame)` + `draw_overlay(frame_bgr, results)`；
- 推理抛异常 → 打印日志，使用原帧继续推流；
- `draw_overlay` 绘制（帧上直接标注，无侧面板）：
  - L1 框：按 final_id 颜色（0-5 红 / 6-11 蓝），粗框；
  - L2：4 角点黄色多边形（无角点则黄框）；
  - L3：plate 区域黄框；
  - 文本行（label_x, label_y 依次下移）：
    - `L1 <name> <conf%>`（橙）
    - `L2 armor <RED|BLUE> genre=<g> <conf%>` 或 `L2 MISS`（黄/红）
    - `L3 <number> <conf%>` 或 `L3 MISS`（黄/红）
    - `FINAL <name> [<decision>] <MATCH|MISS>`（按 MATCH 颜色 / 红）
- SHM 读取：`hikcamera::SharedFrameReader reader; reader.open(shm_name); auto f = reader.wait_next(500ms);` 跳过无效帧；用 `f.mat()`（RGB8）与 `f.metadata()` 的宽高构造 cv::Mat（注意 `stride_bytes` 可能大于 width*3）。

## 错误处理

| 场景 | 行为 |
|---|---|
| enable_inference=on，engine 加载失败 | ERROR 日志，`infer=nullptr` 降级透传 |
| 单帧推理异常 | ERROR 日志，透传该帧 |
| SHM 打开失败 / 超时 | 与现状一致（wait_next 有 2s 默认超时，循环继续） |
| 推流失败（JPEG/ZMQ） | 与现状一致（停线程） |

## 测试

1. **单元测试（可选）**：`ArmorInfer` 纯逻辑可测——以 `model/generated/l1l2l3_test_100frames` 中的少量帧跑通，比对 `results.csv` 输出与 `annotate_l1l2l3` 一致。
2. **编译验证**：`colcon build --packages-select radar_bridge radar_camera`。
3. **离线回放验证**：用已录制的相机帧目录（或 `model/generated/l1l2l3_annotate` 输入帧）跑修复后的 `annotate_l1l2l3`，确认输出与改动前一致（回归）。
4. **实时验证**：宿主机准备 TRT stage（`RADAR_TENSORRT_STAGE` 指向新目录或修复 `/tmp/radar_camera_trt_host` 权限）→ 容器内 `enable_inference: true` 启动 radar_bridge → egui 连接 5557 观察标注画面；`enable_inference: false` 验证纯透传回归。

## 不做的事（YAGNI）

- 不改 egui、不改 5557 端口、不加 RTSP 推流、不做视频录制、不加右侧信息面板；
- 不做多相机、不做多目标跟踪（帧间关联）；
- 不迁移 `raw_shm_reader.cpp`（radar_camera 录制侧，不在本需求范围）。
