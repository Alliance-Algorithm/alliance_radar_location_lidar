# 坐标完全信相机（点云仅配准）实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 官方坐标 100% 来自相机确认 track；雷达（点云）只做 GICP 配准，fusion 与 radar_lidar 的聚类路径全部停用。

**Architecture:** 删除 radar_fusion 的 `/lidar/cluster` 订阅与 `lidar_tracks_` 独立池（含相机贴类别循环），`publish_lidar_location` 只填相机池；radar_lidar 新增 `enable_cluster` 参数（默认 false），关闭时跳过聚类处理、不发布 `/lidar/cluster`。

**Tech Stack:** C++20 / ROS 2 Jazzy / gtest / colcon（容器 `devcontainer-radar-develop-1`，挂载 `/workspace`）

**Spec:** `docs/superpowers/specs/2026-08-05-camera-only-location-design.md`

## Global Constraints

- 所有源码改动在 worktree `/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.worktrees/camera-only`（分支 `feat/camera-only`）完成
- 代码风格：clang-format（CI 检查 `clang-format --dry-run --Werror`，格式文件由仓库 `.clang-format` 定义）
- 不改动：相机链路（radar_camera）、GICP 定位（localization_stage）、默认位置兜底、`update_fusion_mode`
- 构建/测试在容器内执行（worktree 映射为容器内 `/workspace/.worktrees/camera-only`）
- 构建命令模板（容器内）：
  ```bash
  docker exec devcontainer-radar-develop-1 bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/ros_ws/install/setup.bash && cd /workspace/.worktrees/camera-only/ros_ws && colcon build --packages-select radar_fusion radar_lidar --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-dev'
  ```
- 测试命令模板（容器内，worktree 的 build 完成后）：
  ```bash
  docker exec devcontainer-radar-develop-1 bash -lc 'source /opt/ros/jazzy/setup.bash && source /workspace/.worktrees/camera-only/ros_ws/install/setup.bash && cd /workspace/.worktrees/camera-only/ros_ws && ROS_DOMAIN_ID=123 colcon test --packages-select radar_fusion radar_lidar --event-handlers console_direct+'
  ```
  （radar_lidar 测试要求 `ROS_DOMAIN_ID` 环境变量在 0-232 区间）

---

### Task 1: fusion 测试先导——删 cluster 用例、改写相机路径、新增红用例

**Files:**
- Modify: `ros_ws/src/radar_fusion/test/test_fusion_node.cpp`

**Interfaces:**
- Consumes: 现有 fixture 成员 `cluster_pub_`、`camera_detection_pub_`、`make_cluster_msg`、`make_camera_detection`、`make_camera_slot`、`make_empty_camera_detection`、`wait_for_*_gen`
- Produces: 新用例 `CameraTaggedClusterNoLongerFillsLocation`（Task 2 实现后从红转绿）

- [ ] **Step 1: `wait_for_discovery` 去掉 cluster 订阅检查**

`test_fusion_node.cpp` 的 `wait_for_discovery`（约 151-165 行）当前条件：
```cpp
if (cluster_pub_->get_subscription_count() > 0
    && lidar_pose_pub_->get_subscription_count() > 0
```
改为：
```cpp
if (lidar_pose_pub_->get_subscription_count() > 0
```
（cluster 订阅删除后 `get_subscription_count()` 恒 0，会导致所有用例 discovery 超时）

- [ ] **Step 2: 删除 6 个 cluster 依赖用例**

删除以下 `TEST_F`（整段，含函数体）：
- `ClusterOnlyInputPublishesStatusWithoutLocalizationPose`（约 309-317）
- `ClusterTrackingUsesMessageTimeInsteadOfWallTime`（约 319-347）
- `CameraDetectionNearLidarTrackKeepsFusedOutputActive`（约 531-558）
- `EmptyCameraFramesDoNotDeleteLidarTrack`（约 560-591）
- `LidarClusterInheritsCameraClassAndOutputsLocation`（约 858-892）
- `LidarClusterClassPersistsAfterCameraStops`（约 894-922，文件末尾）

- [ ] **Step 3: 改写 4 个 track 生命周期用例为相机路径**

每个用例开头插入与 `CameraDetectionsCreateConfirmedTracksWhenFusionEnabled`（约 506-513）相同的节点替换 4 行（enable_camera_fusion=true + wait_for_discovery(true)）。

**3a. `TentativeTrackIsDroppedAfterSingleMiss` → `CameraTentativeTrackIsDroppedAfterTimeout`**（原 349-363）：
```cpp
    auto bl = track_pub_gen_;

    camera_detection_pub_->publish(make_camera_detection(0.0, 0.0, 0, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 1, 500ms));

    // tentative（1 hit）无后续输入 → track 超时（1.5s）删除
    std::this_thread::sleep_for(1700ms);
    ASSERT_FALSE(wait_for_track_pub_gen(bl + 2, 200ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_LT(last_track_marker_count_, 3u);
    }
```

**3b. `ConfirmedTrackSurvivesSingleMissButDropsAfterSecondMiss` → `CameraConfirmedTrackSurvivesGapButDropsAfterTimeout`**（原 365-396）：
```cpp
    auto bl = track_pub_gen_;

    camera_detection_pub_->publish(make_camera_detection(0.0, 0.0, 0, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 1, 500ms));

    camera_detection_pub_->publish(make_camera_detection(0.4, 0.0, 1, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 2, 500ms));

    camera_detection_pub_->publish(make_camera_detection(0.8, 0.0, 2, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 3, 500ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_GE(last_track_marker_count_, 3u);
    }

    // 空帧（相机路径不 mark_missed）→ 确认 track 保留
    auto bl2 = track_pub_gen_;
    camera_detection_pub_->publish(make_empty_camera_detection(2, 100000000u));
    camera_detection_pub_->publish(make_empty_camera_detection(2, 200000000u));
    std::this_thread::sleep_for(500ms);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_EQ(last_track_marker_count_, 3u);
    }

    // 超时后删除（track_timeout 1.5s）
    std::this_thread::sleep_for(1500ms);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_EQ(last_track_marker_count_, 0u);
    }
```

**3c. `GlobalGreedyAssociationKeepsTwoConfirmedTracks` → `CameraGlobalAssociationKeepsTwoConfirmedTracks`**（原 398-421，双 class 交叉验证）：
```cpp
    auto bl = track_pub_gen_;

    // 每帧 hero(class 0) + inf3(class 2) 两个槽位
    camera_detection_pub_->publish(make_camera_slot(0.0, 0.0, 0, 0, 0u));
    camera_detection_pub_->publish(make_camera_slot(1.5, 0.0, 2, 0, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 1, 500ms));

    camera_detection_pub_->publish(make_camera_slot(0.4, 0.0, 0, 1, 0u));
    camera_detection_pub_->publish(make_camera_slot(1.9, 0.0, 2, 1, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 2, 500ms));

    camera_detection_pub_->publish(make_camera_slot(0.8, 0.0, 0, 2, 0u));
    camera_detection_pub_->publish(make_camera_slot(2.3, 0.0, 2, 2, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 3, 500ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_GE(last_track_marker_count_, 6u);
    }

    // 位置交叉 → 同 class 匹配保持各自 track
    auto bl2 = track_pub_gen_;
    camera_detection_pub_->publish(make_camera_slot(1.2, 0.0, 0, 3, 0u));
    camera_detection_pub_->publish(make_camera_slot(1.1, 0.0, 2, 3, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl2 + 1, 500ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_EQ(last_track_marker_count_, 6u);
    }
```

**3d. `ConfirmedTracksArePublishedToFusedTracks` → `CameraConfirmedTracksArePublishedToFusedTracks`**（原 455-472）：
```cpp
    auto bl_track = track_pub_gen_;
    auto bl_fused = fused_track_pub_gen_;

    camera_detection_pub_->publish(make_camera_detection(0.0, 0.0, 0, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl_track + 1, 500ms));

    camera_detection_pub_->publish(make_camera_detection(0.4, 0.0, 1, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl_track + 2, 500ms));

    camera_detection_pub_->publish(make_camera_detection(0.8, 0.0, 2, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl_track + 3, 500ms));
    ASSERT_TRUE(wait_for_fused_track_pub_gen(bl_fused + 3, 500ms));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GE(last_track_marker_count_, 3u);
    EXPECT_EQ(last_fused_track_marker_count_, 1u);
```

- [ ] **Step 4: 新增红用例**（文件末尾，相机贴类别后聚类不得再填坐标）：

```cpp
TEST_F(FusionNodeTest, CameraTaggedClusterNoLongerFillsLocation) {
    // 雷达聚类确认 track（无类别本就不填坐标）
    cluster_pub_->publish(make_cluster_msg(0.5, 0.0, 0.0, 0, 0u));
    cluster_pub_->publish(make_cluster_msg(1.0, 0.0, 0.0, 1, 0u));
    cluster_pub_->publish(make_cluster_msg(1.5, 0.0, 0.0, 2, 0u));
    const auto track_bl = track_pub_gen_;
    ASSERT_TRUE(wait_for_track_pub_gen(track_bl + 1, 1s)) << "lidar track not confirmed";

    // 相机单帧贴类别（class 2=inf3）：相机 track 仅 1 hit（tentative，不进坐标）。
    // 旧行为会给 lidar track 贴类别并填 inf3 槽位 —— 删除贴类别后不应再填。
    camera_detection_pub_->publish(make_camera_slot(1.0, 0.0, 2, 3, 0u));
    std::this_thread::sleep_for(600ms); // 覆盖 10Hz 输出周期 + 处理时延

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_location_.opponent_infantry_3_x, 0);
    EXPECT_EQ(last_location_.opponent_infantry_3_y, 0);
}
```

- [ ] **Step 5: 构建 + 跑 radar_fusion 测试，确认新用例 RED 且其余 GREEN**

Run: 构建命令 + 测试命令（见 Global Constraints）
Expected: `CameraTaggedClusterNoLongerFillsLocation` **FAILED**（旧行为贴类别后填了 inf3 槽位），其余用例通过

- [ ] **Step 6: Commit**

```bash
git add ros_ws/src/radar_fusion/test/test_fusion_node.cpp
git commit -m "test(fusion): drop lidar cluster test cases, move lifecycle tests to camera path"
```

---

### Task 2: fusion 源码删除聚类路径

**Files:**
- Modify: `ros_ws/src/radar_fusion/src/radar_fusion_node.cpp`
- Modify: `ros_ws/src/radar_fusion/include/radar_fusion/radar_fusion_node.hpp`

**Interfaces:**
- Consumes: Task 1 的 fixture/用例不变（`cluster_pub_` 保留供 Task 1 新用例使用）
- Produces: `publish_lidar_location` 新签名 `void publish_lidar_location(const std::vector<KalmanTracker>& tracks)`（单参）

- [ ] **Step 1: 删除 `radar_fusion_node.cpp` 中的聚类路径**

按以下清单逐项删除：

1. 构造函数（约 70-71 行）：删除
   ```cpp
   sub_cluster_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar/cluster", 10,
       [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_cluster(msg); });
   ```
2. location_timer_ 回调（约 114 行）：删除 `publish_lidar_tracks(lidar_tracks_, stamp);`
3. `on_camera_detection` 中的贴类别循环（约 171-193 行，注释 `// 相机识别给雷达聚类 track 贴类别` 整块，含 `if (!lidar_tracks_.empty()) { ... }`）
4. `on_cluster` 整个函数（约 257-277 行）
5. `process_lidar_clusters` 整个函数（约 279 行起，至 `publish_lidar_tracks` 之前）
6. `publish_lidar_tracks` 整个函数（约 430-460 行，至 `publish_fused_tracks` 之前）
7. `publish_lidar_location`（约 539-581 行）：
   - 签名改为 `void RadarFusionNode::publish_lidar_location(const std::vector<KalmanTracker>& tracks)`
   - 删除 `const std::vector<...>& lidar_tracks` 第二参数
   - 删除 `fill_slots(lidar_tracks);`（约 581 行）
   - 删除注释中"雷达池（继承相机类别）后填覆盖"的说明（566 行注释改为说明坐标只来自相机池）

- [ ] **Step 2: 删除 `radar_fusion_node.hpp` 对应声明**

按以下清单删除：
1. `void on_cluster(sensor_msgs::msg::PointCloud2::SharedPtr msg);`（约 35 行）
2. `void process_lidar_clusters(const std::vector<Eigen::Vector2d>& measurements, int64_t now_ns);` 及其上方注释（约 37-38 行）
3. `void publish_lidar_tracks(...)` 声明（约 45-47 行）
4. `publish_lidar_location` 签名改为单参 `tracks`（约 48-50 行）
5. 成员 `std::vector<KalmanTracker> lidar_tracks_;`（约 70 行）
6. 成员 `int next_lidar_track_id_ = 0;`（约 74 行）
7. 成员 `rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cluster_;`（约 77 行）

若删除后 `#include <sensor_msgs/msg/point_cloud2.hpp>` 不再使用，一并删除（检查 `on_lidar_pose`/`process_measurements` 是否用到 sensor_msgs 类型——`process_lidar_clusters` 是唯一使用点，删函数后无引用则移除 include）。

- [ ] **Step 3: 构建 + 跑 radar_fusion 测试，确认全绿**

Run: 构建命令 + 测试命令
Expected: 全部通过（含 Task 1 新增的 `CameraTaggedClusterNoLongerFillsLocation`，现为 GREEN）

- [ ] **Step 4: 检查残留引用**

Run: `rg -n "lidar_tracks|sub_cluster|on_cluster|process_lidar_clusters|publish_lidar_tracks" ros_ws/src/radar_fusion/`
Expected: 无匹配（test_fusion_node.cpp 中仅剩 Task 1 用例里的 `cluster_pub_` 发布调用，允许存在）

- [ ] **Step 5: Commit**

```bash
git add ros_ws/src/radar_fusion
git commit -m "feat(fusion): camera-only location output, drop lidar cluster path"
```

---

### Task 3: radar_lidar 新增 enable_cluster 开关（默认关闭）

**Files:**
- Modify: `ros_ws/src/radar_lidar/test/test_radar_lidar_node.cpp`
- Modify: `ros_ws/src/radar_lidar/include/radar_lidar/radar_lidar_node.hpp`
- Modify: `ros_ws/src/radar_lidar/src/radar_lidar_node.cpp`
- Modify: `ros_ws/src/radar_lidar/config/runtime.yaml`

**Interfaces:**
- Consumes: 现有 fixture `RadarLidarSurfaceTest`（SetUp 中 `opts` 构造节点）；常量 `kClusterTopic`、`kClusterVizTopic`
- Produces: 参数 `enable_cluster`（bool，默认 false）；`RadarLidarNode::enable_cluster_` 成员

- [ ] **Step 1: 测试先导——SetUp 显式开启 cluster + 新增默认关闭用例**

在 `test_radar_lidar_node.cpp` SetUp 的 `opts`（约 159-171 行）末尾追加：
```cpp
        opts.append_parameter_override("enable_cluster", true);
```
（保持现有 `OutputTopicsPreservedWithExactQoS` 的 cluster topic 断言不变）

新增用例（放 `OutputTopicsPreservedWithExactQoS` 之后）：
```cpp
TEST_F(RadarLidarSurfaceTest, ClusterTopicsAbsentWhenDisabled) {
    rclcpp::NodeOptions opts;
    opts.automatically_declare_parameters_from_overrides(true);
    opts.append_parameter_override("map_path", map_path_);
    opts.append_parameter_override("scan_topic", scan_topic_);
    opts.append_parameter_override("use_odin_relocalization_tf", false);
    opts.append_parameter_override("hardware_id", std::string(kHardwareId));
    opts.append_parameter_override("initial_pose_enabled", true);
    opts.append_parameter_override("initial_pose_tx", 0.3);
    opts.append_parameter_override("initial_pose_ty", 0.0);
    opts.append_parameter_override("initial_pose_tz", 1.0);
    opts.append_parameter_override("initial_pose_roll", 0.0);
    opts.append_parameter_override("initial_pose_pitch", 0.0);
    opts.append_parameter_override("initial_pose_yaw", 0.0);
    opts.append_parameter_override("enable_cluster", false);
    auto node = std::make_shared<radar_lidar::node::RadarLidarNode>(opts);

    const auto topics = node->get_topic_names_and_types();
    std::map<std::string, std::string> actual;
    for (const auto& [name, types] : topics) {
        if (!types.empty()) actual[name] = types[0];
    }
    EXPECT_EQ(actual.count(kClusterTopic), 0u)
        << kClusterTopic << " should not exist when enable_cluster=false";
    EXPECT_EQ(actual.count(kClusterVizTopic), 0u)
        << kClusterVizTopic << " should not exist when enable_cluster=false";
}
```

- [ ] **Step 2: 构建 + 跑 radar_lidar 测试，确认新用例 RED**

Run: 构建命令 + 测试命令
Expected: `ClusterTopicsAbsentWhenDisabled` **FAILED**（当前无 `enable_cluster` 参数，节点无条件发布 cluster topic）；其余用例通过

- [ ] **Step 3: 实现 `enable_cluster` 参数**

`radar_lidar_node.hpp`：在 `detection_enabled_` 附近新增成员：
```cpp
    bool enable_cluster_ = false;
```

`radar_lidar_node.cpp` 构造函数（`get_parameter("hardware_id", ...)` 附近，约 77 行后）：
```cpp
    get_parameter_or("enable_cluster", enable_cluster_, false);
```

- [ ] **Step 4: 条件创建 cluster 发布器**

`radar_lidar_node.cpp` 构造函数（约 143-145 行）：
```cpp
    pub_clusters_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/cluster", 10);
    pub_cluster_viz_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar/cluster_viz", 10);
```
改为：
```cpp
    if (enable_cluster_) {
        pub_clusters_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/cluster", 10);
        pub_cluster_viz_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar/cluster_viz", 10);
    }
```
（成员 `pub_clusters_`/`pub_cluster_viz_` 声明保留，用 `std::shared_ptr` 空指针表示关闭；`publish_clusters` 内已有空指针保护则无需改——若没有，`on_scan` 调用处已用 `enable_cluster_` 短路）

- [ ] **Step 5: on_scan 短路聚类处理**

`radar_lidar_node.cpp` `on_scan` 检测段（约 249-262 行）：
```cpp
    if (detection_enabled_) {
        types::PointCloud scan_in_map;
        transform_scan_to_map(frame.points, *pose, scan_in_map);

        auto dynamic_result = dynamic_stage_.process(scan_in_map);
        if (dynamic_result && !dynamic_result->empty()) {
            publish_dynamic(*dynamic_result, frame.stamp);

            auto cluster_result = cluster_stage_.process(*dynamic_result);
            if (cluster_result && !cluster_result->empty()) {
                publish_clusters(*cluster_result, frame.stamp);
            }
        }
    }
```
改为（cluster 处理与发布整体短路；dynamic 保留）：
```cpp
    if (detection_enabled_) {
        types::PointCloud scan_in_map;
        transform_scan_to_map(frame.points, *pose, scan_in_map);

        auto dynamic_result = dynamic_stage_.process(scan_in_map);
        if (dynamic_result && !dynamic_result->empty()) {
            publish_dynamic(*dynamic_result, frame.stamp);

            // 坐标只信相机（点云仅配准）：聚类停用，节省每帧聚类计算
            if (enable_cluster_) {
                auto cluster_result = cluster_stage_.process(*dynamic_result);
                if (cluster_result && !cluster_result->empty()) {
                    publish_clusters(*cluster_result, frame.stamp);
                }
            }
        }
    }
```

- [ ] **Step 6: runtime.yaml 增加开关**

`radar_lidar/config/runtime.yaml`（`hardware_id` 后、`use_odin_relocalization_tf` 附近）：
```yaml
    # 坐标只信相机（点云仅配准）：聚类停用，不发布 /lidar/cluster
    enable_cluster: false
```

- [ ] **Step 7: 构建 + 跑 radar_lidar 测试，确认全绿**

Run: 构建命令 + 测试命令
Expected: 全部通过（`ClusterTopicsAbsentWhenDisabled` 现为 GREEN；SetUp 显式 `enable_cluster=true` 的用例不变）

- [ ] **Step 8: Commit**

```bash
git add ros_ws/src/radar_lidar
git commit -m "feat(radar_lidar): cluster processing off by default (camera-only coordinates)"
```

---

### Task 4: 全量验证 + clang-format + 推送

**Files:**
- 无新增文件

- [ ] **Step 1: 全量构建 + 全量测试**

Run: 构建命令 + 测试命令（radar_fusion、radar_lidar 两包）
Expected: 全部通过

- [ ] **Step 2: clang-format 检查**

```bash
docker exec devcontainer-radar-develop-1 bash -lc 'cd /workspace/.worktrees/camera-only && find ros_ws/src/radar_fusion ros_ws/src/radar_lidar -type f \( -name "*.cpp" -o -name "*.hpp" \) -print0 | xargs -0 clang-format --dry-run --Werror'
```
Expected: 无输出（通过）。若报格式错误：`clang-format -i` 修复后重跑并 `git add` 已格式化文件

- [ ] **Step 3: 检查 spec 覆盖**

对照 spec 逐项确认：
- [ ] fusion 不再订阅 /lidar/cluster、无 lidar_tracks_ 池、贴类别循环已删
- [ ] publish_lidar_location 只填相机池
- [ ] radar_lidar enable_cluster=false（runtime.yaml + 代码默认）
- [ ] 测试覆盖：聚类不再影响坐标（Task 1 红用例转绿）

- [ ] **Step 4: Commit 收尾 + 推送**

```bash
git add -A
git commit -m "style: clang-format"  # 仅当 Step 2 有格式修复
git push -u origin feat/camera-only
```

- [ ] **Step 5: 汇报**

报告：测试结果摘要（用例数、红→绿路径）、commit 列表、push 状态。等待用户决定是否开 PR 或合入 develop。

---

## 自审记录

**Spec 覆盖：** 删订阅/删池/删贴类别/单池输出（Task 2）、enable_cluster 开关 + runtime.yaml（Task 3）、测试更新 + 聚类无效验证（Task 1）、雷达端测试适配（Task 3 Step 1-2）——全覆盖。

**占位符扫描：** 无 TBD/TODO；所有测试代码完整给出。

**类型一致性：** `publish_lidar_location` 新签名单参（Task 2 Step 1/2 一致）；`enable_cluster` 参数名在 hpp/cpp/yaml/测试四端一致；`make_camera_slot(x, y, class_id, sec, nanosec)` 参数顺序与现有 helper 定义（273 行）一致。
