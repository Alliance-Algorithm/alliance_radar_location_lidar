# OdinTuneNode 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 `odin_tune_node`——订阅 Odin1 直出原始点云（`/odin1/cloud_raw`），用 odometry 对齐的滑动窗口背景模型差分提取动态点，复用现有欧氏聚类，全部参数 ROS2 动态可调，供真机无地图调参。

**Architecture:** 三个纯算法组件（`PoseBuffer` 位姿缓存、`BackgroundModel` 滑动窗口帧缓存+位姿对齐、`FrameDifferencer` KdTree 最近邻差分）以 header-only 形式放在 `include/radar_lidar/odin_tune/`，ROS 节点 `odin_tune_node` 在 `tools/` 下组装它们。动态点 → 可选 ROI 裁剪 → 复用现有 `cluster::ClusterStage` → 发布 /odin_tune/* 话题。节点参数通过 `add_on_set_parameters_callback` 实时重建各 stage。

**Tech Stack:** C++23、ROS2 Jazzy（rclcpp）、PCL（kdtree、voxel、segmentation）、Eigen3、gtest。

## Global Constraints

- **绝不影响比赛主链路**：不改动 `src/radar_lidar_node.cpp`、`src/dynamic_cloud_stage.cpp`、`src/cluster_stage.cpp` 及其头文件；`_core` 库（`CMakeLists.txt` 中的 `add_library(radar_lidar_core ...)`）源文件列表**不新增、不删除**；新增组件全部 header-only（只加头文件，不加 .cpp 到 core）
- 新组件命名空间 `radar_lidar::odin_tune`，节点类命名空间 `radar_lidar::node`
- 不修改比赛用的 `radar_bringup/config/lidar/odin_driver.yaml`（其 `sendodom: 0`）；新建 `odin_driver_tune.yaml` 开启 `sendodom: 1`
- 动态点提取语义：**背景模型不含当前帧**（先差分、后 add），背景帧数不足 `bg_num_frames` 时不产生动态点
- 点云坐标约定：cloud_raw 点在雷达系，odometry 位姿为雷达在 odom 系；相对变换 `p_target = T_odom_target^{-1} * T_odom_prev * p_prev`
- 帧差判定：`nearest_dist_sq > diff_threshold²` 判为动态点（严格大于）
- 构建命令在 `ros_ws/` 下执行：`colcon build --packages-select radar_lidar`、`colcon test --packages-select radar_lidar --event-handlers console_direct+`

---

### Task 1: PoseBuffer 组件（TDD）

**Files:**
- Create: `ros_ws/src/radar_lidar/include/radar_lidar/odin_tune/pose_buffer.hpp`
- Modify: `ros_ws/src/radar_lidar/CMakeLists.txt`（测试目标区，`if(BUILD_TESTING)` 块内末尾）
- Test: `ros_ws/src/radar_lidar/test/test_odin_tune.cpp`

**Interfaces:**
- Produces: `radar_lidar::odin_tune::PoseBuffer`
  - `explicit PoseBuffer(std::int64_t max_span_ns)` — 缓存条目的最大时间跨度
  - `void add(radar_lidar::types::Timestamp stamp, const Eigen::Isometry3d& pose)` — 插入（自动丢弃过旧条目）
  - `auto lookup(radar_lidar::types::Timestamp stamp) const -> std::optional<Eigen::Isometry3d>` — 返回时间差绝对值最小的位姿；若最近条目时间差 > `max_span_ns` 返回 `std::nullopt`
  - `void clear()`

- [ ] **Step 1: 写失败测试**

创建 `test/test_odin_tune.cpp`，仅包含 PoseBuffer 测试：

```cpp
#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "radar_lidar/odin_tune/pose_buffer.hpp"
#include "radar_lidar/data_format.hpp"

namespace {

auto make_pose(double x, double y, double yaw) -> Eigen::Isometry3d {
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation()     = Eigen::Vector3d(x, y, 0.0);
    p.linear()          = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return p;
}

} // namespace

TEST(PoseBufferTest, LookupExactStamp) {
    radar_lidar::odin_tune::PoseBuffer buf;
    const auto pose = make_pose(1.0, 2.0, 0.3);
    buf.add(100, pose);

    auto result = buf.lookup(100);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->isApprox(pose, 1e-9));
}

TEST(PoseBufferTest, LookupNearest) {
    radar_lidar::odin_tune::PoseBuffer buf;
    buf.add(100, make_pose(0.0, 0.0, 0.0));
    buf.add(200, make_pose(1.0, 1.0, 0.0));

    auto result = buf.lookup(199);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->translation().x(), 1.0, 1e-9);
}

TEST(PoseBufferTest, LookupTooOldReturnsNullopt) {
    radar_lidar::odin_tune::PoseBuffer buf(1'000);  // 1us 跨度
    buf.add(100, make_pose(0.0, 0.0, 0.0));

    EXPECT_FALSE(buf.lookup(100 + 1'001).has_value());
}

TEST(PoseBufferTest, ClearEmptiesBuffer) {
    radar_lidar::odin_tune::PoseBuffer buf;
    buf.add(100, make_pose(0.0, 0.0, 0.0));
    buf.clear();

    EXPECT_FALSE(buf.lookup(100).has_value());
}
```

- [ ] **Step 2: 运行测试确认失败**

Run: `colcon test --packages-select radar_lidar --event-handlers console_direct+`（此时 CMake 尚无测试目标，预期报 "test_odin_tune.cpp 未加入构建"；先加 CMake 目标再跑）

- [ ] **Step 3: 在 CMakeLists.txt 添加测试目标**

在 `if(BUILD_TESTING)` 块的 `ament_add_gtest(radar_registration_tests ...)` 之后追加：

```cmake
  ament_add_gtest(odin_tune_tests
      test/test_odin_tune.cpp
  )
  target_link_libraries(odin_tune_tests ${PROJECT_NAME}_core)
  target_include_directories(odin_tune_tests PRIVATE include)
```

- [ ] **Step 4: 写最小实现**

创建 `include/radar_lidar/odin_tune/pose_buffer.hpp`：

```cpp
#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <utility>

#include <Eigen/Geometry>

#include "radar_lidar/data_format.hpp"

namespace radar_lidar::odin_tune {

/// @brief 按时间戳缓存 odometry 位姿，支持最近邻查询
/// 时间戳须递增插入（Odin odometry 按序到达）；内部按时间差丢弃过旧条目。
class PoseBuffer {
public:
    /// @param max_span_ns 缓存条目的最大时间跨度（ns），超过视为失配
    explicit PoseBuffer(std::int64_t max_span_ns = 2'000'000'000LL)
        : max_span_ns_(max_span_ns) { }

    void add(types::Timestamp stamp, const Eigen::Isometry3d& pose) {
        entries_.emplace_back(stamp, pose);
        while (entries_.size() > 1
            && stamp - entries_.front().first > max_span_ns_) {
            entries_.pop_front();
        }
    }

    auto lookup(types::Timestamp stamp) const -> std::optional<Eigen::Isometry3d> {
        if (entries_.empty()) {
            return std::nullopt;
        }
        const auto* best = &entries_.front();
        std::int64_t best_delta = std::llabs(stamp - best->first);
        for (const auto& entry : entries_) {
            const std::int64_t delta = std::llabs(stamp - entry.first);
            if (delta < best_delta) {
                best_delta = delta;
                best       = &entry;
            }
        }
        if (best_delta > max_span_ns_) {
            return std::nullopt;
        }
        return best->second;
    }

    void clear() {
        entries_.clear();
    }

private:
    std::deque<std::pair<types::Timestamp, Eigen::Isometry3d>> entries_;
    std::int64_t max_span_ns_;
};

} // namespace radar_lidar::odin_tune
```

- [ ] **Step 5: 运行测试确认通过**

Run: `colcon test --packages-select radar_lidar --event-handlers console_direct+`
Expected: `odin_tune_tests` 中 `PoseBufferTest.*` 全部 PASS

- [ ] **Step 6: 提交**

```bash
git add ros_ws/src/radar_lidar/include/radar_lidar/odin_tune/pose_buffer.hpp \
        ros_ws/src/radar_lidar/test/test_odin_tune.cpp \
        ros_ws/src/radar_lidar/CMakeLists.txt
git commit -m "feat(radar_lidar): PoseBuffer for odin_tune frame-difference tuning node"
```

---

### Task 2: BackgroundModel 组件（TDD）

**Files:**
- Create: `ros_ws/src/radar_lidar/include/radar_lidar/odin_tune/background_model.hpp`
- Modify: `ros_ws/src/radar_lidar/test/test_odin_tune.cpp`（追加测试）
- Test: `ros_ws/src/radar_lidar/test/test_odin_tune.cpp`

**Interfaces:**
- Consumes: `radar_lidar::types::PointCloud`（`std::vector<Eigen::Vector3d>`）、`Eigen::Isometry3d`
- Produces: `radar_lidar::odin_tune::BackgroundModel`
  - `explicit BackgroundModel(int max_frames)` — 窗口容量（>0）
  - `void add(const types::PointCloud& points, const Eigen::Isometry3d& odom_pose)` — 缓存一帧（超容量弹出最旧）
  - `auto align_to(const Eigen::Isometry3d& target_pose) const -> types::PointCloud` — 全部缓存帧变换到 target 雷达系后合并；相对变换 `p_t = T_target^{-1} * T_odom_i * p_i`
  - `auto frame_count() const -> int`
  - `void clear()`

- [ ] **Step 1: 写失败测试**

在 `test/test_odin_tune.cpp` 追加（include `"radar_lidar/odin_tune/background_model.hpp"`）：

```cpp
TEST(BackgroundModelTest, EmptyModelAlignsEmpty) {
    radar_lidar::odin_tune::BackgroundModel model(5);

    auto cloud = model.align_to(Eigen::Isometry3d::Identity());
    EXPECT_TRUE(cloud.empty());
}

TEST(BackgroundModelTest, SamePoseKeepsPoints) {
    radar_lidar::odin_tune::BackgroundModel model(5);
    radar_lidar::types::PointCloud pts {
        Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 2.0, 0.0) };
    const auto pose = make_pose(5.0, 5.0, 0.0);
    model.add(pts, pose);

    auto cloud = model.align_to(pose);
    ASSERT_EQ(cloud.size(), 2u);
    EXPECT_NEAR(cloud[0].x(), 1.0, 1e-9);
    EXPECT_NEAR(cloud[1].y(), 2.0, 1e-9);
}

TEST(BackgroundModelTest, RelativeTransformAlignsFrames) {
    radar_lidar::odin_tune::BackgroundModel model(5);
    // 帧 A：雷达在原点，看到 (10,0,0)
    radar_lidar::types::PointCloud pts { Eigen::Vector3d(10.0, 0.0, 0.0) };
    model.add(pts, make_pose(0.0, 0.0, 0.0));
    // 帧 B：雷达移到 (5,0,0) 看向同一点 (10,0,0)（世界系），相对变换 T_B^{-1}*T_A
    const auto pose_b = make_pose(5.0, 0.0, 0.0);

    auto cloud = model.align_to(pose_b);
    ASSERT_EQ(cloud.size(), 1u);
    // 帧 A 的点 (10,0,0) 变换到 B 雷达系 = (5,0,0)
    EXPECT_NEAR(cloud[0].x(), 5.0, 1e-9);
}

TEST(BackgroundModelTest, WindowSlides) {
    radar_lidar::odin_tune::BackgroundModel model(2);
    model.add({ Eigen::Vector3d(1, 0, 0) }, make_pose(0, 0, 0));
    model.add({ Eigen::Vector3d(2, 0, 0) }, make_pose(0, 0, 0));
    model.add({ Eigen::Vector3d(3, 0, 0) }, make_pose(0, 0, 0));

    EXPECT_EQ(model.frame_count(), 2);
    auto cloud = model.align_to(make_pose(0, 0, 0));
    ASSERT_EQ(cloud.size(), 2u);
    EXPECT_NEAR(cloud[0].x(), 2.0, 1e-9);
    EXPECT_NEAR(cloud[1].x(), 3.0, 1e-9);
}

TEST(BackgroundModelTest, ClearResets) {
    radar_lidar::odin_tune::BackgroundModel model(5);
    model.add({ Eigen::Vector3d(1, 0, 0) }, make_pose(0, 0, 0));
    model.clear();

    EXPECT_EQ(model.frame_count(), 0);
    EXPECT_TRUE(model.align_to(Eigen::Isometry3d::Identity()).empty());
}
```

- [ ] **Step 2: 运行测试确认失败**

Run: `colcon test --packages-select radar_lidar --event-handlers console_direct+`
Expected: FAIL — 编译错误 "background_model.hpp: No such file"

- [ ] **Step 3: 写最小实现**

创建 `include/radar_lidar/odin_tune/background_model.hpp`：

```cpp
#pragma once

#include <deque>
#include <utility>

#include <Eigen/Geometry>

#include "radar_lidar/data_format.hpp"

namespace radar_lidar::odin_tune {

/// @brief 滑动窗口帧缓存：缓存 (雷达系点云, odom 位姿)，支持对齐到任意目标位姿
/// 对齐公式：p_target = T_odom_target^{-1} * T_odom_i * p_i
class BackgroundModel {
public:
    explicit BackgroundModel(int max_frames)
        : max_frames_(max_frames) { }

    void add(const types::PointCloud& points, const Eigen::Isometry3d& odom_pose) {
        frames_.emplace_back(Frame { points, odom_pose });
        while (static_cast<int>(frames_.size()) > max_frames_) {
            frames_.pop_front();
        }
    }

    auto align_to(const Eigen::Isometry3d& target_pose) const -> types::PointCloud {
        const Eigen::Isometry3d target_inv = target_pose.inverse();
        types::PointCloud result;
        for (const auto& frame : frames_) {
            const Eigen::Isometry3d rel = target_inv * frame.pose;
            result.reserve(result.size() + frame.points.size());
            for (const auto& p : frame.points) {
                result.push_back(rel * p);
            }
        }
        return result;
    }

    auto frame_count() const -> int {
        return static_cast<int>(frames_.size());
    }

    void clear() {
        frames_.clear();
    }

private:
    struct Frame {
        types::PointCloud points;
        Eigen::Isometry3d pose;
    };
    std::deque<Frame> frames_;
    int max_frames_;
};

} // namespace radar_lidar::odin_tune
```

- [ ] **Step 4: 运行测试确认通过**

Run: `colcon test --packages-select radar_lidar --event-handlers console_direct+`
Expected: `BackgroundModelTest.*` 全部 PASS

- [ ] **Step 5: 提交**

```bash
git add ros_ws/src/radar_lidar/include/radar_lidar/odin_tune/background_model.hpp \
        ros_ws/src/radar_lidar/test/test_odin_tune.cpp
git commit -m "feat(radar_lidar): BackgroundModel sliding-window frame cache for odin_tune"
```

---

### Task 3: FrameDifferencer 组件（TDD）

**Files:**
- Create: `ros_ws/src/radar_lidar/include/radar_lidar/odin_tune/frame_differencer.hpp`
- Modify: `ros_ws/src/radar_lidar/test/test_odin_tune.cpp`（追加测试）
- Test: `ros_ws/src/radar_lidar/test/test_odin_tune.cpp`

**Interfaces:**
- Consumes: `radar_lidar::types::PointCloud`（当前帧、背景模型）
- Produces: `radar_lidar::odin_tune::FrameDifferencer`
  - `explicit FrameDifferencer(double distance_threshold)` — 帧差距离阈值 (m)，>0
  - `auto differ(const types::PointCloud& current, const types::PointCloud& background) const -> types::PointCloud` — 返回 current 中距 background 最近邻距离 **严格大于** 阈值的点；`background` 为空时返回空
  - `void set_distance_threshold(double t)`

- [ ] **Step 1: 写失败测试**

在 `test/test_odin_tune.cpp` 追加（include `"radar_lidar/odin_tune/frame_differencer.hpp"`）：

```cpp
TEST(FrameDifferencerTest, EmptyBackgroundYieldsEmpty) {
    radar_lidar::odin_tune::FrameDifferencer diff(0.3);
    radar_lidar::types::PointCloud cur { Eigen::Vector3d(1, 0, 0) };

    EXPECT_TRUE(diff.differ(cur, { }).empty());
}

TEST(FrameDifferencerTest, ClosePointIsStatic) {
    radar_lidar::odin_tune::FrameDifferencer diff(0.3);
    radar_lidar::types::PointCloud cur { Eigen::Vector3d(0.0, 0.0, 0.0) };
    radar_lidar::types::PointCloud bg { Eigen::Vector3d(0.1, 0.0, 0.0) };

    EXPECT_TRUE(diff.differ(cur, bg).empty());
}

TEST(FrameDifferencerTest, FarPointIsDynamic) {
    radar_lidar::odin_tune::FrameDifferencer diff(0.3);
    radar_lidar::types::PointCloud cur { Eigen::Vector3d(0.0, 0.0, 0.0) };
    radar_lidar::types::PointCloud bg { Eigen::Vector3d(1.0, 0.0, 0.0) };

    auto result = diff.differ(cur, bg);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].x(), 0.0, 1e-9);
}

TEST(FrameDifferencerTest, ExactlyThresholdIsStatic) {
    radar_lidar::odin_tune::FrameDifferencer diff(0.3);
    radar_lidar::types::PointCloud cur { Eigen::Vector3d(0.0, 0.0, 0.0) };
    radar_lidar::types::PointCloud bg { Eigen::Vector3d(0.3, 0.0, 0.0) };

    EXPECT_TRUE(diff.differ(cur, bg).empty());
}

TEST(FrameDifferencerTest, ThresholdCanBeUpdated) {
    radar_lidar::odin_tune::FrameDifferencer diff(0.3);
    radar_lidar::types::PointCloud cur { Eigen::Vector3d(0.0, 0.0, 0.0) };
    radar_lidar::types::PointCloud bg { Eigen::Vector3d(0.5, 0.0, 0.0) };

    diff.set_distance_threshold(0.6);
    EXPECT_TRUE(diff.differ(cur, bg).empty());
}
```

- [ ] **Step 2: 运行测试确认失败**

Run: `colcon test --packages-select radar_lidar --event-handlers console_direct+`
Expected: FAIL — 编译错误 "frame_differencer.hpp: No such file"

- [ ] **Step 3: 写最小实现**

创建 `include/radar_lidar/odin_tune/frame_differencer.hpp`：

```cpp
#pragma once

#include <vector>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "radar_lidar/data_format.hpp"

namespace radar_lidar::odin_tune {

/// @brief 当前帧 vs 背景模型 KdTree 最近邻差分
/// 背景模型须已对齐到当前帧坐标系；距离严格大于阈值判为动态点。
class FrameDifferencer {
public:
    explicit FrameDifferencer(double distance_threshold)
        : distance_threshold_(distance_threshold) { }

    auto differ(const types::PointCloud& current, const types::PointCloud& background) const
        -> types::PointCloud {
        if (background.empty()) {
            return { };
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr bg_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        bg_cloud->reserve(background.size());
        for (const auto& p : background) {
            bg_cloud->emplace_back(
                static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
        }
        bg_cloud->width    = bg_cloud->size();
        bg_cloud->height   = 1;
        bg_cloud->is_dense = true;

        pcl::KdTreeFLANN<pcl::PointXYZ> tree;
        tree.setInputCloud(bg_cloud);

        const float thresh_sq =
            static_cast<float>(distance_threshold_ * distance_threshold_);

        types::PointCloud result;
        result.reserve(current.size());
        std::vector<int> idx(1);
        std::vector<float> dist_sq(1);
        for (const auto& p : current) {
            const pcl::PointXYZ query(static_cast<float>(p.x()),
                static_cast<float>(p.y()), static_cast<float>(p.z()));
            if (tree.nearestKSearch(query, 1, idx, dist_sq) > 0 && dist_sq[0] > thresh_sq) {
                result.push_back(p);
            }
        }
        return result;
    }

    void set_distance_threshold(double t) {
        distance_threshold_ = t;
    }

private:
    double distance_threshold_;
};

} // namespace radar_lidar::odin_tune
```

- [ ] **Step 4: 运行测试确认通过**

Run: `colcon test --packages-select radar_lidar --event-handlers console_direct+`
Expected: `FrameDifferencerTest.*` 全部 PASS

- [ ] **Step 5: 提交**

```bash
git add ros_ws/src/radar_lidar/include/radar_lidar/odin_tune/frame_differencer.hpp \
        ros_ws/src/radar_lidar/test/test_odin_tune.cpp
git commit -m "feat(radar_lidar): FrameDifferencer KdTree nearest-neighbor differencing"
```

---

### Task 4: odin_tune_node 节点（自定义点类型 + 参数 + 回调 + 发布）

**Files:**
- Create: `ros_ws/src/radar_lidar/include/radar_lidar/odin_tune/cloud_point.hpp`
- Create: `ros_ws/src/radar_lidar/tools/radar_lidar/odin_tune_node.hpp`（与现有 `offline_detection_node.hpp` 同目录约定）
- Create: `ros_ws/src/radar_lidar/tools/odin_tune_node.cpp`
- Create: `ros_ws/src/radar_lidar/tools/odin_tune_runtime.cpp`
- Modify: `ros_ws/src/radar_lidar/CMakeLists.txt`
- Modify: `ros_ws/src/radar_lidar/package.xml`

**Interfaces:**
- Consumes: `PoseBuffer`、`BackgroundModel`、`FrameDifferencer`（上述接口）；`cluster::ClusterStage`（`radar_lidar::cluster::ClusterStage(config::ClusterConfig)`，`process(types::PointCloud) -> std::expected<std::vector<cluster::ClusterResult>, std::string>`）
- Produces: executable `odin_tune_node`；参数：`scan_topic`(str, `/odin1/cloud_raw`)、`odom_topic`(str, `/odin1/odometry`)、`output_frame`(str, `odom`)、`conf_threshold`(double, 35)、`voxel_leaf`(double, 0.05，0=关闭)、`roi_enabled`(bool, true)、`roi_x_min/x_max/y_min/y_max/z_min/z_max`(double)、`bg_num_frames`(int, 10)、`diff_threshold`(double, 0.3)、`cluster_tolerance`(double, 0.25)、`min_cluster_size`(int, 5)、`max_cluster_size`(int, 1000)

- [ ] **Step 1: 自定义点类型头文件**

创建 `include/radar_lidar/odin_tune/cloud_point.hpp`（格式定义见 Odin 驱动 README 4.4 节）：

```cpp
#pragma once

#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace radar_lidar::odin_tune {

/// @brief Odin1 cloud_raw 自定义点格式（x,y,z,intensity,confidence,offset_time）
struct OdinPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::uint8_t intensity = 0;
    std::uint16_t confidence = 0;
    float offset_time = 0.0f;
};

} // namespace radar_lidar::odin_tune

POINT_CLOUD_REGISTER_POINT_STRUCT(radar_lidar::odin_tune::OdinPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint8_t, intensity, intensity)
    (std::uint16_t, confidence, confidence)
    (float, offset_time, offset_time))
```

- [ ] **Step 2: 节点头文件**

创建 `tools/radar_lidar/odin_tune_node.hpp`：

```cpp
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "radar_lidar/cluster_stage.hpp"
#include "radar_lidar/data_format.hpp"
#include "radar_lidar/odin_tune/background_model.hpp"
#include "radar_lidar/odin_tune/frame_differencer.hpp"
#include "radar_lidar/odin_tune/pose_buffer.hpp"

namespace radar_lidar::node {

/// @brief Odin1 直出点云聚类调参节点（无地图，帧差法动态提取）
/// 订阅 /odin1/cloud_raw + odometry，发布 /odin_tune/*；参数实时可调。
class OdinTuneNode : public rclcpp::Node {
public:
    OdinTuneNode();

private:
    struct Params {
        std::string scan_topic { "/odin1/cloud_raw" };
        std::string odom_topic { "/odin1/odometry" };
        std::string output_frame { "odom" };
        double conf_threshold { 35.0 };
        double voxel_leaf { 0.05 };
        config::RoiBounds roi { .use_roi = true,
            .x_min                  = -11.0,
            .x_max                  = 14.0,
            .y_min                  = -7.5,
            .y_max                  = 7.5,
            .z_min                  = 0.0,
            .z_max                  = 1.4 };
        int bg_num_frames { 10 };
        double diff_threshold { 0.3 };
        config::ClusterConfig cluster { };
    };

    void init();
    void declare_and_load_params();
    void rebuild_stages();

    void on_scan(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr& msg);

    void publish_dynamic(const types::PointCloud& pts, types::Timestamp stamp);
    void publish_background(const types::PointCloud& pts, types::Timestamp stamp);
    void publish_clusters(const std::vector<cluster::ClusterResult>& clusters,
        types::Timestamp stamp);
    void publish_diag(std::size_t dynamic_count, std::size_t cluster_count, double elapsed_ms,
        types::Timestamp stamp);

    Params params_;
    odin_tune::PoseBuffer pose_buffer_;
    std::optional<odin_tune::BackgroundModel> bg_model_;
    std::optional<odin_tune::FrameDifferencer> differencer_;
    std::optional<cluster::ClusterStage> cluster_stage_;
    std::uint64_t frame_count_ { 0 };
    std::uint64_t skipped_no_odom_ { 0 };
    std::uint64_t skipped_warmup_ { 0 };

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_background_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clusters_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_cluster_viz_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_diag_;
};

} // namespace radar_lidar::node
```

- [ ] **Step 3: 节点实现**

创建 `tools/odin_tune_node.cpp`（仿 `radar_lidar_node.cpp` 发布模式；AABB 可视化代码复用其 `publish_clusters` 结构）：

```cpp
#include "radar_lidar/odin_tune_node.hpp"

#include <algorithm>
#include <format>
#include <print>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "radar_lidar/geometry_utils.hpp"
#include "radar_lidar/odin_tune/cloud_point.hpp"

namespace radar_lidar::node {

namespace {
    constexpr std::int64_t kMaxOdomDelayNs = 500'000'000LL;  // 0.5s
} // namespace

OdinTuneNode::OdinTuneNode()
    : Node("odin_tune_node",
          rclcpp::NodeOptions { }.automatically_declare_parameters_from_overrides(true))
    , pose_buffer_(kMaxOdomDelayNs) {
    init();
}

void OdinTuneNode::init() {
    declare_and_load_params();
    rebuild_stages();

    sub_scan_ = create_subscription<sensor_msgs::msg::PointCloud2>(params_.scan_topic,
        rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_scan(msg); });
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(params_.odom_topic,
        rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });

    pub_dynamic_ = create_publisher<sensor_msgs::msg::PointCloud2>("/odin_tune/dynamic", 10);
    pub_background_ =
        create_publisher<sensor_msgs::msg::PointCloud2>("/odin_tune/background", 10);
    pub_clusters_ = create_publisher<sensor_msgs::msg::PointCloud2>("/odin_tune/clusters", 10);
    pub_cluster_viz_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("/odin_tune/cluster_viz", 10);
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/odin_tune/diag", 10);

    RCLCPP_INFO(get_logger(),
        "odin_tune ready. scan=%s odom=%s bg_frames=%d diff=%.3f cluster_tol=%.3f",
        params_.scan_topic.c_str(), params_.odom_topic.c_str(), params_.bg_num_frames,
        params_.diff_threshold, params_.cluster.cluster_tolerance);
}

void OdinTuneNode::declare_and_load_params() {
    auto declare = [this](const std::string& name, const rclcpp::ParameterValue& def) {
        if (!has_parameter(name)) {
            declare_parameter(name, def);
        }
    };
    declare("scan_topic", params_.scan_topic);
    declare("odom_topic", params_.odom_topic);
    declare("output_frame", params_.output_frame);
    declare("conf_threshold", params_.conf_threshold);
    declare("voxel_leaf", params_.voxel_leaf);
    declare("roi_enabled", params_.roi.use_roi);
    declare("roi_x_min", params_.roi.x_min);
    declare("roi_x_max", params_.roi.x_max);
    declare("roi_y_min", params_.roi.y_min);
    declare("roi_y_max", params_.roi.y_max);
    declare("roi_z_min", params_.roi.z_min);
    declare("roi_z_max", params_.roi.z_max);
    declare("bg_num_frames", params_.bg_num_frames);
    declare("diff_threshold", params_.diff_threshold);
    declare("cluster_tolerance", params_.cluster.cluster_tolerance);
    declare("min_cluster_size", params_.cluster.min_cluster_size);
    declare("max_cluster_size", params_.cluster.max_cluster_size);

    auto get = [this](const std::string& name, auto& dst) {
        dst = get_parameter(name).get_value<std::decay_t<decltype(dst)>>();
    };
    get("scan_topic", params_.scan_topic);
    get("odom_topic", params_.odom_topic);
    get("output_frame", params_.output_frame);
    get("conf_threshold", params_.conf_threshold);
    get("voxel_leaf", params_.voxel_leaf);
    get("roi_enabled", params_.roi.use_roi);
    get("roi_x_min", params_.roi.x_min);
    get("roi_x_max", params_.roi.x_max);
    get("roi_y_min", params_.roi.y_min);
    get("roi_y_max", params_.roi.y_max);
    get("roi_z_min", params_.roi.z_min);
    get("roi_z_max", params_.roi.z_max);
    get("bg_num_frames", params_.bg_num_frames);
    get("diff_threshold", params_.diff_threshold);
    get("cluster_tolerance", params_.cluster.cluster_tolerance);
    get("min_cluster_size", params_.cluster.min_cluster_size);
    get("max_cluster_size", params_.cluster.max_cluster_size);

    add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        for (const auto& p : params) {
            const auto& n = p.get_name();
            if (n == "scan_topic" || n == "odom_topic" || n == "output_frame") {
                continue;  // topic/frame 变更需重启
            }
            const std::string allowed[] = { "conf_threshold", "voxel_leaf", "roi_enabled",
                "roi_x_min", "roi_x_max", "roi_y_min", "roi_y_max", "roi_z_min", "roi_z_max",
                "bg_num_frames", "diff_threshold", "cluster_tolerance", "min_cluster_size",
                "max_cluster_size" };
            if (std::find(std::begin(allowed), std::end(allowed), n) == std::end(allowed)) {
                return rcl_interfaces::msg::SetParametersResult { false,
                    "unknown parameter: " + n };
            }
        }
        for (const auto& p : params) {
            const auto& n = p.get_name();
            if (n == "conf_threshold") params_.conf_threshold = p.as_double();
            else if (n == "voxel_leaf") params_.voxel_leaf = p.as_double();
            else if (n == "roi_enabled") params_.roi.use_roi = p.as_bool();
            else if (n == "roi_x_min") params_.roi.x_min = p.as_double();
            else if (n == "roi_x_max") params_.roi.x_max = p.as_double();
            else if (n == "roi_y_min") params_.roi.y_min = p.as_double();
            else if (n == "roi_y_max") params_.roi.y_max = p.as_double();
            else if (n == "roi_z_min") params_.roi.z_min = p.as_double();
            else if (n == "roi_z_max") params_.roi.z_max = p.as_double();
            else if (n == "bg_num_frames") params_.bg_num_frames = p.as_int();
            else if (n == "diff_threshold") params_.diff_threshold = p.as_double();
            else if (n == "cluster_tolerance")
                params_.cluster.cluster_tolerance = p.as_double();
            else if (n == "min_cluster_size") params_.cluster.min_cluster_size = p.as_int();
            else if (n == "max_cluster_size") params_.cluster.max_cluster_size = p.as_int();
        }
        rebuild_stages();
        return rcl_interfaces::msg::SetParametersResult { true, "" };
    });
}

void OdinTuneNode::rebuild_stages() {
    bg_model_     = odin_tune::BackgroundModel(params_.bg_num_frames);
    differencer_  = odin_tune::FrameDifferencer(params_.diff_threshold);
    cluster_stage_ = cluster::ClusterStage(params_.cluster);
}

void OdinTuneNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr& msg) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    const auto& p          = msg->pose.pose.position;
    const auto& o          = msg->pose.pose.orientation;
    pose.translation()     = Eigen::Vector3d(p.x, p.y, p.z);
    pose.linear() =
        Eigen::Quaterniond(o.w, o.x, o.y, o.z).toRotationMatrix();
    pose_buffer_.add(rclcpp::Time(msg->header.stamp).nanoseconds(), pose);
}

void OdinTuneNode::on_scan(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    ++frame_count_;
    const auto t0 = std::chrono::steady_clock::now();

    pcl::PointCloud<odin_tune::OdinPoint> raw;
    pcl::fromROSMsg(*msg, raw);

    const types::Timestamp stamp = rclcpp::Time(msg->header.stamp).nanoseconds();

    // 1. confidence 过滤 + 无效点过滤 → 雷达系点云
    types::PointCloud frame_pts;
    frame_pts.reserve(raw.size());
    for (const auto& pt : raw.points) {
        if (pt.confidence < static_cast<std::uint16_t>(params_.conf_threshold)) continue;
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        frame_pts.emplace_back(pt.x, pt.y, pt.z);
    }
    if (frame_pts.empty()) return;

    // 2. 体素下采样（可选）
    if (params_.voxel_leaf > 0.0) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>);
        in->reserve(frame_pts.size());
        for (const auto& p : frame_pts) {
            in->emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
                static_cast<float>(p.z()));
        }
        in->width = in->size(); in->height = 1; in->is_dense = true;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ds(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setLeafSize(static_cast<float>(params_.voxel_leaf),
            static_cast<float>(params_.voxel_leaf), static_cast<float>(params_.voxel_leaf));
        vg.setInputCloud(in);
        vg.filter(*ds);
        frame_pts.clear();
        frame_pts.reserve(ds->size());
        for (const auto& p : ds->points) {
            frame_pts.emplace_back(p.x, p.y, p.z);
        }
    }

    // 3. 当前帧位姿
    const auto pose = pose_buffer_.lookup(stamp);
    if (!pose) {
        ++skipped_no_odom_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "No odometry for stamp %ld (skipped=%lu)", stamp, skipped_no_odom_);
        return;
    }

    // 4. 背景差分（背景不含当前帧：先 differ 后 add）
    const auto background = bg_model_->align_to(*pose);
    if (bg_model_->frame_count() < params_.bg_num_frames) {
        ++skipped_warmup_;
        bg_model_->add(frame_pts, *pose);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Background warmup %d/%d (skipped=%lu)", bg_model_->frame_count(),
            params_.bg_num_frames, skipped_warmup_);
        publish_background(background, stamp);
        return;
    }
    bg_model_->add(frame_pts, *pose);

    auto dynamic_pts = differencer_->differ(frame_pts, background);
    publish_background(background, stamp);

    // 5. 可选 ROI 裁剪（odom 系）
    if (params_.roi.use_roi) {
        dynamic_pts = geom::clip_roi_aabb(dynamic_pts, params_.roi);
    }

    // 6. 聚类
    std::vector<cluster::ClusterResult> clusters;
    if (!dynamic_pts.empty()) {
        auto result = cluster_stage_->process(dynamic_pts);
        if (result) {
            clusters = *result;
        }
    }

    publish_dynamic(dynamic_pts, stamp);
    publish_clusters(clusters, stamp);

    const auto t1 = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    publish_diag(dynamic_pts.size(), clusters.size(), elapsed_ms, stamp);
}

void OdinTuneNode::publish_dynamic(const types::PointCloud& pts, types::Timestamp stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(pts.size());
    for (const auto& p : pts) {
        cloud->emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
            static_cast<float>(p.z()));
    }
    cloud->width = cloud->size(); cloud->height = 1; cloud->is_dense = true;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = rclcpp::Time(stamp);
    msg.header.frame_id = params_.output_frame;
    pub_dynamic_->publish(msg);
}

void OdinTuneNode::publish_background(const types::PointCloud& pts, types::Timestamp stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(pts.size());
    for (const auto& p : pts) {
        cloud->emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
            static_cast<float>(p.z()));
    }
    cloud->width = cloud->size(); cloud->height = 1; cloud->is_dense = true;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = rclcpp::Time(stamp);
    msg.header.frame_id = params_.output_frame;
    pub_background_->publish(msg);
}

void OdinTuneNode::publish_clusters(
    const std::vector<cluster::ClusterResult>& clusters, types::Timestamp stamp) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroids(new pcl::PointCloud<pcl::PointXYZ>);
    centroids->reserve(clusters.size());
    for (const auto& c : clusters) {
        centroids->emplace_back(static_cast<float>(c.centroid.x()),
            static_cast<float>(c.centroid.y()), static_cast<float>(c.centroid.z()));
    }
    centroids->width = centroids->size(); centroids->height = 1; centroids->is_dense = true;
    sensor_msgs::msg::PointCloud2 centroid_msg;
    pcl::toROSMsg(*centroids, centroid_msg);
    centroid_msg.header.stamp = rclcpp::Time(stamp);
    centroid_msg.header.frame_id = params_.output_frame;
    pub_clusters_->publish(centroid_msg);

    visualization_msgs::msg::MarkerArray markers;
    for (size_t i = 0; i < clusters.size(); ++i) {
        const auto& c = clusters[i];

        visualization_msgs::msg::Marker box;
        box.header.stamp = rclcpp::Time(stamp);
        box.header.frame_id = params_.output_frame;
        box.ns = "clusters";
        box.id = static_cast<int>(i);
        box.type = visualization_msgs::msg::Marker::CUBE;
        box.action = visualization_msgs::msg::Marker::ADD;
        box.pose.position.x = (c.min_bound.x() + c.max_bound.x()) / 2.0;
        box.pose.position.y = (c.min_bound.y() + c.max_bound.y()) / 2.0;
        box.pose.position.z = (c.min_bound.z() + c.max_bound.z()) / 2.0;
        box.pose.orientation.w = 1.0;
        box.scale.x = std::max(0.01, c.max_bound.x() - c.min_bound.x());
        box.scale.y = std::max(0.01, c.max_bound.y() - c.min_bound.y());
        box.scale.z = std::max(0.01, c.max_bound.z() - c.min_bound.z());
        box.color.r = 0.0f; box.color.g = 1.0f; box.color.b = 0.0f; box.color.a = 0.3f;
        box.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(box);

        visualization_msgs::msg::Marker centroid;
        centroid.header.stamp = rclcpp::Time(stamp);
        centroid.header.frame_id = params_.output_frame;
        centroid.ns = "centroids";
        centroid.id = static_cast<int>(i);
        centroid.type = visualization_msgs::msg::Marker::SPHERE;
        centroid.action = visualization_msgs::msg::Marker::ADD;
        centroid.pose.position.x = c.centroid.x();
        centroid.pose.position.y = c.centroid.y();
        centroid.pose.position.z = c.centroid.z();
        centroid.pose.orientation.w = 1.0;
        centroid.scale.x = 0.15; centroid.scale.y = 0.15; centroid.scale.z = 0.15;
        centroid.color.r = 1.0f; centroid.color.g = 0.0f; centroid.color.b = 0.0f;
        centroid.color.a = 1.0f;
        centroid.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(centroid);
    }
    pub_cluster_viz_->publish(markers);
}

void OdinTuneNode::publish_diag(std::size_t dynamic_count, std::size_t cluster_count,
    double elapsed_ms, types::Timestamp stamp) {
    diagnostic_msgs::msg::DiagnosticStatus diag;
    diag.name = "odin_tune/detection";
    diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag.message = std::format("dynamic={} clusters={} time_ms={:.2f}", dynamic_count,
        cluster_count, elapsed_ms);
    diag.hardware_id = "odin1";
    pub_diag_->publish(diag);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "{}", diag.message);
}

} // namespace radar_lidar::node
```

- [ ] **Step 4: runtime main**

创建 `tools/odin_tune_runtime.cpp`：

```cpp
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "radar_lidar/odin_tune_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<radar_lidar::node::OdinTuneNode>());
    rclcpp::shutdown();
    return 0;
}
```

- [ ] **Step 5: CMakeLists.txt 添加可执行目标**

在 `ament_auto_add_executable(grid_map_tool ...)` 之后追加：

```cmake
ament_auto_add_executable(odin_tune_node
    tools/odin_tune_node.cpp
    tools/odin_tune_runtime.cpp
)
target_link_libraries(odin_tune_node ${PROJECT_NAME}_core)
target_include_directories(odin_tune_node PRIVATE include tools)
```

在 `install(TARGETS ...)` 列表（`grid_map_tool` 之后）追加 `odin_tune_node`：

```cmake
install(TARGETS ${PROJECT_NAME}_node registration_tool offline_detection_node offline_test_node grid_map_tool odin_tune_node
    DESTINATION lib/${PROJECT_NAME})
```

- [ ] **Step 6: package.xml 添加依赖**

在 `<depend>visualization_msgs</depend>` 后追加：

```xml
  <depend>nav_msgs</depend>
  <depend>rcl_interfaces</depend>
```

- [ ] **Step 7: 构建验证**

Run: `colcon build --packages-select radar_lidar`
Expected: 编译成功，生成 `odin_tune_node` 可执行文件

Run: `ros2 run radar_lidar odin_tune_node`（Ctrl-C 退出；无点云订阅时仅打印启动日志）
Expected: 启动日志 "odin_tune ready. ..." 正常输出，无崩溃

- [ ] **Step 8: 提交**

```bash
git add ros_ws/src/radar_lidar/include/radar_lidar/odin_tune/cloud_point.hpp \
        ros_ws/src/radar_lidar/tools/radar_lidar/odin_tune_node.hpp \
        ros_ws/src/radar_lidar/tools/odin_tune_node.cpp \
        ros_ws/src/radar_lidar/tools/odin_tune_runtime.cpp \
        ros_ws/src/radar_lidar/CMakeLists.txt \
        ros_ws/src/radar_lidar/package.xml
git commit -m "feat(radar_lidar): odin_tune_node - map-less frame-difference clustering with dynamic params"
```

---

### Task 5: 参数 YAML + 驱动配置 + launch

**Files:**
- Create: `ros_ws/src/radar_lidar/config/odin_tune.yaml`
- Create: `ros_ws/src/radar_bringup/config/lidar/odin_driver_tune.yaml`
- Create: `ros_ws/src/radar_bringup/launch/odin_tune.launch.py`

**Interfaces:**
- Consumes: 节点参数名（Task 4）；Odin 驱动 `host_sdk_sample` executable + `config_file` 参数（参考 `odin_localization.launch.py`）

- [ ] **Step 1: 节点参数 YAML**

创建 `config/odin_tune.yaml`（注意 `odometry_high` 默认即可，与规格一致用 `/odin1/odometry`）：

```yaml
odin_tune_node:
  ros__parameters:
    scan_topic: /odin1/cloud_raw
    odom_topic: /odin1/odometry
    output_frame: odom
    conf_threshold: 35.0
    voxel_leaf: 0.05
    roi_enabled: false
    roi_x_min: -11.0
    roi_x_max: 14.0
    roi_y_min: -7.5
    roi_y_max: 7.5
    roi_z_min: 0.0
    roi_z_max: 1.4
    bg_num_frames: 10
    diff_threshold: 0.3
    cluster_tolerance: 0.25
    min_cluster_size: 5
    max_cluster_size: 1000
```

> 说明：无地图时无法获得场地系位姿，`roi_enabled` 默认 `false`；真机调参时若雷达固定摆放可用 `ros2 param set` 开启并按实测边界收紧。

- [ ] **Step 2: Odin 驱动配置（开启 odometry）**

复制 `config/lidar/odin_driver.yaml` 为 `config/lidar/odin_driver_tune.yaml`，仅将 `sendodom: 0` 改为 `sendodom: 1`：

```yaml
# Odin 驱动配置 — 直出点云 + odometry（调参用，不影响比赛主配置 odin_driver.yaml）
# 用法: ros2 launch odin_ros_driver odin1_ros2.launch.py config_file:=此文件路径
# 差异: sendodom 1（比赛主配置为 0）

register_keys:
  strict_usb3.0_check: 0
  use_host_ros_time: 0
  streamctrl: 1

  sendrgbcompressed: 0
  sendrgb: 0
  sendrgbundistort: 0
  sendimu: 0
  enable_imu_smooth: 0

  sendodom: 1
  send_odom_baselink_tf: 0

  senddtof: 1
  cloud_raw_confidence_threshold: 35
  dtof_fps: 100

  sendcloudslam: 0
  sendcloudrender: 0
  senddepth: 0
  sendreprojection: 0
  sendoverlay: 0

  recorddata: 0
  devstatuslog: 1
  save_log: 0
  pubintensitygray: 0
  showpath: 0
  showcamerapose: 0

  custom_map_mode: 0
  custom_init_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
  relocalization_map_abs_path: ""
  mapping_result_dest_dir: ""
  mapping_result_file_name: ""
  sendimagemask: 0
  image_mask_abs_path: ""
  resetalgo: 0
```

- [ ] **Step 3: launch 文件**

创建 `launch/odin_tune.launch.py`（仿 `odin_localization.launch.py` 结构，仅起驱动 + 调参节点）：

```python
#!/usr/bin/env python3
"""Odin1 直出点云聚类调参 launch：odin 驱动（cloud_raw + odometry）+ odin_tune_node。
不影响比赛主链路。用法:
    ros2 launch radar_bringup odin_tune.launch.py
调参: ros2 param set /odin_tune_node <param> <value>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("radar_bringup")
    radar_dir   = get_package_share_directory("radar_lidar")

    tune_params_arg = DeclareLaunchArgument(
        "tune_params",
        default_value=os.path.join(radar_dir, "config", "odin_tune.yaml"),
        description="odin_tune_node parameter YAML",
    )
    odin_config_arg = DeclareLaunchArgument(
        "odin_config",
        default_value=os.path.join(bringup_dir, "config", "lidar", "odin_driver_tune.yaml"),
        description="Odin driver control_command.yaml (senddtof + sendodom)",
    )

    odin_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("odin_config")}],
    )

    tune_node = Node(
        package="radar_lidar",
        executable="odin_tune_node",
        name="odin_tune_node",
        output="screen",
        parameters=[LaunchConfiguration("tune_params")],
    )

    return LaunchDescription([
        tune_params_arg,
        odin_config_arg,
        odin_node,
        tune_node,
    ])
```

- [ ] **Step 4: 构建 + 语法验证**

Run: `colcon build --packages-select radar_lidar radar_bringup`
Expected: 编译成功

Run: `ros2 launch radar_bringup odin_tune.launch.py --show-args`
Expected: 显示 `tune_params`、`odin_config` 两个可配置参数，无报错

- [ ] **Step 5: 提交**

```bash
git add ros_ws/src/radar_lidar/config/odin_tune.yaml \
        ros_ws/src/radar_bringup/config/lidar/odin_driver_tune.yaml \
        ros_ws/src/radar_bringup/launch/odin_tune.launch.py
git commit -m "feat(bringup): odin_tune launch + driver config (sendodom on) for map-less tuning"
```

---

### Task 6: 文档 + 全量验证

**Files:**
- Modify: `README.md`（LiDAR 驱动章节后新增小节）

- [ ] **Step 1: README 新增使用说明**

在 `README.md` 的 `#### Odin1 内置重定位（可选）` 小节之后（`---` 之前）追加：

```markdown
#### Odin1 直出聚类调参（无地图，比赛链路不受影响）

`odin_tune_node` 订阅 Odin1 直出原始点云 `/odin1/cloud_raw`，用 odometry
对齐的滑动窗口背景模型差分提取动态点，再走欧氏聚类，用于无地图时真机调参。
**比赛链路不使用此节点**；调参结论回填比赛 YAML。

```bash
ros2 launch radar_bringup odin_tune.launch.py
```

Foxglove 查看：

- `/odin_tune/dynamic` 动态点云
- `/odin_tune/background` 背景模型（调试）
- `/odin_tune/clusters` 聚类质心 + `/odin_tune/cluster_viz` AABB 边框
- `/odin_tune/diag` 诊断（dynamic/clusters/time_ms）

实时调参（改完立即生效，无需重启）：

```bash
ros2 param set /odin_tune_node diff_threshold 0.25
ros2 param set /odin_tune_node cluster_tolerance 0.15
ros2 param set /odin_tune_node bg_num_frames 20
```

可用参数：`conf_threshold`、`voxel_leaf`、`roi_enabled` + `roi_*`、
`bg_num_frames`、`diff_threshold`、`cluster_tolerance`、`min_cluster_size`、
`max_cluster_size`。`scan_topic` / `odom_topic` / `output_frame` 需重启生效。

> 背景模型不含当前帧；启动后前 `bg_num_frames` 帧处于预热期不输出动态点。
```

- [ ] **Step 2: 全量测试 + 构建**

Run: `colcon test --packages-select radar_lidar --event-handlers console_direct+`
Expected: 全部测试 PASS（含新增 `odin_tune_tests`）

Run: `colcon build --packages-select radar_lidar radar_bringup`
Expected: 编译成功，无警告新增

- [ ] **Step 3: 回归确认主链路无改动**

Run: `git diff --stat origin/develop...HEAD -- ros_ws/src/radar_lidar/src ros_ws/src/radar_bringup/config/lidar/odin_driver.yaml`
Expected: 空输出（比赛主链路文件未被触碰）

- [ ] **Step 4: 提交**

```bash
git add README.md
git commit -m "docs: odin_tune_node usage for map-less clustering tuning"
```
