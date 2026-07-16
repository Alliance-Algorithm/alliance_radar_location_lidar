# C++ 命名约定

Alliance Radar 全项目统一的 C++ 命名规范。所有 `radar_*` 包必须遵守。
`.clang-format` 只管格式（空白/缩进/括号），命名由本文档约束。

## 总表

| 类别 | 风格 | 示例 |
|---|---|---|
| 命名空间 | `snake_case`，统一 `radar::` 顶层 | `radar` / `radar::fusion` / `radar::camera` |
| 类 / 结构体 | `PascalCase` | `LidarPipeline` / `KalmanTracker` / `MapData` |
| 枚举类型 | `PascalCase` | `TrackLifecycle` / `ArmorColor` |
| 枚举值 | `UPPER_SNAKE` | `TENTATIVE` / `CONFIRMED` / `RED` |
| 方法 / 自由函数 | `snake_case`（尾置返回类型） | `process()` / `set_map()` / `clip_roi_aabb()` |
| 成员变量 | `snake_case_`（**带尾下划线**，区分成员与同名 getter） | `map_` / `scan_topic_` / `prev_pose_` |
| 公开结构体字段 | `snake_case` | `fitness_score` / `frame_id` |
| 局部变量 | `snake_case` | `cloud_in_map` / `map_result` |
| 编译期常量 / 静态工厂 | `kPascalCase` | `kMaxIterations` / `kEpsilon` / `kIdentity()` |
| config 结构体字段 | `snake_case` | `bullet_speed` / `voxel_leaf_size` |
| 宏 | `UPPER_SNAKE` | `RADAR_LIDAR_EXPORT` |
| 类型别名 | `PascalCase` | `PointCloud` / `SGicpTree` |
| 模板参数 | `PascalCase` 或单大写字母 | `class State` / `class T` / `SendT` |
| concept | `snake_case` + `_trait` 后缀 | `scalar3d_trait` / `has_config_trait` |
| 文件名 / 目录名 | `snake_case` | `dynamic_cloud.hpp` / `map_data.cpp` |

## 方法命名

- 一律 `snake_case`，优先尾置返回类型：`auto process() -> Result;`
- `const` / `noexcept` 正常后缀：`auto is_locked() const noexcept -> bool;`

### getter 前缀规则

| 场景 | 规则 | 示例 |
|---|---|---|
| 简单字段访问（直接返回成员/平凡状态） | **裸名词**，无 `get_` | `state()` / `size()` / `is_locked()` / `converged()` |
| 虚接口方法（抽象基类） | **`get_` 前缀** | `get_direction()` / `get_timestamp()` |
| 静态元数据 | **`get_` 前缀** | `get_prefix()` / `get_enum_name()` |
| 非平凡计算（派生值，非直接读成员） | **`get_` 前缀** | `get_attack_window()` |

## 成员变量

- 成员变量一律 `snake_case_`，**带尾下划线**，不加 `m_` 前缀。
- 尾下划线用于区分成员与同名 getter（如成员 `state_` ↔ 访问器 `state()`），
  也避免构造参数与成员撞名。算法核心库默认不用 PIMPL（算法类需被单元测试直接
  访问内部），靠尾下划线做作用域标记；若某模块确有理由使用 PIMPL，仍需遵守
  `Impl` 细节不外泄到头文件的通用约束。

  ```cpp
  class KalmanTracker {
  public:
      auto state() const -> const KalmanState& { return state_; }
  private:
      KalmanState state_;
      double process_noise_pos_ = 0.1;
  };
  ```

## 坐标变换（强制）

**存储的变换矩阵变量必须编码源/目标坐标系**，格式 `t_目标_源`（全小写 snake）：

```cpp
Eigen::Isometry3d t_map_lidar;    // map ← lidar：把 lidar 系的点变到 map 系
Eigen::Isometry3d t_map_camera;   // map ← camera
Eigen::Isometry3d t_lidar_camera; // lidar ← camera
```

规则：
- 前缀 `t_`，随后 `目标_源`，下标顺序与矩阵乘法对齐，便于肉眼查错：

  ```cpp
  t_map_camera = t_map_lidar * t_lidar_camera;  // lidar 相消，剩 map←camera
  auto cloud_in_map = t_map_lidar * cloud_in_lidar;
  ```

- **变换矩阵**（`t_map_lidar`）与**被变换后的数据**（`cloud_in_map`）分开命名，不得混用。
- 禁止把变换矩阵笼统命名为 `transform` / `T`（无 frame 信息，无法判断方向）。
- 转换**函数**可用已清晰表达方向的描述性名（如 `transform_scan_to_map`），
  无需强行套 `into_<frame>()`——名字已含源与目标即可。

## 数学符号例外（KF / 数值代码）

卡尔曼滤波、状态估计等**数值公式**中的标准矩阵符号，**允许保留教材单大写字母**，
以保持公式与文献一致、可读：

```cpp
Eigen::Vector4d x;   // 状态
Eigen::Matrix4d P;   // 状态协方差
Eigen::Matrix4d F;   // 状态转移
Eigen::Matrix H;     // 观测矩阵
Eigen::Matrix K;     // 卡尔曼增益
Eigen::Matrix Q;     // 过程噪声
Eigen::Matrix R;     // 观测噪声
Eigen::Matrix S;     // 新息协方差

// 公式保持文献形态：
K = P * H.transpose() * S.inverse();
```

例外仅限：
1. 卡尔曼 / 信息滤波的标准符号（`x` `P` `F` `H` `K` `Q` `R` `S` 等）。
2. 纯数学推导的局部计算。

对外接口和长生命周期成员优先用语义名（如 `covariance`），单字母仅用于公式局部。

## 命名空间层级

- 顶层统一 `radar::`。
- 子模块用嵌套：`radar::fusion` / `radar::camera` / `radar::calibration` /
  `radar::config` / `radar::types` / `radar::geom`。
- 禁止平行的 `Radar::`（大写）或全局裸类。

## 属性标注（`[[nodiscard]]` 等）

`[[nodiscard]]` 只加在**公开、跨文件调用、返回值真正不该被忽略**的接口上
（如资源获取、错误返回、状态判断类的 getter/预测函数）。

不要加在：
- 匿名 namespace / 文件内部的私有辅助函数——调用点通常就在同一文件里几行之内，
  作者对返回值用法一目了然，标注不提供额外信息，只是冗余噪音。
- 已通过其他方式（如 `std::expected` 返回类型本身）表达"必须处理返回值"语义的函数。

一般原则：**不给代码添加不产生实际收益的限制/标注**。每加一个属性、
每加一层包装都要问“这防住了什么真实的错误”，答不出来就不加。

## 日志分层

日志方式由代码所在层决定，不混用：

| 层 | 是否依赖 rclcpp | 日志方式 |
|---|---|---|
| 算法核心库（如 `radar_lidar_core`：localization / map_data / cluster 等） | 无（ROS-free 静态库） | **不打日志**。错误用 `std::expected` 返回，需上报的状态用返回值带出 |
| ROS 节点层（pipeline / runtime / fusion_node 等） | 有 | 一律 `RCLCPP_*`（`INFO`/`WARN`/`ERROR` 及 `_THROTTLE` 版本） |
| 独立 CLI 工具（如 `registration_tool`） | 无 | 允许 `std::println` / stdout |

规则：
- **算法核心库保持零 IO 副作用**，不得 `std::println` / `printf` / `std::cout`。
  库内产生的事件（如"位姿已锁定"）以状态形式暴露（getter / 返回字段），
  由节点层读取后用 `RCLCPP_*` 打，日志才带节点名 / 时间戳 / level 控制 / Foxglove 聚合。
- 节点层禁止 `std::println` / `std::cout`，一律走 `RCLCPP_*`。

## 占位符注释

未完成代码的占位标记统一用 `// TODO:` / `// FIXME:` / `// HACK:` /
`// WARN:` 前缀（冒号不可省略），团队 Neovim 环境普遍装有
`todo-comments.nvim`，靠这个前缀识别高亮和 `:TodoTelescope` 聚合列表，
写法不规范会导致占位标记既不会高亮也不会被工具收集，实质等同于丢失。

| 关键词 | 用途 | 示例 |
|---|---|---|
| `TODO` | 计划要做但还没做的功能/逻辑 | `// TODO: 实现 VIOManager 类` |
| `FIXME` | 已知有 bug 或临时糙做法，需要回头修 | `// FIXME: 未处理 confidence=0 的边界情况` |
| `HACK` | 明知不优雅但当前先这样绕过去的写法 | `// HACK: 手动加 1ms 规避时间戳相同导致的排序不稳定` |
| `WARN` | 提醒调用者/后续维护者注意的风险点 | `// WARN: 此函数假设点云已按 curvature 升序排列` |

规则：
- 关键词后必须跟英文冒号 `:`，`todo-comments.nvim` 按此模式匹配，
  漏写冒号（如 `// TODO 实现`）不会被识别。
- 大段占位（如整个类/模块待实现）在 `TODO:` 首行之后，用普通注释列出
  要实现的核心步骤和参考来源，方便后续接手者直接定位，不需要重复加
  `TODO:` 前缀：

  ```cpp
  // TODO: 实现 VIOManager 类
  // 核心功能：
  //   1. 将地图中的点投影到相机图像平面
  //   2. 提取 patch，计算仿射 warp 矩阵
  //   3. NCC 匹配 + 光度误差最小化
  // 参考: FAST-LIVO2/src/vio.cpp 的 VIOManager::img_point_cov_update()
  ```

- 严禁用中文关键词（“待办”“暂未实现”等）替代，工具不识别，等同于
  没写标记；中文说明放在关键词冒号后面的正文部分即可。
