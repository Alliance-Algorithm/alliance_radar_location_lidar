# SO3 数学模板工具设计

日期: 2026-07-28  
状态: 待评审

## 1. 目标

集中当前 `radar_fast_livo2` 中重复的 SO(3) 数学实现，减少核心业务代码中的手写矩阵公式，同时保留现有 ESIKF 的状态布局、坐标系约定和 `double` 计算路径。

本阶段只做轻量数学工具，不引入完整的 `SO3`/`SE3` 值类型，不改回环检测逻辑，不模板化 `StatesGroup`、`VoxelMapManager` 或 `GtsamBackend`。

## 2. 当前问题

当前实现存在以下重复或边界风险:

- `esikf_state.hpp` 中有 `skewSym`、两个 `Exp` 重载和 `Log`。
- `imu_processing.cpp` 又实现了一份 `exp_so3`。
- `voxel_map.cpp` 和 `vio.cpp` 多处直接构造 skew 矩阵。
- `livmapper_node.cpp` 的重力对齐手写了一份 Rodrigues 公式。
- `Log` 未 clamp `acos` 输入，接近 180 度时 `theta / sin(theta)` 不稳定。
- 两个 `Exp` 实现使用不同的小角度阈值。

## 3. 设计

新增 header-only 文件:

```text
ros_ws/src/radar_fast_livo2/include/radar_fast_livo2/lie.hpp
```

提供无状态静态工具类:

```cpp
namespace radar::fast_livo2::lie {

template <typename Scalar>
struct SO3Math {
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Mat3 = Eigen::Matrix<Scalar, 3, 3>;

    SO3Math() = delete;

    static Mat3 hat(const Vec3& v);
    static Mat3 exp(const Vec3& phi);

    template <typename Time>
    static Mat3 exp(const Vec3& angular_velocity, Time dt);

    static Vec3 log(const Mat3& rotation);
    static Mat3 fromTwoVectors(const Vec3& from, const Vec3& to);
};

using SO3d = SO3Math<double>;
using SO3f = SO3Math<float>;

} // namespace radar::fast_livo2::lie
```

### 3.1 API 语义

- `hat(v)` 返回满足 `hat(v) * w == v.cross(w)` 的反对称矩阵。
- `exp(phi)` 的输入是旋转向量，单位为弧度，返回 SO(3) 旋转矩阵。
- `exp(angular_velocity, dt)` 只做角速度到旋转向量的缩放，然后调用同一份 `exp(phi)` 实现；`dt` 单位为秒。
- `log(rotation)` 返回轴角旋转向量，单位为弧度。
- `fromTwoVectors(from, to)` 返回把 `from` 方向旋转到 `to` 方向的最短旋转。

所有函数不保存状态、不分配堆内存，适合当前 header-only 使用方式。

### 3.2 数值策略

- `exp` 使用旋转向量直接计算 Rodrigues 系数，近零角度使用 Taylor 展开，避免重复归一化。
- `log` 先将 `acos` 参数 clamp 到 `[-1, 1]`。
- `log` 对零角度使用反对称部分的小角度近似。
- `log` 对接近 `pi` 的旋转从 `R + I` 中选择稳定的非零轴分量，并归一化后返回 `pi * axis`。
- `float` 和 `double` 使用同一套公式，阈值通过 `std::numeric_limits<Scalar>` 派生，不在业务代码中散落魔数。
- 输入向量长度过小时，`fromTwoVectors` 返回单位阵。
- 两向量反向时，选择与输入向量最不平行的坐标轴构造正交轴，并返回绕该轴旋转 `pi` 的结果。

## 4. 业务代码使用方式

### 4.1 `StatesGroup`

保留现有别名:

```cpp
using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
```

仅将旋转增量和差分改为:

```cpp
using lie::SO3d;

state.rot_end *= SO3d::exp(delta_rotation);
const V3D delta_rotation = SO3d::log(previous.rot_end.transpose() * current.rot_end);
```

不改变当前右扰动约定:

```text
R_new = R_old * Exp(delta)
delta  = Log(R_old^T * R_new)
```

### 4.2 IMU 传播和去畸变

删除 `imu_processing.cpp` 的局部 `exp_so3`，统一使用:

```cpp
R_imu *= SO3d::exp(angular_velocity, dt);
const M3D R_i = R_head * SO3d::exp(angular_velocity, dt);
```

协方差传播使用同一实现处理负时间增量，不新增另一套反向旋转函数。

### 4.3 地图和 VIO

将 `skewSym(v)` 替换为 `SO3d::hat(v)`，不改变任何 Jacobian 公式和矩阵乘法顺序。

### 4.4 重力对齐

将 `livmapper_node.cpp` 手写的 Rodrigues 矩阵替换为:

```cpp
state_.rot_end = SO3d::fromTwoVectors(avg_gravity, expected_gravity)
    * state_.rot_end;
```

左乘顺序保持不变，因为这里是在世界坐标系中对当前姿态施加重力方向校正。

## 5. 明确不做的事情

- 不把 `StatesGroup` 改成 `StatesGroup<Scalar>`。
- 不把所有 Eigen 矩阵替换成 `SO3Math` 成员对象。
- 不新增 `SE3Math`、齐次矩阵封装或点云 traits。
- 不改 KISS-Matcher 子模块源码。
- 不把距离邻近判断升级成真实回环检测。
- 不修改 GTSAM 因子、TF 发布或 ROS topic。
- 不为每个调用点增加包装函数或重复别名。

## 6. 最小测试范围

只新增一个针对 `lie.hpp` 的 gtest，覆盖高价值行为:

1. `hat(v) * w` 与 `v.cross(w)` 一致。
2. `exp(Zero)` 返回单位阵，`log(Identity)` 返回零向量。
3. 常规小角度满足 `log(exp(phi)) ~= phi`。
4. 接近 180 度时 `log` 结果有限且方向正确，不产生 NaN。
5. `fromTwoVectors` 能处理同向、普通方向和反向输入。

不测试业务流程、ROS 节点、随机点云、性能基准或重复覆盖 `Eigen` 自身行为。`float` 只通过一个最小编译/数值实例验证，不复制整套测试。

## 7. 实现顺序

1. 新增 `lie.hpp` 和最小 gtest，先验证模板接口与边界公式。
2. 在 `esikf_state.hpp` 中引入 `SO3d`，删除旧 SO(3) 函数。
3. 在 `imu_processing.cpp`、`voxel_map.cpp`、`vio.cpp`、`lio_drift_diagnostics.hpp` 和重力对齐代码中替换调用。
4. 运行 `radar_fast_livo2` 的目标测试和编译检查。
5. 只在行为一致且测试通过后删除旧实现，不做无关格式化或业务重构。

## 8. 验收标准

- 工程中只保留一份 `hat`、SO(3) `exp` 和 SO(3) `log` 实现。
- IMU 传播、ESIKF 状态更新、漂移诊断和重力对齐的现有行为保持不变。
- 右扰动和坐标系约定不变。
- 最小 SO(3) 测试通过，包含接近 180 度的边界。
- `radar_fast_livo2` 编译通过。
- 核心业务文件不引入无关测试、日志或额外抽象。

## 9. 设计自检

- 范围: 仅 SO(3) 公共数学工具，适合一个实现迭代。
- 一致性: API 统一使用旋转向量和矩阵，业务仍使用 Eigen `double`。
- 风险: `log` 的接近 `pi` 分支是唯一需要重点验证的数值路径。
- 简洁性: 不引入有状态类、SE(3)、traits 或回环业务改造。
- 待评审: 用户确认后再进入 implementation plan 和代码修改。
