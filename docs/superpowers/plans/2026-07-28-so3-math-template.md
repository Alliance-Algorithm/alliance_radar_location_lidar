# SO3 Math Template Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace duplicated SO(3) formulas in `radar_fast_livo2` with one minimal, numerically stable `SO3Math<Scalar>` header-only utility while preserving the existing LIO behavior and right-perturbation convention.

**Architecture:** Add `radar_fast_livo2/lie.hpp` containing only a stateless `SO3Math<Scalar>` utility with `hat`, `exp`, `log`, and `fromTwoVectors`. Keep `StatesGroup`, `VoxelMapManager`, IMU propagation, VIO, and ROS interfaces based on Eigen `double`; migrate only their repeated SO(3) call sites. Add one small gtest target for the utility’s high-value numerical cases.

**Tech Stack:** C++23, Eigen3, ROS 2 `ament_cmake`, `ament_cmake_gtest`, GoogleTest.

## Global Constraints

- Do not add unnecessary `[[nodiscard]]`, wrapper layers, aliases, logs, or tests.
- Do not change the existing right-perturbation convention: `R_new = R_old * Exp(delta)` and `delta = Log(R_old.transpose() * R_new)`.
- Do not template `StatesGroup`, `VoxelMapManager`, `GtsamBackend`, or the KISS-Matcher submodule.
- Do not add `SE3`, point-cloud traits, loop-closure logic, GTSAM factor changes, TF changes, or ROS topic changes.
- Keep the LIO production path on `Eigen::Matrix3d`/`Eigen::Vector3d`; use `SO3f` only for the minimal float template check.
- Do not modify unrelated existing worktree changes, submodules, or `tools/loop_closure/gtsam_backend.py`.
- Do not stage or commit unrelated files; no commit is required for this implementation session unless explicitly requested.

---

### Task 1: Add Minimal SO3 API Test Target

**Files:**
- Create: `ros_ws/src/radar_fast_livo2/test/test_lie.cpp`
- Modify: `ros_ws/src/radar_fast_livo2/CMakeLists.txt:79-97`

**Interfaces:**
- Consumes: The planned API `radar::fast_livo2::lie::SO3Math<Scalar>` with aliases `SO3d` and `SO3f`.
- Produces: Test executable `test_lie` registered with the package test suite.

- [ ] **Step 1: Add the smallest useful gtest source**

Create `test/test_lie.cpp` with only these cases:

```cpp
#include "radar_fast_livo2/lie.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

using radar::fast_livo2::lie::SO3d;
using radar::fast_livo2::lie::SO3f;

TEST(SO3Math, HatMatchesCrossProduct) {
    const Eigen::Vector3d v(1.0, -2.0, 3.0);
    const Eigen::Vector3d w(-4.0, 5.0, 0.5);
    EXPECT_TRUE((SO3d::hat(v) * w).isApprox(v.cross(w), 1e-12));
}

TEST(SO3Math, IdentityAndRoundTrip) {
    EXPECT_TRUE(SO3d::exp(Eigen::Vector3d::Zero()).isApprox(
        Eigen::Matrix3d::Identity(), 1e-12));
    EXPECT_TRUE(SO3d::log(Eigen::Matrix3d::Identity()).isApprox(
        Eigen::Vector3d::Zero(), 1e-12));

    const Eigen::Vector3d phi(0.2, -0.1, 0.3);
    EXPECT_TRUE(SO3d::log(SO3d::exp(phi)).isApprox(phi, 1e-12));
}

TEST(SO3Math, SupportsFloat) {
    const Eigen::Vector3f phi(0.2F, -0.1F, 0.3F);
    EXPECT_TRUE(SO3f::log(SO3f::exp(phi)).isApprox(phi, 1e-5F));
}

TEST(SO3Math, LogNearPiIsFinite) {
    const double pi = std::acos(-1.0);
    const Eigen::Vector3d axis = Eigen::Vector3d(1.0, 2.0, -3.0).normalized();
    const Eigen::Vector3d recovered =
        SO3d::log(SO3d::exp(axis * (pi - 1e-10)));

    EXPECT_TRUE(recovered.allFinite());
    EXPECT_NEAR(recovered.norm(), pi, 1e-8);
    EXPECT_NEAR(std::abs(recovered.normalized().dot(axis)), 1.0, 1e-8);
}

TEST(SO3Math, AlignsSameOrdinaryAndOppositeVectors) {
    const Eigen::Vector3d x = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y = Eigen::Vector3d::UnitY();

    EXPECT_TRUE(SO3d::fromTwoVectors(x, x).isApprox(
        Eigen::Matrix3d::Identity(), 1e-12));
    EXPECT_TRUE((SO3d::fromTwoVectors(x, y) * x).isApprox(y, 1e-12));
    EXPECT_TRUE((SO3d::fromTwoVectors(x, -x) * x).isApprox(-x, 1e-12));
}

}  // namespace
```

Do not add random tests, ROS node tests, performance tests, or duplicate float versions of every case.

- [ ] **Step 2: Register the header-only test in CMake**

Under the existing `if(BUILD_TESTING)` block in `CMakeLists.txt`, add:

```cmake
ament_add_gtest(test_lie test/test_lie.cpp)
target_include_directories(test_lie PRIVATE include)
target_link_libraries(test_lie Eigen3::Eigen)
```

Do not link the test to `${PROJECT_NAME}_core`; the test exercises a header-only Eigen utility and should not pull the full ROS/PCL/GTSAM static library.

- [ ] **Step 3: Run the new target before implementation**

Run from `ros_ws`:

```bash
colcon build --packages-select radar_fast_livo2 --cmake-args -DBUILD_TESTING=ON
```

Expected: configuration/build fails because `radar_fast_livo2/lie.hpp` does not exist. This confirms the test target actually includes the new API before production implementation.

### Task 2: Implement the Minimal `SO3Math<Scalar>` Utility

**Files:**
- Create: `ros_ws/src/radar_fast_livo2/include/radar_fast_livo2/lie.hpp`

**Interfaces:**
- Consumes: Eigen fixed-size 3D vectors/matrices and scalar `float` or `double`.
- Produces: `SO3Math<Scalar>`, `SO3d`, and `SO3f` with the exact API below.

- [ ] **Step 1: Add the public template surface without attributes**

Use this interface and no additional public helpers:

```cpp
#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>

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

}  // namespace radar::fast_livo2::lie
```

Define all functions inline inside the header. Do not add OpenCV, Sophus, GTSAM, or PCL includes.

- [ ] **Step 2: Implement `hat` and `exp` once**

Implement `hat` with direct matrix assignment so that `hat(v) * w` equals `v.cross(w)`.

Implement `exp(phi)` from the rotation vector directly:

```cpp
const Scalar theta_squared = phi.squaredNorm();
const Scalar small_angle = std::sqrt(std::numeric_limits<Scalar>::epsilon());
const Mat3 K = hat(phi);

Scalar a;
Scalar b;
if (theta_squared < small_angle * small_angle) {
    a = Scalar(1) - theta_squared / Scalar(6);
    b = Scalar(0.5) - theta_squared / Scalar(24);
} else {
    const Scalar theta = std::sqrt(theta_squared);
    a = std::sin(theta) / theta;
    b = (Scalar(1) - std::cos(theta)) / theta_squared;
}

return Mat3::Identity() + a * K + b * K * K;
```

The two-argument overload must only scale the angular velocity and delegate to the same implementation:

```cpp
return exp(angular_velocity * static_cast<Scalar>(dt));
```

- [ ] **Step 3: Implement stable `log`**

Use this decision sequence:

1. Compute `cos_theta = clamp((rotation.trace() - 1) / 2, -1, 1)` and `theta = acos(cos_theta)`.
2. For `theta < sqrt(epsilon)`, return half of the vee vector from `rotation - rotation.transpose()`.
3. For `pi - theta < sqrt(epsilon)`, extract an axis from `rotation + Identity` by selecting the largest diagonal component, compute the other components from symmetric off-diagonal entries, normalize the axis, and return `theta * axis`.
4. Otherwise return `theta / (2 * sin(theta))` multiplied by the vee vector.

If the near-pi axis extraction has a degenerate denominator, use the selected coordinate axis as a finite fallback. The result must never call `acos` with an unclamped argument and must not divide by a near-zero `sin(theta)` in the near-pi branch.

- [ ] **Step 4: Implement `fromTwoVectors` with parallel and antiparallel handling**

Normalize both inputs. If either norm is below `sqrt(epsilon)`, return `Mat3::Identity()`.

For ordinary vectors:

```cpp
const Vec3 cross = from_normalized.cross(to_normalized);
const Scalar cosine = clamp(from_normalized.dot(to_normalized), Scalar(-1), Scalar(1));
const Mat3 K = hat(cross);
return Mat3::Identity() + K + K * K * ((Scalar(1) - cosine) / cross.squaredNorm());
```

For same-direction vectors, return identity. For opposite vectors, choose the coordinate axis least aligned with `from_normalized`, cross it with the input to produce an orthogonal rotation axis, normalize it, and return `exp(axis * pi)`.

- [ ] **Step 5: Run the focused test target**

Run:

```bash
colcon build --packages-select radar_fast_livo2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select radar_fast_livo2 --event-handlers console_direct+
```

Expected: `test_lie` passes, including the near-pi and float cases. If the near-pi test fails, fix only the axis extraction or threshold in `lie.hpp`; do not weaken the test tolerance or remove the boundary case.

### Task 3: Migrate Existing Call Sites Without Changing Semantics

**Files:**
- Modify: `ros_ws/src/radar_fast_livo2/include/radar_fast_livo2/esikf_state.hpp`
- Modify: `ros_ws/src/radar_fast_livo2/src/imu_processing.cpp`
- Modify: `ros_ws/src/radar_fast_livo2/src/voxel_map.cpp`
- Modify: `ros_ws/src/radar_fast_livo2/src/vio.cpp`
- Modify: `ros_ws/src/radar_fast_livo2/include/radar_fast_livo2/lio_drift_diagnostics.hpp`
- Modify: `ros_ws/src/radar_fast_livo2/src/livmapper_node.cpp`

**Interfaces:**
- Consumes: `lie::SO3d` from `lie.hpp`.
- Produces: The same Eigen matrix/vector results and the same state update ordering as before.

- [ ] **Step 1: Replace state math and remove old definitions**

In `esikf_state.hpp`:

- Include `radar_fast_livo2/lie.hpp`.
- Remove the local `skewSym`, both `Exp` overloads, and `Log` definitions.
- In `StatesGroup::operator+` and `operator+=`, call `lie::SO3d::exp` using the first three state increments.
- In `StatesGroup::operator-`, call `lie::SO3d::log` on `b.rot_end.transpose() * this->rot_end`.
- Keep `RotMtoEuler`, `pointWithVar`, `StatesGroup`, aliases, state indices, and covariance behavior unchanged.

Use the existing right update exactly:

```cpp
a.rot_end = this->rot_end * lie::SO3d::exp(rotation_delta);
```

- [ ] **Step 2: Remove duplicate IMU Rodrigues implementation**

In `imu_processing.cpp`:

- Delete the anonymous-namespace `exp_so3` function.
- Replace all five calls with `lie::SO3d::exp`, preserving `dt` signs:
  - covariance rotation block: `lie::SO3d::exp(angvel_avr, -dt)`
  - forward rotation: `lie::SO3d::exp(angvel_avr, dt)`
  - point-time interpolation: `lie::SO3d::exp(w_head, dt)`
  - no-IMU forward propagation: `lie::SO3d::exp(state.bias_g, dt)`
  - reverse compensation: `lie::SO3d::exp(state.bias_g, -dt_j)`

Do not alter integration order, bias subtraction, covariance propagation, or point compensation formulas.

- [ ] **Step 3: Replace skew and log call sites**

Use `lie::SO3d::hat` in the existing Jacobian/covariance locations:

- `voxel_map.cpp`: all three `skewSym` uses.
- `vio.cpp`: both `skewSym` uses.

Use `lie::SO3d::log` in `lio_drift_diagnostics.hpp` for the existing rotation correction metric. Do not change units or covariance indexing.

- [ ] **Step 4: Replace gravity alignment Rodrigues code**

In `livmapper_node.cpp`, replace the hand-written cross-product/axis-angle matrix block with:

```cpp
const V3D expected_grav(0.0, 0.0, -9.81);
state_.rot_end = lie::SO3d::fromTwoVectors(avg_grav, expected_grav)
    * state_.rot_end;
```

Keep the existing stationary detection, logging, left-multiplication order, and state assignment. The utility handles same-direction and opposite-direction vectors.

- [ ] **Step 5: Search for stale duplicate implementations**

Run:

```bash
rg -n "skewSym|exp_so3|\bExp\(|\bLog\(" \
  ros_ws/src/radar_fast_livo2/include \
  ros_ws/src/radar_fast_livo2/src \
  ros_ws/src/radar_fast_livo2/test
```

Expected: no old `skewSym`, `exp_so3`, or standalone `Exp`/`Log` implementation remains in the package. `lie::SO3d::exp`, `lie::SO3d::log`, and test names are allowed.

### Task 4: Verify the Package and Review the Diff

**Files:**
- Verify only: all files from Tasks 1-3

**Interfaces:**
- Consumes: The migrated package and focused SO(3) tests.
- Produces: Build/test evidence and a clean, narrowly scoped diff.

- [ ] **Step 1: Run the focused package build and tests**

From `ros_ws`, run:

```bash
colcon build --packages-select radar_fast_livo2 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select radar_fast_livo2 --event-handlers console_direct+
```

Expected: package build exits successfully, `test_lie` passes, and the existing `test_lio_drift_diagnostics`, `test_shm_camera`, and `test_ros_image_camera` targets remain green.

- [ ] **Step 2: Check formatting and changed-file scope**

From the repository root, run:

```bash
git diff --check
git status --short
git diff --stat -- \
  docs/superpowers/specs/2026-07-28-so3-math-template-design.md \
  docs/superpowers/plans/2026-07-28-so3-math-template.md \
  ros_ws/src/radar_fast_livo2
```

Confirm that unrelated submodule modifications and `tools/loop_closure/gtsam_backend.py` are not included in the implementation diff. Do not stage or commit anything unless the user explicitly requests it.

- [ ] **Step 3: Compare semantic invariants**

Review the final diff and confirm:

- `StatesGroup` still uses `R_old * Exp(delta)`.
- `operator-` still computes `Log(R_old.transpose() * R_new)`.
- IMU calls preserve positive and negative `dt` signs.
- Gravity alignment still left-multiplies the current world-frame pose.
- No ROS topics, TF ownership, GTSAM factors, or KISS-Matcher code changed.
- Only one implementation of `hat`, SO(3) `exp`, and SO(3) `log` exists in `radar_fast_livo2`.

## Plan Self-Review

- Spec coverage: The plan covers the approved header-only utility, stable zero/near-pi behavior, all listed call sites, minimal tests, and package verification.
- Scope: No SE(3), point-cloud traits, loop closure, or business behavior changes are included.
- Type consistency: `SO3Math<Scalar>::Vec3`/`Mat3`, `SO3d`, `SO3f`, and all call sites use matching Eigen scalar types.
- Test scope: One test file with five focused tests; no large or redundant test suite.
- Placeholder scan: No unresolved implementation choices are left; thresholds, branches, files, and commands are specified above.
