# Task 3 Report: Static Registration TF And Status

## Status

Implemented and verified LiDAR Task 3 in the `competition-bringup` worktree.

## Changes

- Added `radar_interfaces/msg/RegistrationStatus.msg` with `INITIALIZING`, `REGISTERING`, `LOCKED`, and `FAILED` states plus `state` and `reason` fields.
- Added a reliable, transient-local, keep-last(1) `/localization/registration_status` publisher.
- Replaced the runtime dynamic TF broadcaster with `tf2_ros::StaticTransformBroadcaster`.
- Published `map -> radar_base` once, on the first transition to `LOCKED` after accepted GICP registration.
- Removed runtime publication of that edge on `/tf`.
- Gated pose and detection output until the localization stage reports `LOCKED`.
- Updated obsolete surface assertions that assumed an initial pose locked without registration.
- Added surface coverage for status QoS, late-subscriber delivery, absence of pre-lock TF, one immutable `/tf_static` transform, and absence from `/tf`.

## TDD Evidence

The focused surface test was first run before the node implementation. It failed because no `/tf_static` publisher existed. After the implementation, the same test passed and the complete package suite was run.

## Verification

All ROS build and test commands ran in `radar:develop` with the worktree mounted at `/workspace`.

```text
clang-format --dry-run --Werror ros_ws/src/radar_lidar/include/radar_lidar/geometry_utils.hpp ros_ws/src/radar_lidar/include/radar_lidar/radar_lidar_node.hpp ros_ws/src/radar_lidar/src/radar_lidar_node.cpp ros_ws/src/radar_lidar/test/test_radar_lidar_node.cpp
colcon build --packages-select radar_interfaces radar_lidar radar_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select radar_interfaces radar_lidar radar_bringup --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --all --verbose

Summary: 53 tests, 0 errors, 0 failures, 0 skipped
```

## Self-Review

- Confirmed only `radar_interfaces` and `radar_lidar` production/test files were changed; bridge code was not touched.
- Confirmed unrelated dirty camera-driver submodules and the pre-existing untracked plan remain untouched.
- Confirmed `tf2_msgs` is test-only and `radar_interfaces` is a runtime dependency.
- Confirmed the status publisher's reliability, durability, history, and depth through ROS graph endpoint inspection.
- Confirmed test callback state outlives locally scoped ROS subscriptions.

## Concerns

- Existing PCL/CMake policy warnings remain during configure; they predate this task and do not affect the successful build or tests.
- The optional Odin relocalization path does not transition `LocalizationStage` to `LOCKED`; a real GICP result must still be accepted. This matches the stated requirement that initial/external estimates are not proof of registration.

## Round 1/5 Fix

### Findings Addressed

- Odin relocalization is now estimate-only and can never suppress `LocalizationStage::process()`. Every usable scan still executes GICP until accepted lock; after lock, pose output remains equal to the frozen static transform.
- The GICP result remains `T_map_lidar`. The node consumes the bringup-owned `radar_base -> lidar_link` static TF as `T_radar_base_lidar` and computes exactly `T_map_radar_base = T_map_lidar * inverse(T_radar_base_lidar)` before publishing `/lidar/pose` and `map -> radar_base`.
- Added a non-identity `T_radar_base_lidar` regression. The test publishes the same `radar_base -> lidar_link` TF convention used by `radar_bringup/config/common/extrinsics.yaml`; no duplicate calibration parameters or second TF authority were introduced.
- Added a rejected scan that remains `REGISTERING` and asserts zero pose, dynamic cloud, cluster cloud, and cluster visualization output before a subsequent accepted scan.

### TDD Evidence

- `OdinEstimateCannotBypassOrChangeFrozenGicpRegistration` failed before the fix because the Odin branch skipped GICP and no locked pose arrived.
- `ComposesNonIdentityRadarBaseToLidarExtrinsic` was added before the composition helper and initially failed to build because `map_radar_base_pose` did not exist.
- `RejectedRegistrationPublishesNoPoseOrDetectionBeforeAcceptance` exercises the real ROS node and remains green because Task 3 already gated the rejected path correctly; it now explicitly protects that required behavior and `REGISTERING` state.

### Exact Verification

All commands ran in `radar:develop` with the worktree mounted at `/workspace`:

```text
clang-format --dry-run --Werror ros_ws/src/radar_lidar/include/radar_lidar/geometry_utils.hpp ros_ws/src/radar_lidar/include/radar_lidar/radar_lidar_node.hpp ros_ws/src/radar_lidar/src/radar_lidar_node.cpp ros_ws/src/radar_lidar/test/test_radar_lidar_node.cpp
colcon build --packages-select radar_interfaces radar_lidar radar_bringup --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select radar_interfaces radar_lidar radar_bringup --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --all --verbose

Summary: 56 tests, 0 errors, 0 failures, 0 skipped
```

### Round 1 Concerns

- The node waits up to one second on first use for the bringup static `radar_base -> lidar_link` extrinsic, then caches it as immutable calibration. If that TF is absent, pose/static-TF publication remains gated and the node retries on the next scan.
- Existing PCL/CMake policy warnings remain unrelated to this fix.
