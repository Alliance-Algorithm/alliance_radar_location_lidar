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
colcon build --packages-select radar_interfaces radar_lidar --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select radar_interfaces radar_lidar --event-handlers console_direct+ --return-code-on-test-failure
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
- The optional Odin relocalization path does not transition `LocalizationStage` to `LOCKED`, so with Task 3 gating it remains on GICP fallback until a real GICP result is accepted. This matches the stated requirement that initial/external estimates are not proof of registration.
