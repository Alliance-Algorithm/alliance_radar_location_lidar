# Task 4 Report: Registration Timeout And Failure Exit

## Status

Implemented and verified.

## Changes

- Added positive finite `registration_timeout_sec` configuration with a 30-second runtime default.
- Started one node-owned registration deadline after successful map initialization and canceled it after real-GICP lock.
- Preserved the most recent scan or registration rejection and published terminal transient-local `FAILED` status as `Registration timeout: <last rejection>`.
- Deferred shutdown by 100 ms after terminal publication so reliable subscribers can receive the status, then requested ROS shutdown.
- Propagated timeout failure from `RadarLidarNode` to a non-zero executable exit.
- Added and forwarded the launch argument; launch rejects invalid timeout and `side` values outside `{red, blue}`.
- Added a ROS surface regression test proving an insufficient scan times out with the exact rejection context and publishes neither pose nor `map -> radar_base` static TF.

## TDD Evidence

- RED: focused timeout test processed `Too few points: 10` but received no `FAILED` status and ROS remained active.
- GREEN: the same focused test received `FAILED` with `Registration timeout: Too few points: 10`, observed no pose/static TF, and observed shutdown.

## Verification

- Container: `radar:develop`, repository mounted at `/workspace`; no host ROS build or test was run.
- Focused: `RADAR_LIDAR_TEST_DOMAIN_ID=231 ./build/radar_lidar/radar_surface_tests --gtest_filter='*RegistrationTimeout*:*InsufficientScan*'` passed 1/1.
- Full requested packages: `colcon test --packages-select radar_interfaces radar_lidar radar_bringup --event-handlers console_direct+ --return-code-on-test-failure` passed all 4 CTest targets, 0 failures.
- Runtime boundary: installed `radar_lidar_node` exited with status 1 after a 0.2-second timeout.
- Launch boundary: invalid side and non-positive timeout rejected; valid `2.5` forwarded as a numeric node parameter.
- `git diff --check` passed.

## Self-Review

- Confirmed the node is the sole registration-timeout authority; no launch timer was added.
- Confirmed only accepted real-GICP lock cancels the deadline and retains existing static `map -> radar_base` ownership.
- Corrected rejection capture to occur before the extrinsic lookup so missing TF cannot cause a stale timeout reason.
- Preserved unrelated dirty submodules and the unrelated untracked architecture plan.

## Concerns

- The 100 ms post-publication shutdown handoff is intentionally separate from the registration deadline. It improves delivery of the terminal reliable/transient-local status before process exit but is still a bounded scheduling assumption under extreme executor starvation.

## Review Round 1/5

### Changes

- Made `FAILED` terminal by returning from `on_scan` before processing queued scans and rechecking after registration work before any TF, pose, or perception output.
- Extended the timeout regression test with a real `radar_base -> lidar_link` extrinsic and a registration-capable scan published from the observed `FAILED` callback.
- Tightened the rejection assertion to exact equality with `Registration timeout: Too few points: 10`.

### TDD And Verification

- RED command in `radar:develop`:
  `docker run --rm -v "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.worktrees/competition-bringup:/workspace" -w /workspace/ros_ws radar:develop bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --packages-select radar_interfaces radar_lidar --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && RADAR_LIDAR_TEST_DOMAIN_ID=231 ./build/radar_lidar/radar_surface_tests --gtest_filter="RadarLidarTimeoutTest.RegistrationTimeoutAfterInsufficientScanFailsWithoutPoseOrStaticTf"'`
- RED result: 0/1 passed; after `FAILED`, the queued scan logged `Pose locked`, `pose_count` was 2, and `static_tf_count` was 1.
- GREEN command in `radar:develop`:
  `docker run --rm -v "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.worktrees/competition-bringup:/workspace" -w /workspace/ros_ws radar:develop bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --packages-select radar_interfaces radar_lidar --cmake-args -DBUILD_TESTING=ON && source install/setup.bash && RADAR_LIDAR_TEST_DOMAIN_ID=231 ./build/radar_lidar/radar_surface_tests --gtest_filter="RadarLidarTimeoutTest.RegistrationTimeoutAfterInsufficientScanFailsWithoutPoseOrStaticTf:RadarLidarSurfaceTest.AcceptedRegistrationPublishesImmutableStaticTransformAndLatchedStatus"'`
- GREEN result: 2/2 passed in 835 ms; timeout remained terminal and accepted registration behavior remained intact.

### Concerns

- No new concerns. Terminal-state synchronization follows the executable's existing single-threaded executor architecture.
