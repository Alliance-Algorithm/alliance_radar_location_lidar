# Task 4 Report: Validate Inputs And Critical Shutdown

## Status

Implemented and ready to commit.

## Implementation

- Added an `OpaqueFunction` before every hardware or ROS process action in the
  competition launch.
- Validated `side`, `sensor`, finite positive `registration_timeout_sec`, and
  the map, camera, fusion, and bridge file paths before startup.
- Preserved the existing launch argument contract and registration-gated
  consumer startup.
- Registered a launch-wide `OnProcessExit` handler before any process starts.
- Started camera inference, fusion, and bridge only when the registration gate
  exits with code 0.
- Requested whole-launch shutdown for a failed registration gate or any later
  critical process exit, including a clean unexpected exit.
- Ignored child exit events once coordinated launch shutdown was already in
  progress, preventing shutdown-induced exits from becoming new failures.
- Added the `launch_testing` test dependency and registered the startup test
  target with CMake.
- Made no H.264, identity, fusion algorithm, architecture documentation, or
  bridge protocol changes.

## TDD Evidence

### RED

Created and registered `test_competition_launch.py` before changing production
launch code. All 13 initial cases failed because `validated_startup()` and
`critical_process_handler()` did not exist.

### GREEN

After the minimal launch implementation:

- Invalid side/sensor, zero and non-finite timeout, and missing map/config files
  are rejected.
- A real `LaunchService` test proves invalid input prevents a scheduled
  hardware stub from starting.
- Registration failure requests shutdown, while registration success starts
  the complete consumer set.
- Critical child exit requests whole-launch shutdown.
- A real `LaunchService` test proves one critical stub exit interrupts a
  running sibling stub.
- Child exits during coordinated shutdown are ignored by the handler.

The final startup suite contains 14 cases. The existing 7 launch-contract cases
continue to pass unchanged.

## Verification

All ROS commands ran in `radar:develop` with this worktree mounted at
`/workspace`.

Passing affected build and test command:

```sh
docker run --rm -v "$PWD:/workspace" -w /workspace/ros_ws radar:develop \
  bash -lc 'source /opt/ros/jazzy/setup.bash && \
  colcon build --packages-select radar_interfaces radar_fusion radar_bringup --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select radar_bringup --event-handlers console_direct+ && \
  colcon test-result --verbose'
```

Result:

- 3 affected packages built successfully.
- 3 `radar_bringup` CTest targets passed.
- Aggregate result: 112 tests, 0 errors, 0 failures, 0 skipped.
- `ros2 launch radar_bringup competition.launch.py --show-args` passed and
  displayed the expected argument contract.
- `ros2 launch radar_bringup competition.launch.py side:=green` failed in the
  validator with no process-start output.
- `git diff --check` passed.

## Self-Review

- Confirmed validation executes before the global exit handler, hardware
  includes, localization include, and registration gate.
- Confirmed the process-exit handler is registered before any process starts.
- Confirmed the handler distinguishes the registration gate by action identity
  and treats every other process exit as critical.
- Confirmed coordinated shutdown exits do not emit recursive shutdown events.
- Replaced an initially weak marker assertion with a real `LaunchService`
  ordering test.
- Mutation check: removing a validation branch, moving validation after the
  stub, accepting gate failure, failing to release consumers, ignoring a
  critical exit, or re-emitting shutdown during teardown is covered.
- Preserved unrelated dirty camera submodules and untracked plan files.

## Concerns

- Sourcing the existing workspace overlay prints the pre-existing warning that
  `install/radar_camera/share/radar_camera/local_setup.bash` is absent. The
  affected package build and all Task 4 tests pass, and the dirty camera
  submodules were intentionally left untouched.
- Hardware-backed startup was not run by design; orchestration is exercised
  with process stubs as required.
