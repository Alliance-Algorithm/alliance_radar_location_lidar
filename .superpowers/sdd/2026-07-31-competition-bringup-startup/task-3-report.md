# Task 3 Report: Gate Consumers On Registration Status

## Status

Implemented and ready to commit.

## Implementation

- Added the installed `registration_gate` ROS 2 Python executable.
- Subscribed to `/localization/registration_status` with depth 1, reliable,
  transient-local QoS.
- Continued waiting through nonterminal `INITIALIZING` and `REGISTERING` states.
- Exited 0 on `LOCKED`.
- Printed the lidar-provided reason to stderr and exited nonzero on `FAILED`.
- Added no timer, timeout, or sleep to the gate; lidar remains the sole timeout
  authority.
- Started camera inference, fusion, and bridge from the gate's zero-exit launch
  handler only.
- Emitted a launch `Shutdown` event for every nonzero gate exit.
- Added `rclpy` and `radar_interfaces` runtime dependencies and installed the
  executable through `radar_bringup` CMake.

## TDD Evidence

### Gate RED

Registered `test_registration_gate.py` before creating or installing the gate.
Both tests failed with `FileNotFoundError` for the absent installed
`registration_gate`, proving the tests exercised the missing production
behavior.

### Gate GREEN

After the minimal subscriber implementation, both process-level tests passed:

- A `LOCKED` sample published before gate startup was received through
  transient-local durability and produced exit code 0.
- A `FAILED` sample published before gate startup produced a nonzero exit and
  included `registration quality below threshold` in stderr.

### Launch RED

Added orchestration assertions before modifying the launch. They failed because
fusion was still a top-level node and no `OnProcessExit` handler existed.

### Launch GREEN

After moving the three consumers behind the gate exit handler, all six launch
contract tests and both gate integration tests passed.

## Verification

All ROS commands ran in `radar:develop` with the worktree mounted at
`/workspace`.

Passing package-scoped command:

```sh
docker run --rm -v "$PWD:/workspace" -w /workspace/ros_ws radar:develop \
  bash -lc 'source /opt/ros/jazzy/setup.bash && source install/setup.bash && \
  colcon build --packages-select radar_bringup --symlink-install && \
  source install/setup.bash && \
  colcon test --packages-select radar_bringup --event-handlers console_direct+ && \
  colcon test-result --verbose'
```

Result:

- `radar_bringup` build passed.
- 2 CTest targets passed.
- 8 pytest cases passed (6 launch contract, 2 gate integration).
- Aggregate existing workspace test results: 95 tests, 0 errors, 0 failures,
  0 skipped.
- `git diff --check` passed.
- Python compilation and `package.xml` XML validation passed in the container.

## Self-Review

- Confirmed no H.264 changes.
- Confirmed no documentation changes outside this required task report.
- Confirmed no bridge protocol, JSON, command, address, or callback changes.
- Confirmed dirty camera submodules and unrelated untracked plans were not
  modified or staged.
- Confirmed no production timeout, timer, or sleep was introduced.
- Mutation check: changing QoS durability/reliability, terminal exit code,
  failure reason output, top-level consumer placement, success action count, or
  failure shutdown behavior is covered by a test.

## Concerns

A clean `colcon build --packages-up-to radar_bringup --symlink-install` cannot
complete because the pre-existing dirty `radar_camera`/camera-driver state lacks
`hikcamera/shm.hpp`. The failure occurs while compiling `radar_camera`, before
`radar_bringup`, and is unrelated to Task 3. Per the preserve-dirt constraint,
the third-party camera submodules were not changed. The package-scoped build and
all Task 3 tests pass using the existing installed dependency overlay.
