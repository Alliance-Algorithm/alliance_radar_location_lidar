# Task 4 Fix Report

## Findings Addressed

- Made recorder and raw-SHM reader terminal failures observable through synchronized state and reason APIs. `RawVideoRecorder::failure_reason()` and `RawShmReader::failure_reason()` are protected by their state mutexes; reader failures now have an explicit `ReaderState::failed` state, while FIFO/reader overrun remains `ReaderState::overrun`.
- Made enabled recorder startup fail synchronously for output and encoder prerequisites. `start()` validates and creates the output directory, confirms it is a directory, and allocates/opens/closes the configured `h264_nvenc` encoder before setting the recorder running or launching its worker. The worker still opens the actual segment independently, so runtime segment failures remain observable.
- Added a thread-safe `RadarCameraNode::status()` and `failure_reason()` API. The recording monitor now observes both recorder and reader terminal states, records the originating reason, stops reader and recorder immediately, stops inference, requests `rclcpp::shutdown()`, and marks the node failed. The runtime catches constructor/startup exceptions and returns process status 1; after a monitor failure it returns 1 instead of treating `infer_running=false` as successful shutdown.
- Added an RAII guard for the existing inference SHM descriptor. If any later node-constructor step throws after `shm_open`, the descriptor is closed; ownership is released only after inference startup has completed successfully.
- Kept recording outside the inference frame loop. No SDK, driver, SHM-writer, or inference-loop recording calls were added.

## Test and Contract Coverage

- Disabled recorder startup remains a no-op with no worker and `stopped` state.
- Enabled startup rejects an output path that is a regular file before worker startup and rejects an unavailable encoder synchronously.
- Recorder FIFO failure coverage verifies that an overrun reaches terminal recorder state, exposes a non-empty reason, stops the worker, and remains observable after `stop()`.
- Reader failed-start and idempotent stop coverage remains independent of live SHM; the reader state API is covered for failed startup.
- Existing recorder frame-order and opt-in hardware tests remain unchanged in scope and do not require a live camera unless explicitly enabled.

## Verification

```text
git diff --check
exit status: 0

source /opt/ros/jazzy/setup.bash && colcon build --packages-select radar_camera --cmake-args -DBUILD_TESTING=ON
exit status: 1
```

The package build stopped before CMake compilation because this worktree does not contain the built dependency artifact:

```text
install/radar_interfaces/share/radar_interfaces/package.sh
```

Consequently, `radar_camera_tests` was not built or run locally. No independent recorder compile was possible without the package dependency graph. The source diff passes whitespace validation.

## Remaining Concerns

- Hardware NVENC open/encode behavior, FFmpeg output validity, rollover, and full-resolution recording still require a host with the camera package dependencies and NVIDIA runtime. The synchronous probe now makes encoder availability a startup failure, but it does not replace the opt-in hardware test.
- The node runtime status is process-fatal only after the monitor observes a terminal recording state. Constructor/startup failures are process-fatal through the runtime exception handler. Normal ROS shutdown remains a zero exit status.
- The required enabled startup order is explicit in the node: start inference first, then construct FIFO/recorder/reader, start recorder, start reader, and start the monitor. The recorder’s synchronous preflight ensures its worker cannot race ahead of reader startup.

## Follow-up Fix

- Moved `infer_thread_start()` ahead of all recording starts. Recording monitor startup now occurs only after inference has successfully launched, so a failed inference startup cannot leave a recorder, reader, or monitor behind.
- Added a constructor-local scope guard with an explicit `constructor_cleanup()` path. Any exception after a reader, recorder, monitor, or inference thread has started stops the monitor first, then reader, recorder, and inference in reverse dependency order, before member destruction begins.
- Transferred the inference SHM descriptor from the local guard to `shm_fd_` immediately after `shm_open`. The constructor cleanup path therefore closes the descriptor even when inference startup fails before a thread is created; successful destruction uses the same idempotent close path.
- Added node-level lifecycle contract tests for inference-before-recording ordering, reverse cleanup of all started components, and SHM cleanup when inference never starts.

## Follow-up Verification

```text
clang-format -i \
  ros_ws/src/radar_camera/src/radar_camera_node.cpp \
  ros_ws/src/radar_camera/include/radar_camera/radar_camera_node.hpp \
  ros_ws/src/radar_camera/test/test_camera_recording_contract.cpp
exit status: 0

git diff --check
exit status: 0

source /opt/ros/jazzy/setup.bash && \
  colcon test --packages-select radar_camera \
  --ctest-args -R radar_camera_tests --event-handlers console_direct+
exit status: 1 before compilation
```

The available test invocation remains blocked by the worktree environment. `colcon` reports that `install/radar_interfaces/share/radar_interfaces/package.sh` is missing, and the worktree also lacks `local_setup.bash`. Consequently the new node-level tests were added but could not be compiled or executed locally.

## Follow-up Concerns

- The lifecycle tests use static node contract helpers because constructing `RadarCameraNode` requires the unavailable OpenVINO/radar interface dependency artifacts and a live camera SHM object. They verify the ordering and unwind contract without requiring hardware.
- Full constructor failure injection through every concrete recorder/reader failure point still requires a dependency-complete build; the actual constructor catch path is covered by code inspection and the isolated contract assertions.
