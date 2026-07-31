# Task 5 Report

## Changes

- Updated `radar_bringup/launch/competition.launch.py` to start `radar_camera_node` while preserving the Hik camera, localization, fusion, and bridge launch actions.
- Forwarded `enable_raw_recording` and all ten recording parameters with the exact YAML defaults.
- Strengthened the dependency-light static launch contract test to parse the launch AST and prove all 11 `LaunchConfiguration` substitutions are in the `radar_camera_node` parameters list, including the disabled `enable_raw_recording:=false` default path.
- Registered the contract test with the `radar_bringup` CMake test pattern.
- Documented disabled/enabled competition commands and explicit camera, SHM, NVENC, model, LiDAR, map, and storage hardware blockers.

## Verification

- `python3 -m unittest discover -s ros_ws/src/radar_bringup/test -p 'test_*.py' -v` passed: 4 tests.
- `python3 -m py_compile ros_ws/src/radar_bringup/launch/competition.launch.py ros_ws/src/radar_bringup/test/test_competition_launch_contract.py` passed.
- `git diff --check` passed.
- No runtime launch or hardware recording test was run because this environment lacks the complete ROS dependency artifacts and competition hardware.

## Concerns

- Runtime launch, camera SHM input, TensorRT model loading, NVENC encoding, LiDAR connectivity, and full-resolution segment validation require the competition hardware and dependencies. No SDK, driver, or SHM-writer files were changed.

## Final Fix Wave Report

### Findings Addressed

- Changed `RawVideoRecorder::stop()` to close the FIFO input and request a graceful worker drain. The worker continues popping accepted frames in FIFO order until the queue is empty, then finalizes the active MPEG-TS segment and JSONL sidecar before the caller joins it. Terminal encoder, output, sidecar, and FIFO-overrun failures still use the explicit failure/overrun abort path.
- Added `RawVideoRecorder.StopDrainsAcceptedFramesBeforeFinalizing`, which pushes three accepted frames, calls `stop()` immediately, and verifies `queued == encoded == 3`, sidecar sequence order `[1, 2, 3]`, and a stopped terminal state.
- Replaced raw-reader SDK SHM ownership with direct read-only POSIX ownership: `shm_open(O_RDONLY)`, `fstat` exact-size validation, `mmap(PROT_READ, MAP_SHARED)`, and direct `munmap`/`close`. The reader does not call `hikcamera::SHMInit`, `SHMGetPtr`, `SHMReleasePtr`, or `SHMClose`, and it never accesses semaphore/mutex fields. Frame-counter polling and slot copying remain unchanged.
- Added a pure `valid_shm_object_size` contract test covering exact, undersized, and oversized objects. No SDK, driver, or SHM-writer source was modified.
- Fixed the flush `EAGAIN` packet cleanup leak and made raw SHM timestamp conversion an explicit `duration_cast<std::chrono::nanoseconds>`.

### Verification

```text
git diff --check
exit status: 0

clang-format --dry-run --Werror <six touched C++ files>
exit status: 0

raw reader SDK API scan (SHMInit/SHMGetPtr/SHMReleasePtr/SHMClose/SHMUnlink)
no matches in raw_shm_reader.cpp

source /opt/ros/jazzy/setup.bash && colcon test --packages-select radar_camera \
  --ctest-args -R radar_camera_tests --event-handlers console_direct+
exit status: 1 before compilation
```

The package test command remains blocked by the pre-existing worktree dependency setup gap: `local_setup.bash` is absent and `install/radar_interfaces/share/radar_interfaces/package.sh` cannot be found. Therefore the new recorder and mapping tests were not compiled or executed locally.

### Return Status, Hash, Tests, Concerns

- Return status: final review fixes implemented and committed.
- Implementation hash: recorded after commit in the final response.
- Tests: whitespace and formatting checks passed; static raw-reader API scan passed; package test discovery was blocked before compilation by the missing `radar_interfaces` artifact.
- Concerns: graceful drain and NVENC sidecar assertions require the dependency-complete camera build and hardware/runtime encoder. Read-only SHM mapping requires an existing writer-created object with exactly `sizeof(hikcamera::imageSHM)` bytes; the reader intentionally does not create, resize, initialize, or synchronize the SHM object.
