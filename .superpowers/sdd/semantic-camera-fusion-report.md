# Semantic Camera/Fusion Correction Report

Date: 2026-07-31
Worktree: `/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.worktrees/competition-bringup`
Branch: `feat/competition-bringup`
Base: `74fd2e8`
Container: `radar:develop`

## Result

Implemented the semantic flow:

`radar_camera` model class ID -> projected semantic camera array -> identity-bound fusion track -> official centimeter `LidarLocation` -> unchanged radar bridge/ZMQ transport.

No `radar_bridge` or ZMQ protocol source files were modified. H264 was not implemented. Architecture documentation was not modified.

## Production Changes

- `radar_camera` maps model IDs `0..5` to blue and `6..11` to red, retaining semantic class, team, projected map position, and confidence.
- The competition camera publisher now emits `radar_interfaces/msg/CameraDetectionArray` on `/camera/detection`.
- Camera output timestamps use `this->now()` at publication/reception time. The prior `steady_clock` epoch conversion was removed.
- `radar_fusion` subscribes to `CameraDetectionArray` and filters unknown identity, invalid confidence, and non-finite positions.
- Known camera identity creates an immediately confirmed track, bypassing `min_hits_to_confirm` for that semantic slot.
- Repeated camera observations update the existing `(team, semantic_class)` track. LiDAR updates continue using the same tracker and therefore preserve identity metadata.
- Unknown tracks never populate official output slots.
- Official output is selected by team and semantic class, not track order. Blue is mapped to opponent fields and red to ally fields under the existing convention.
- Map offsets remain `map_to_rm_offset_x=14.0` and `map_to_rm_offset_y=7.5`; conversion is now meters to centimeters with `*100.0`.
- Conversion rejects non-finite, negative, and `uint16_t`-overflow values by emitting zero.
- Runtime camera fusion defaults to enabled with `camera_timeout_sec=1.5`.

## TDD Evidence

### RED

The first implementation test run was executed in `radar:develop`:

```bash
docker run --rm --network host \
  -v "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.worktrees/competition-bringup:/workspace" \
  radar:develop bash -lc 'source /opt/ros/jazzy/setup.bash && cd /workspace/ros_ws && \
  colcon build --packages-select radar_interfaces radar_camera radar_fusion \
  --cmake-args -DBUILD_TESTING=ON'
```

The expected semantic test failure was:

```text
undefined reference to radar_camera::projection::Projector::proj_semantic_postprocess(...)
```

The same run also exposed an existing container/runtime issue for the full camera node:

```text
fatal error: hikcamera/shm.hpp: No such file or directory
```

The camera core test target does not require that missing staged SDK header and was subsequently built and run successfully.

### GREEN

Camera core:

```bash
docker run --rm --network host \
  -v "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.worktrees/competition-bringup:/workspace" \
  radar:develop bash -lc 'source /opt/ros/jazzy/setup.bash && cd /workspace/ros_ws && \
  colcon build --packages-select radar_camera --cmake-target radar_camera_tests \
  --cmake-args -DBUILD_TESTING=ON && ./build/radar_camera/radar_camera_tests --gtest_color=no'
```

Output: `22 tests from 7 test suites`, `22 passed`.

Fusion:

```bash
colcon build --packages-select radar_interfaces radar_fusion --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select radar_fusion --event-handlers console_direct+ --return-code-on-test-failure
```

Output: `23 tests from 3 test suites`, `23 passed`.

Important asserted values:

```text
meters_to_cm(1.25, 14.0) = 1525
meters_to_cm(-14.1, 14.0) = 0
meters_to_cm(infinity, 0.0) = 0
meters_to_cm(700.0, 0.0) = 0
```

Interfaces and bringup contract:

```bash
colcon build --packages-select radar_interfaces radar_fusion radar_bringup \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select radar_interfaces radar_fusion radar_bringup \
  --event-handlers console_direct+ --return-code-on-test-failure
```

Passing targets:

- `radar_interfaces`: `1/1` test passed.
- `radar_fusion`: `23/23` tests passed.
- `radar_bringup/test_competition_contract`: `7/7` passed.
- `radar_bringup/test_registration_gate`: `3/3` passed.
- `radar_bringup/test_competition_launch`: `14/14` passed.

Python message import and replay syntax check:

```bash
python3 -m py_compile src/radar_bringup/test/test_fusion_bridge_csv_replay.py
python3 -c "from radar_interfaces.msg import CameraDetection, CameraDetectionArray; print(CameraDetection.TEAM_BLUE, CameraDetection.CLASS_HERO)"
```

Output:

```text
2 1
```

## CSV Replay Status

The replay test was updated to publish `CameraDetectionArray` with:

- `hero_b`: blue hero
- `eng_r`: red engineer
- `inf3_b`: blue infantry 3

It now asserts:

- opponent hero fields equal the CSV `rm_x_cm/rm_y_cm` oracle;
- opponent infantry 3 fields equal the CSV oracle;
- ally engineer fields equal the CSV oracle;
- bridge JSON fields equal a matching ROS `LidarLocation` message.

The test could not execute in this checkout/container because collection stops before the test body:

```text
ModuleNotFoundError: No module named 'zmq'
```

The requested blue replay assets were also not present in the worktree:

```text
camera_ray_per_frame_blue.csv: not found
testdata/images: not found
```

Therefore no CSV oracle output values can honestly be reported from an executed replay run. The test is syntactically valid and its ROS message constants were imported successfully after sourcing the workspace overlay.

## Commits

- `b678bc1 test: cover semantic camera fusion flow`
- `ce43709 feat: bind camera semantics through fusion`

The report is intentionally committed separately after final verification.

## Concerns

- The full `radar_camera` executable still depends on the external Hikcamera SDK header `hikcamera/shm.hpp`; the `radar:develop` image used here does not expose it. The core camera tests pass.
- The CSV replay remains unverified end-to-end until `python3-zmq` and the blue CSV/image fixtures are available in the test environment.
- Existing fusion tests create multiple nodes with the same ROS node name and emit the pre-existing rosout publisher warning; all assertions pass.
- The worktree already contained unrelated modified Hikcamera submodules and untracked startup plan files. They were left untouched.
