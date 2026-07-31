# Vision-First Semantic Fusion Report

Date: 2026-07-31
Worktree: `/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.worktrees/competition-bringup`
Branch: `feat/competition-bringup`
Container: `radar:develop`

## Result

Implemented loosely coupled semantic fusion:

- Valid semantic camera detections establish official centimeter slots immediately.
- LiDAR updates only an existing semantic identity when all coordinates are finite and the nearest point is within `camera_lidar_consistency_distance`.
- Missing LiDAR never changes the trusted camera position.
- Far LiDAR is rejected and cannot overwrite the trusted camera position.
- After camera timeout, close LiDAR can maintain and update the historical semantic identity until `identity_retention_sec` expires.
- Expired slots are removed and publish zero in their official fields.
- Unknown LiDAR never creates semantic official slots.

The typed `CameraDetectionArray` contract, `/lidar/location` message, radar bridge, and ZMQ protocol were unchanged. H264 was not implemented. The architecture document now reflects the independent-node topology, GICP registration/static TF handoff, and fusion ownership of `/lidar/location`.

## Configuration

`radar_fusion` declares and loads:

```yaml
camera_lidar_consistency_distance: 1.0
identity_retention_sec: 1.5
```

Both values are present in the installed `ros_ws/src/radar_fusion/config/runtime.yaml` and are validated as finite positive values.

## TDD Evidence

### RED

Command:

```bash
docker run --rm --network host \
  -v "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.worktrees/competition-bringup:/workspace" \
  radar:develop bash -lc 'source /opt/ros/jazzy/setup.bash && cd /workspace/ros_ws && \
  colcon build --packages-select radar_interfaces radar_fusion --cmake-args -DBUILD_TESTING=ON && \
  colcon test --packages-select radar_fusion --event-handlers console_direct+ --return-code-on-test-failure'
```

Observed output:

```text
[==========] Running 29 tests from 3 test suites.
[  PASSED  ] 24 tests.
[  FAILED  ] 5 tests
```

The five new position tests failed against the previous tracker-derived official output. The failure was behavioral, not a compile or collection error.

### GREEN

Final focused command:

```bash
docker run --rm --network host \
  -v "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.worktrees/competition-bringup:/workspace" \
  radar:develop bash -lc 'source /opt/ros/jazzy/setup.bash && cd /workspace/ros_ws && \
  colcon build --packages-select radar_interfaces radar_fusion --cmake-args -DBUILD_TESTING=ON && \
  colcon test --packages-select radar_fusion --event-handlers console_direct+ --return-code-on-test-failure'
```

Observed output:

```text
[==========] 29 tests from 3 test suites ran.
[  PASSED  ] 29 tests.
100% tests passed, 0 tests failed out of 1
```

The six requested semantic behaviors are covered by:

- `CameraOnlySemanticDetectionPublishesOfficialSlotImmediately`
- `CloseLidarUpdatesExistingSemanticSlot`
- `MissingLidarRetainsTrustedCameraPosition`
- `FarLidarDoesNotOverwriteTrustedCameraPosition`
- `CloseLidarRetainsIdentityAfterCameraTimeout`
- `SemanticSlotExpiresAfterIdentityRetention`

## Full Container Verification

Command:

```bash
docker run --rm --network host \
  -v "/home/yukikaze/Documents/workspace/alliance_radar_location_lidar/.worktrees/competition-bringup:/workspace" \
  -v "/testdata:/testdata:ro" \
  -e RADAR_CAMERA_RAY_CSV=/testdata/csv/camera_ray_per_frame_blue.csv \
  -e RADAR_REPLAY_IMAGE_DIR=/testdata/images \
  radar:develop bash -lc 'source /opt/ros/jazzy/setup.bash && cd /workspace/ros_ws && \
  colcon build --packages-select radar_interfaces radar_fusion radar_bridge radar_bringup \
    --cmake-args -DBUILD_TESTING=ON && \
  colcon test --packages-select radar_interfaces radar_fusion radar_bringup \
    --event-handlers console_direct+ --return-code-on-test-failure'
```

Observed successful targets before replay collection:

```text
radar_interfaces: 1/1 passed
radar_fusion: 29/29 passed
test_competition_contract: 7/7 passed
test_registration_gate: 3/3 passed
test_competition_launch: 14/14 passed
```

The first replay attempt was blocked by the container missing Python `zmq`:

```text
ModuleNotFoundError: No module named 'zmq'
```

After installing `python3-zmq` in the container, the replay source collected but exposed two startup-message races. The harness was updated to ignore initial zero-valued `/lidar/location` messages and wait for the expected semantic payload. A subsequent run without the temporary package install again failed at collection because the package is not present in the base image; therefore end-to-end replay is not claimed as passed.

The replay source passed:

```bash
python3 -m py_compile src/radar_bringup/test/test_fusion_bridge_csv_replay.py
```

The replay harness now explicitly covers camera-only publication and a far-LiDAR update after an empty camera frame. It retains the blue fixture mounts and environment variables shown above. The core `radar_fusion` tests and all non-replay bringup tests pass; replay requires a container image with Python `zmq` installed.

## T-DT_Radar Comparison

Conceptually this follows the T-DT_Radar direction of treating vision semantics as the identity-bearing source and using geometric sensing as supporting evidence. This implementation is deliberately adapted to this repository's contracts:

- Camera semantics publish directly into the existing official `LidarLocation` slots.
- LiDAR is a gated position maintenance/correction source, not an identity source.
- A large camera-LiDAR deviation is rejected instead of allowing the auxiliary sensor to move the trusted semantic slot.
- Camera dropout retains semantic identity only for the explicit configured interval.
- No T-DT_Radar code, message, or bridge protocol was copied.

## Concerns

- End-to-end CSV/ZMQ replay remains unverified in the base `radar:develop` image because it lacks the Python `zmq` module at collection time. The harness is syntax-checked and keeps `/testdata` blue CSV/image mounts.
- Existing tests emit the pre-existing ROS warning about repeated `radar_fusion_node` names; assertions pass.
- The worktree contains unrelated modified Hikcamera submodules and untracked startup plans; they were not staged or changed.
