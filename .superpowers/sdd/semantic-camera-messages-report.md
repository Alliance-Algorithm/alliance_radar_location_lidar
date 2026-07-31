# Semantic Camera Messages Report

Date: 2026-07-31
Worktree: `competition-bringup`
Container: `radar:develop`

## Scope

Added the first semantic fusion interface unit to `radar_interfaces`:

- `CameraDetection.msg`
- `CameraDetectionArray.msg`
- Generated-interface contract test and `ament_cmake_pytest` wiring

No camera, fusion, bridge, ZMQ, or documentation source files were modified. Existing unrelated worktree changes were left untouched.

## Contract

`CameraDetection.msg` contains:

- Team constants: `TEAM_UNKNOWN`, `TEAM_RED`, `TEAM_BLUE`
- Semantic class constants: `CLASS_UNKNOWN`, `CLASS_HERO`, `CLASS_ENGINEER`, `CLASS_INFANTRY_3`, `CLASS_INFANTRY_4`, `CLASS_AERIAL`, `CLASS_SENTRY`
- `uint8 team`
- `uint8 semantic_class`
- `geometry_msgs/Point position`
- `float32 confidence`

`CameraDetectionArray.msg` contains `std_msgs/Header header` and `CameraDetection[] detections`.

The team and class domains both require an `UNKNOWN` value. ROS message constants share one namespace, so the constants use `TEAM_` and `CLASS_` prefixes to represent the two distinct `UNKNOWN` values without a name collision.

## TDD Evidence

### RED

Before adding the messages or generator registration, the interface test was run in a fresh `radar:develop` container:

```text
python3 -m pytest -q src/radar_interfaces/test/test_camera_detection_interfaces.py
```

It failed during collection with:

```text
ModuleNotFoundError: No module named 'radar_interfaces'
```

This was the expected failure because the generated message package did not yet exist.

### GREEN

After adding the two messages, generator registration, package dependencies, and test registration:

```text
colcon build --packages-select radar_interfaces --cmake-args -DBUILD_TESTING=ON
```

Result: `radar_interfaces` built successfully.

```text
python3 -m pytest -q src/radar_interfaces/test/test_camera_detection_interfaces.py
```

Result: `1 passed`.

```text
colcon test --packages-select radar_interfaces --event-handlers console_direct+ --return-code-on-test-failure
```

Result: `1/1` CTest targets passed, `0` failed.

## Concerns

- Sourcing the existing workspace overlay emits the pre-existing warning `not found: "/workspace/ros_ws/install/radar_camera/share/radar_camera/local_setup.bash"`; it did not prevent this package from building or testing.
- The requested team/class `UNKNOWN` constants cannot both be exposed as a bare `UNKNOWN` constant in one ROS message, so the names are explicitly domain-prefixed.
