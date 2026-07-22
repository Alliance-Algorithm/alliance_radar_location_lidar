# Hik SHM FAST-LIVO2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver Hik camera frames from the SHM-only driver directly to FAST-LIVO2 as 2736x1824 mono8 images with broadcast multi-consumer semantics.

**Architecture:** Replace the shared-consumer cursor with a four-slot broadcast ring. Each slot is protected by a process-shared read/write lock; a condition variable announces new sequences. FAST-LIVO2 and radar_bridge each own a persistent reader cursor.

**Tech Stack:** C++23, pthread process-shared synchronization, POSIX SHM, OpenCV, ROS2 Jazzy, ament_cmake.

## Global Constraints

- Do not add a ROS image relay or fallback input path.
- Do not copy a full 5472x3648 BGR frame in a consumer.
- Default VIO output is 2736x1824 mono8.
- Preserve device ticks without assuming their frequency.
- Do not modify estimator mathematics in this plan.

---

### Task 1: Broadcast SHM protocol

**Files:**
- Modify: `ros_ws/third-party/hikcamera_sdk/include/hikcamera/capturer.hpp`
- Modify: `ros_ws/third-party/hikcamera_sdk/include/hikcamera/shm.hpp`
- Modify: `ros_ws/third-party/hikcamera_sdk/src/capturer.impl.hpp`
- Modify: `ros_ws/third-party/hikcamera_sdk/src/shm.cpp`
- Modify: `ros_ws/third-party/hikcamera_sdk/CMakeLists.txt`
- Create: `ros_ws/third-party/hikcamera_sdk/test/test_shm.cpp`

**Interfaces:**
- Produces: `FrameMetadata`, `SharedFrame`, `SharedFrameReader::open()`,
  `SharedFrameReader::wait_next()`, and metadata-rich `Camera::Image`.

- [ ] Write tests that initialize a temporary SHM object, publish synthetic sequences, open two
  readers, and assert both readers observe the same sequence and metadata.
- [ ] Add a writer-wrap test that holds a read lease while later slots are published and verifies
  the leased pixels remain unchanged.
- [ ] Run the focused test and verify the old API fails to compile.
- [ ] Implement slot metadata, process-shared rwlocks, condition broadcast, persistent mapping,
  and RAII read leases.
- [ ] Extend `Camera::Image` with frame ID, dimensions, stride, device ticks, and exposure time;
  populate them from `MV_FRAME_OUT_INFO_EX`.
- [ ] Run the focused test and verify all broadcast and lease cases pass.

### Task 2: Camera driver writer integration

**Files:**
- Modify: `ros_ws/third-party/hikcamera_ros_driver/src/camera_bridge.cpp`
- Modify: `ros_ws/third-party/hikcamera_ros_driver/include/hikcamera_ros_driver/camera_bridge.hpp`

**Interfaces:**
- Consumes: metadata-rich `hikcamera::SHMWrite` from Task 1.
- Produces: `/hikcamera_shm` protocol version with BGR pixels and paired metadata.

- [ ] Build `hikcamera_ros_driver` against the new SDK and capture the expected compile failure.
- [ ] Replace obsolete SHM pointer/lifecycle calls with the new writer interface.
- [ ] Rebuild and verify the driver links without restoring a ROS image publisher.

### Task 3: radar_bridge independent reader

**Files:**
- Modify: `ros_ws/src/radar_bridge/include/radar_bridge/videostream_bridge.hpp`
- Modify: `ros_ws/src/radar_bridge/src/videostream_bridge.cpp`

**Interfaces:**
- Consumes: `SharedFrameReader` and `SharedFrame` from Task 1.
- Produces: the existing JPEG ZMQ stream.

- [ ] Build `radar_bridge` and capture the expected failure after removing legacy `SHMRead`.
- [ ] Store one persistent reader in `VideoBridge`; wait for independent latest sequences and
  encode the leased BGR view directly.
- [ ] Rebuild and verify no per-frame `mmap`, `munmap`, or full-frame `clone` remains.

### Task 4: FAST-LIVO2 image preprocessing unit

**Files:**
- Create: `ros_ws/src/radar_fast_livo2/include/radar_fast_livo2/shm_camera.hpp`
- Create: `ros_ws/src/radar_fast_livo2/src/shm_camera.cpp`
- Create: `ros_ws/src/radar_fast_livo2/test/test_shm_camera.cpp`
- Modify: `ros_ws/src/radar_fast_livo2/CMakeLists.txt`
- Modify: `ros_ws/src/radar_fast_livo2/package.xml`

**Interfaces:**
- Consumes: `hikcamera::SharedFrameReader`.
- Produces: `CameraFrame {cv::Mat gray; uint64_t sequence; FrameMetadata metadata;}` and
  `ShmCamera::wait_next()`.

- [ ] Write a failing test using a synthetic 5472x3648 BGR slot and assert the output is
  2736x1824 `CV_8UC1`, with unchanged sequence and timestamps.
- [ ] Add a test that calls twice without a new sequence and verifies no duplicate frame is
  returned.
- [ ] Implement direct leased BGR to half-resolution mono conversion without a full BGR clone.
- [ ] Run the focused test and verify both geometry and single-consumption semantics pass.

### Task 5: FAST-LIVO2 node integration

**Files:**
- Modify: `ros_ws/src/radar_fast_livo2/src/livmapper_node.cpp`
- Modify: `ros_ws/src/radar_fast_livo2/config/odin_livo2.yaml`
- Modify: `ros_ws/src/radar_fast_livo2/README.md`

**Interfaces:**
- Consumes: `ShmCamera` from Task 4.
- Produces: one queued mono frame per SHM sequence for VIO.

- [ ] Remove `sensor_msgs/Image`, `cv_bridge`, the ROS image subscription, and nearest ROS-image
  cache.
- [ ] Add `camera/shm_name`, source geometry, and default 2736x1824 parameters.
- [ ] Start one SHM reader thread in LIVO mode and stop/join it with node lifetime.
- [ ] Store only the latest owned grayscale frame; consume each sequence once in `process_frame`.
- [ ] Preserve `img_time_offset` by applying it to host monotonic seconds without inventing a
  device-tick conversion.
- [ ] Build and run focused tests.

### Task 6: Integrated verification

**Files:**
- Verify all files from Tasks 1-5.

- [ ] Run SDK SHM tests.
- [ ] Run FAST-LIVO2 camera tests.
- [ ] Build `hikcamera`, `hikcamera_ros_driver`, `radar_bridge`, and `radar_fast_livo2` together.
- [ ] Run static searches proving no consumer calls legacy `SHMRead`, no ROS image publisher was
  added, and no 60 MB BGR clone exists.
- [ ] Run LSP diagnostics on all changed C++ files.
- [ ] Review the final diff for scope: SHM/image input only, no estimator or fallback changes.
