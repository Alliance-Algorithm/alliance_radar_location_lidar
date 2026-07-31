# Competition Raw Camera Recording Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add optional full-resolution `5472x3648` raw RGB camera recording to `radar_camera` without modifying the Hikvision SDK, camera driver, or SHM writer, while keeping inference non-blocking and making recorder overrun an explicit failure.

**Architecture:** Add a second, read-only SHM consumer inside `radar_camera` using persistent mapping and the existing `frame_counter` stability protocol. Accepted frames move into a preallocated ordered FIFO; the FIFO never drops or overwrites accepted frames. An independent FFmpeg `h264_nvenc` worker converts and encodes frames to 60-second MPEG-TS segments. If SHM stability, FIFO capacity, encoding, or disk throughput cannot keep up, recording transitions to `OVERRUN` and stops instead of blocking inference or hiding a gap.

**Tech Stack:** C++23, ROS 2 Jazzy, OpenCV, Hikcamera SHM API (read-only), FFmpeg libavcodec/libavformat/libavutil/libswscale, NVIDIA NVENC, GTest, ffprobe.

## Global Constraints

- Do not modify `ros_ws/third-party/hikcamera_sdk`, `hikcamera_ros_driver`, or the SHM writer.
- Keep the existing inference `SHMRead()` path and model input dimensions unchanged.
- Recording is disabled by default.
- Raw recordings are unannotated full-resolution RGB source frames encoded as H.264 MPEG-TS.
- Accepted recording frames are ordered and are never silently dropped or overwritten.
- FIFO exhaustion, unstable SHM reads, encoder errors, and segment write errors transition recording to `OVERRUN`/failure; they never block the inference thread.
- Use `cv::Mat` move ownership transfer after the one unavoidable SHM-to-owned-buffer copy.
- Use `h264_nvenc` on the validated RTX 4060 host; do not default to software `libx264`.
- Use a fixed buffer-pool memory budget and reject configurations whose pool cannot hold the configured number of full-resolution frames.
- Verify with unit tests before hardware tests; hardware tests must use bounded duration and a writable non-system output directory.

---

## File Map

### New files

- `ros_ws/src/radar_camera/include/radar_camera/raw_shm_reader.hpp`: full-resolution SHM reader API, frame metadata, stable-copy helpers.
- `ros_ws/src/radar_camera/src/raw_shm_reader.cpp`: persistent mapping, frame-counter polling, stable RGB copy, reader lifecycle.
- `ros_ws/src/radar_camera/include/radar_camera/recording_fifo.hpp`: ordered, bounded FIFO ownership API and overrun state.
- `ros_ws/src/radar_camera/src/recording_fifo.cpp`: FIFO synchronization, buffer accounting, close/overrun transitions.
- `ros_ws/src/radar_camera/include/radar_camera/raw_video_recorder.hpp`: recording configuration, recorder lifecycle, statistics API.
- `ros_ws/src/radar_camera/src/raw_video_recorder.cpp`: FFmpeg initialization, RGB-to-YUV conversion, NVENC encode, MPEG-TS segmentation, sidecar metadata.
- `ros_ws/src/radar_camera/test/test_raw_shm_reader.cpp`: pure SHM slot/counter and frame validation tests.
- `ros_ws/src/radar_camera/test/test_recording_fifo.cpp`: move ownership, FIFO ordering, capacity, and overrun tests.
- `ros_ws/src/radar_camera/test/test_raw_video_recorder.cpp`: configuration, disabled lifecycle, segment naming, and encoder lifecycle tests; hardware tests are skipped unless explicitly enabled.

### Modified files

- `ros_ws/src/radar_camera/include/radar_camera/radar_camera_node.hpp`: own optional recorder and recording configuration/lifecycle state.
- `ros_ws/src/radar_camera/src/radar_camera_node.cpp`: declare/load recording parameters, start recorder after existing initialization, stop before SHM close, report recorder failure without joining it from inference work.
- `ros_ws/src/radar_camera/CMakeLists.txt`: find FFmpeg through `pkg_check_modules`, add recorder sources, link FFmpeg, install headers, register tests.
- `ros_ws/src/radar_camera/package.xml`: declare FFmpeg build/runtime dependency in the package metadata used by the deployment image.
- `ros_ws/src/radar_camera/config/radar_camera.yaml`: add disabled recording defaults and full-resolution recording parameters.
- `.github/workflows/build.yml`: install/check FFmpeg development packages and run the recorder unit tests if CI already provisions the dependency.

---

### Task 1: Define Recording Data Types And Configuration Validation

**Files:**
- Create: `ros_ws/src/radar_camera/include/radar_camera/raw_shm_reader.hpp`
- Create: `ros_ws/src/radar_camera/include/radar_camera/recording_fifo.hpp`
- Create: `ros_ws/src/radar_camera/include/radar_camera/raw_video_recorder.hpp`
- Test: `ros_ws/src/radar_camera/test/test_recording_fifo.cpp`
- Modify: `ros_ws/src/radar_camera/CMakeLists.txt`

**Interfaces:**
- Produce `radar_camera::recording::RawFrame { cv::Mat rgb; uint64_t sequence; uint64_t host_monotonic_ns; }`.
- Produce `radar_camera::recording::RecordingConfig { bool enabled; std::string output_dir; int width; int height; int fps; int bitrate; int gop; std::string encoder; int segment_duration_sec; size_t buffer_pool_frames; size_t max_buffer_bytes; }`.
- Produce `radar_camera::recording::validate_config(const RecordingConfig&) -> std::expected<void, std::string>`.
- Produce `RecordingFifo::try_push(RawFrame&&) -> bool`, `RecordingFifo::pop() -> std::optional<RawFrame>`, `RecordingFifo::request_overrun(std::string)`, `RecordingFifo::overrun() const`, `RecordingFifo::size() const`, and `RecordingFifo::close()`.

- [ ] **Step 1: Write failing FIFO tests**

Test that a moved `cv::Mat` is owned by the FIFO, frames pop in sequence order, an accepted frame is not overwritten at capacity, and exhaustion enters overrun without blocking or accepting another frame.

```cpp
TEST(RecordingFifo, PreservesOrderAndMoveOwnership) {
    RecordingFifo fifo(2);
    RawFrame first{make_image(1), 10, 100};
    ASSERT_TRUE(fifo.try_push(std::move(first)));
    ASSERT_TRUE(first.rgb.empty());
    ASSERT_TRUE(fifo.try_push(RawFrame{make_image(2), 11, 101}));
    EXPECT_EQ(fifo.pop()->sequence, 10);
    EXPECT_EQ(fifo.pop()->sequence, 11);
}

TEST(RecordingFifo, CapacityFailureTransitionsToOverrun) {
    RecordingFifo fifo(1);
    ASSERT_TRUE(fifo.try_push(RawFrame{make_image(1), 1, 1}));
    EXPECT_FALSE(fifo.try_push(RawFrame{make_image(2), 2, 2}));
    EXPECT_TRUE(fifo.overrun());
}
```

- [ ] **Step 2: Run the focused test and confirm failure**

Run from `ros_ws` after the package is configured:

```bash
colcon test --packages-select radar_camera --ctest-args -R radar_camera_tests --output-on-failure
```

Expected: compilation/test failure because the FIFO types do not exist.

- [ ] **Step 3: Implement the minimal FIFO and validators**

Use a mutex and condition variable only for recorder/FIFO coordination. `try_push` must never wait; when full it calls `request_overrun` and returns false. Validate positive even dimensions, positive FPS/bitrate/GOP/segment duration, `encoder == "h264_nvenc"`, non-empty output directory, positive pool count, and `width * height * 3 * pool_count <= max_buffer_bytes` with overflow-safe arithmetic.

- [ ] **Step 4: Run focused FIFO tests**

```bash
colcon test --packages-select radar_camera --ctest-args -R radar_camera_tests --output-on-failure
```

Expected: FIFO ordering, move ownership, capacity failure, and validation tests pass.

- [ ] **Step 5: Commit the data boundary**

```bash
git add ros_ws/src/radar_camera/include/radar_camera/raw_shm_reader.hpp ros_ws/src/radar_camera/include/radar_camera/recording_fifo.hpp ros_ws/src/radar_camera/include/radar_camera/raw_video_recorder.hpp ros_ws/src/radar_camera/test/test_recording_fifo.cpp ros_ws/src/radar_camera/CMakeLists.txt
git commit -m "feat(camera): define loss-aware recording buffers"
```

### Task 2: Implement The Independent Full-Resolution SHM Reader

**Files:**
- Modify: `ros_ws/src/radar_camera/include/radar_camera/raw_shm_reader.hpp`
- Create: `ros_ws/src/radar_camera/src/raw_shm_reader.cpp`
- Create: `ros_ws/src/radar_camera/test/test_raw_shm_reader.cpp`
- Modify: `ros_ws/src/radar_camera/CMakeLists.txt`

**Interfaces:**
- Produce `RawShmReader(std::string shm_name, int width, int height, RecordingFifo&)`.
- Produce `start() -> std::expected<void, std::string>`, `stop() -> void`, `state() -> ReaderState`, and `stats() -> ReaderStats`.
- Keep pure helpers testable: `completed_slot(uint64_t counter, unsigned int slot_num)`, `is_stable(uint64_t before, uint64_t after)`, and `validate_raw_frame_dimensions(int width, int height)`.

- [ ] **Step 1: Write failing reader helper tests**

Cover counter zero, completed slot `(counter - 1) % 4`, stable versus changing counters, dimensions, RGB byte count, and frame metadata preservation.

- [ ] **Step 2: Run the helper tests and confirm failure**

```bash
./build/radar_camera/radar_camera_tests --gtest_filter='RawShmReader*'
```

Expected: failure because the helper functions and reader do not exist.

- [ ] **Step 3: Implement persistent mapping and the reader loop**

Open the existing SHM with `hikcamera::SHMInit(shm_name, sizeof(hikcamera::imageSHM))`, map once with `SHMGetPtr`, and release pointer before closing the descriptor. Poll `frame_counter` without `SHMRead` or semaphore operations. For each new counter, copy exactly `width * height * 3` bytes into a preallocated pool-owned `cv::Mat`, read the slot timestamp, verify the counter is unchanged, and submit the frame to the FIFO using move semantics. On an unstable copy, request FIFO overrun rather than silently skipping it. Stop immediately if the FIFO reports overrun.

- [ ] **Step 4: Run reader and existing camera tests**

```bash
colcon test --packages-select radar_camera --ctest-args -R radar_camera_tests --output-on-failure
```

Expected: all pure reader, FIFO, model, and projector tests pass; no SDK/driver files are modified.

- [ ] **Step 5: Commit the SHM reader**

```bash
git add ros_ws/src/radar_camera/src/raw_shm_reader.cpp ros_ws/src/radar_camera/test/test_raw_shm_reader.cpp ros_ws/src/radar_camera/include/radar_camera/raw_shm_reader.hpp ros_ws/src/radar_camera/CMakeLists.txt
git commit -m "feat(camera): read full-resolution frames from shm"
```

### Task 3: Add FFmpeg NVENC Recorder And MPEG-TS Segments

**Files:**
- Modify: `ros_ws/src/radar_camera/include/radar_camera/raw_video_recorder.hpp`
- Create: `ros_ws/src/radar_camera/src/raw_video_recorder.cpp`
- Create: `ros_ws/src/radar_camera/test/test_raw_video_recorder.cpp`
- Modify: `ros_ws/src/radar_camera/CMakeLists.txt`
- Modify: `ros_ws/src/radar_camera/package.xml`
- Modify: `.github/workflows/build.yml`

**Interfaces:**
- Produce `RawVideoRecorder(RecordingConfig, RecordingFifo&)`.
- Produce `start() -> std::expected<void, std::string>`, `stop() -> void`, `state() -> RecorderState`, and `stats() -> RecorderStats`.
- Produce deterministic `segment_path(output_dir, session_start, segment_index) -> std::filesystem::path`.

- [ ] **Step 1: Verify FFmpeg development dependencies before coding**

Run:

```bash
pkg-config --modversion libavcodec libavformat libavutil libswscale
ffmpeg -hide_banner -encoders | grep h264_nvenc
```

Expected: all four development packages resolve and `h264_nvenc` is listed. If headers/libraries are missing, update the deployment image/CI dependency list before continuing; do not silently fall back to software encoding.

- [ ] **Step 2: Write failing recorder tests**

Test disabled start as a no-op, invalid configuration rejection, idempotent stop, deterministic segment names, FIFO order consumption, and an explicit skip for the hardware encode test when `RADAR_CAMERA_RUN_HW_RECORDING_TESTS` is not set.

- [ ] **Step 3: Run recorder tests and confirm failure**

```bash
./build/radar_camera/radar_camera_tests --gtest_filter='RawVideoRecorder*'
```

Expected: failure because FFmpeg recorder symbols do not exist.

- [ ] **Step 4: Implement FFmpeg lifecycle**

Use `avformat_alloc_output_context2` with an MPEG-TS format, create an H.264 video stream, find `h264_nvenc`, configure width/height/time base/GOP/bitrate, set `bf=0`, low-latency preset/options, open the output URL/path, and write the stream header. Use `sws_getContext` for RGB8 to `AV_PIX_FMT_YUV420P` conversion in the recorder thread. Send packets through `avcodec_send_frame`/`avcodec_receive_packet`, rescale timestamps, and write with `av_interleaved_write_frame`.

- [ ] **Step 5: Implement segment rollover and sidecar accounting**

Close the encoder and MPEG-TS trailer at the configured segment duration, increment the segment index, open the next path, and continue sequence timestamps without reordering. Write sidecar records with sequence, source monotonic timestamp, segment name, and cumulative overrun/error counters. Any open, encode, packet, or write error requests FIFO overrun/failure and stops the recorder.

- [ ] **Step 6: Run unit tests and a bounded synthetic hardware test**

```bash
colcon test --packages-select radar_camera --ctest-args -R radar_camera_tests --output-on-failure
RADAR_CAMERA_RUN_HW_RECORDING_TESTS=1 ./build/radar_camera/radar_camera_tests --gtest_filter='RawVideoRecorder.HardwareEncode*'
```

For hardware acceptance, feed a small bounded number of synthetic `5472x3648` RGB frames, then run:

```bash
ffprobe -v error -select_streams v:0 -show_entries stream=codec_name,width,height -of default=nw=1 /tmp/radar-camera-recording/*.ts
```

Expected: H.264, width `5472`, height `3648`, and a readable MPEG-TS segment.

- [ ] **Step 7: Commit the recorder**

```bash
git add ros_ws/src/radar_camera/src/raw_video_recorder.cpp ros_ws/src/radar_camera/test/test_raw_video_recorder.cpp ros_ws/src/radar_camera/include/radar_camera/raw_video_recorder.hpp ros_ws/src/radar_camera/CMakeLists.txt ros_ws/src/radar_camera/package.xml .github/workflows/build.yml
git commit -m "feat(camera): record raw frames with nvenc"
```

### Task 4: Integrate Recording Into `RadarCameraNode`

**Files:**
- Modify: `ros_ws/src/radar_camera/include/radar_camera/radar_camera_node.hpp`
- Modify: `ros_ws/src/radar_camera/src/radar_camera_node.cpp`
- Modify: `ros_ws/src/radar_camera/config/radar_camera.yaml`
- Modify: `ros_ws/src/radar_camera/CMakeLists.txt`

**Interfaces:**
- Load `enable_raw_recording`, `recording_output_dir`, `recording_width`, `recording_height`, `recording_fps`, `recording_bitrate`, `recording_gop`, `recording_encoder`, `recording_segment_duration_sec`, `recording_buffer_pool_frames`, and `recording_max_buffer_bytes`.
- Own `std::unique_ptr<RawVideoRecorder>` and `std::unique_ptr<RawShmReader>` with destruction order that stops reader/recorder before the existing inference SHM descriptor closes.

- [ ] **Step 1: Add failing node configuration tests**

Extend the camera contract tests to assert all parameters exist, defaults disable recording, disabled startup does not map a second SHM reader, and enabled startup rejects an invalid output directory or encoder.

- [ ] **Step 2: Run the contract tests and confirm failure**

```bash
./build/radar_camera/radar_camera_tests --gtest_filter='CameraRecordingContract*'
```

Expected: failure because recording parameters and lifecycle are not wired.

- [ ] **Step 3: Implement parameter loading and lifecycle wiring**

Declare exact defaults from the design. Validate the recording configuration before opening the recorder. Start `RawVideoRecorder` and `RawShmReader` only after existing model/projector/SHM initialization succeeds. In the node destructor, stop the reader first, then stop the recorder, then let the existing inference stop/descriptor close complete. Do not add a call to the recorder from `infer_thread_start`'s inference loop.

- [ ] **Step 4: Propagate recorder failure without blocking inference**

Expose recorder state and log a throttled fatal/error message when it becomes `OVERRUN` or `FAILED`. The launch integration must be able to observe the node's failure policy; no error may be hidden as a successful recording session. Inference should continue only if the competition launch policy explicitly allows recorder failure; otherwise return a non-zero process result through the existing node failure mechanism.

- [ ] **Step 5: Update YAML and run camera tests**

Add disabled defaults and full-resolution values:

```yaml
enable_raw_recording: false
recording_output_dir: "/data/competition/recordings"
recording_width: 5472
recording_height: 3648
recording_fps: 20
recording_bitrate: 40000000
recording_gop: 20
recording_encoder: "h264_nvenc"
recording_segment_duration_sec: 60
recording_buffer_pool_frames: 8
recording_max_buffer_bytes: 480000000
```

Run:

```bash
colcon test --packages-select radar_camera --event-handlers console_direct+ --return-code-on-test-failure
```

- [ ] **Step 6: Commit node integration**

```bash
git add ros_ws/src/radar_camera/include/radar_camera/radar_camera_node.hpp ros_ws/src/radar_camera/src/radar_camera_node.cpp ros_ws/src/radar_camera/config/radar_camera.yaml ros_ws/src/radar_camera/CMakeLists.txt
git commit -m "feat(camera): integrate raw recording lifecycle"
```

### Task 5: Build, Throughput-Test, And Document Competition Usage

**Files:**
- Modify: `ros_ws/docs/superpowers/specs/2026-07-31-competition-raw-camera-recording-design.md` only if implementation behavior changes.
- Modify: `ros_ws/docs/superpowers/plans/2026-07-31-competition-raw-camera-recording.md` only to mark completed steps during execution.
- Modify: `ros_ws/src/radar_bringup/config/camera/radar_camera.yaml` if competition uses a separate installed camera parameter file.
- Modify: `ros_ws/docs/superpowers/plans/2026-07-31-competition-bringup-integration.md` if the launch command must expose recording parameters.

- [ ] **Step 1: Build the package with tests**

```bash
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
colcon build --packages-select radar_camera --cmake-args -DBUILD_TESTING=ON
```

Expected: `radar_camera` builds with FFmpeg headers/libraries and no changes in SDK/driver packages.

- [ ] **Step 2: Run all package tests**

```bash
colcon test --packages-select radar_camera --event-handlers console_direct+ --return-code-on-test-failure
```

- [ ] **Step 3: Run a bounded full-resolution recording benchmark**

Start the existing camera driver and `radar_camera` with recording enabled for a bounded interval. Capture baseline and recording-enabled inference timing. Observe concurrently:

```bash
nvidia-smi dmon -s pucm -d 1
iostat -xz 1
```

Verify the recorder statistics show `accepted == encoded`, no `OVERRUN`, monotonically increasing source sequences, and no inference latency accumulation. If the configured 20 FPS cannot sustain this, fail the recording configuration and reduce recording FPS only if the requirement permits; never silently skip frames.

- [ ] **Step 4: Validate segments and metadata**

```bash
ffprobe -v error -show_streams /data/competition/recordings/*.ts
```

Verify every segment is decodable, has `5472x3648`, expected FPS, no B-frames, and matching sidecar sequence ranges.

- [ ] **Step 5: Validate disabled competition mode**

Launch the normal competition command with `enable_raw_recording:=false`. Verify no second SHM mapping/reader, encoder, output files, or recorder thread is created and existing inference behavior remains unchanged.

- [ ] **Step 6: Commit final documentation and verification notes**

```bash
git add ros_ws/src/radar_bringup/config/camera/radar_camera.yaml ros_ws/docs/superpowers/specs/2026-07-31-competition-raw-camera-recording-design.md ros_ws/docs/superpowers/plans/2026-07-31-competition-bringup-integration.md
git commit -m "docs: document competition raw recording verification"
```

## Plan Self-Review

- Spec coverage: SDK/driver boundary, full-resolution SHM copy, ordered no-silent-drop FIFO, explicit overrun, NVENC, MPEG-TS segmentation, disabled mode, sidecar metadata, tests, and hardware throughput checks are covered by Tasks 1-5.
- Placeholder scan: no incomplete or unspecified implementation steps remain; the only environment-dependent value is the explicit hardware-test environment variable.
- Type consistency: `RawFrame`, `RecordingConfig`, `RecordingFifo`, `RawShmReader`, and `RawVideoRecorder` signatures are defined before their consumers and use the same names throughout.
- Constraint check: the design does not promise both unbounded no-drop recording and zero backpressure; it uses a fixed pool and explicit overrun failure to protect inference.
