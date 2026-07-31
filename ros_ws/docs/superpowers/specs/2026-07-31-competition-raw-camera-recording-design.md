# Competition Raw Camera Recording Design

## Goal

Add optional full-resolution camera recording to competition mode without modifying the Hikvision SDK, camera driver, or SHM writer, and without making the inference thread wait for video encoding or disk I/O.

The recorder consumes the existing `/hikcamera_shm` ring buffer as an independent read-only consumer, captures stable `5472x3648` RGB8 frames, and writes low-latency H.264 files through the host's NVIDIA NVENC encoder.

## Non-goals

- No changes to the Hikvision SDK, camera driver, SHM layout, or SHM writer.
- No changes to the existing inference image size or inference algorithm.
- No annotation rendering in the raw recording path.
- No change to `radar_bridge` or coordinate transport.
- No requirement that every camera frame be recorded when encoding or storage cannot keep up.

## Existing Boundary

`radar_camera` currently opens `/hikcamera_shm` and calls `hikcamera::SHMRead()` from its inference thread. That call returns an image resized for model input, so the resulting `cv::Mat` is not the full-resolution camera frame and must not be used for the raw recorder.

The raw recorder therefore opens the same SHM object independently and uses the already-established read-only protocol used elsewhere in the workspace:

1. Persistently map the SHM object with `SHMInit` and `SHMGetPtr`.
2. Poll `frame_counter` with acquire loads without consuming the SHM semaphore.
3. Select the completed slot `(counter - 1) % SLOT_NUM`.
4. Copy the full RGB8 slot into owned memory.
5. Re-read `frame_counter`; accept only if it is unchanged.
6. Retry a bounded number of times, then drop the unstable frame.

The SDK and driver remain untouched. The SHM copy is unavoidable because the ring slot may be overwritten after the read.

## Architecture

```text
/hikcamera_shm
      |
      +--> existing radar_camera SHMRead
      |       |
      |       +--> resized model image -> inference -> projection
      |
      +--> RawShmReader thread
              |
              +--> stable 5472x3648 RGB8 owned frame
              +--> latest-only queue
                              |
                              v
                       NVENC recorder thread
                              |
                              +--> RGB8 to encoder input conversion
                              +--> h264_nvenc
                              +--> MPEG-TS segment files
```

The inference thread and recorder thread share no blocking operation. Recording is disabled by default and is started/stopped with the `radar_camera` node.

## Components

### `RawShmReader`

Owns the persistent SHM mapping and a dedicated reader thread. It emits an owned full-resolution RGB8 frame with:

- SHM sequence/frame counter;
- source monotonic timestamp;
- capture dimensions;
- read/drop status counters.

It never returns a pointer into SHM. It does not call a semaphore-consuming API. On instability it drops the frame and continues.

### `LatestFrameQueue`

Provides a non-blocking `try_replace_latest(Frame&&)` operation. Capacity is one full-resolution frame by default. If the encoder is busy, the previous pending frame is replaced and counted as dropped. The queue never waits for the encoder.

The queue owns frames after move. The reader can reuse its next buffer only after ownership has been transferred or the frame was rejected.

### `RawVideoRecorder`

Owns one encoder thread and the output segment lifecycle. It consumes the latest frame, converts RGB8 to the encoder's required YUV format, submits frames to `h264_nvenc`, and writes MPEG-TS segments.

The recorder uses the local NVIDIA hardware discovered during design validation: GeForce RTX 4060 Laptop GPU, FFmpeg `h264_nvenc`, CUDA-capable driver. Encoder initialization failure is fatal when recording is explicitly enabled. A missing UDP receiver is irrelevant because this feature writes local files.

### `RadarCameraNode`

Loads recording parameters, starts the recorder after required camera/inference initialization succeeds, and stops it before closing the existing SHM descriptor. It does not encode, write files, or wait on the recorder from the inference loop.

## Configuration

Recording parameters are explicit and disabled by default:

```yaml
enable_raw_recording: false
recording_output_dir: /data/competition/recordings
recording_width: 5472
recording_height: 3648
recording_fps: 20
recording_bitrate: 40000000
recording_gop: 20
recording_encoder: h264_nvenc
recording_segment_duration_sec: 60
recording_queue_capacity: 1
```

The source dimensions must match the SHM producer configuration. Output dimensions must remain full resolution and even. The first implementation does not resize raw recordings; any mismatch is a startup configuration error rather than a silent crop or scale.

The output directory must exist or be created by the recorder, be writable, and have enough available space for at least one segment. The recorder stops safely and reports an error when a segment cannot be opened or written.

## Output Format

The raw recording is unannotated H.264 in MPEG-TS segments:

```text
competition-YYYYMMDD-HHMMSS-000000.ts
competition-YYYYMMDD-HHMMSS-000001.ts
```

The encoder uses no B-frames, a short GOP, and low-latency NVENC settings. The exact preset may be selected from validated runtime values, with `p1` as the low-overhead default. The recorder writes one segment for the configured duration and closes it before opening the next segment.

Each recording session writes a small JSONL or CSV sidecar containing segment name, sequence, source timestamp, encoded timestamp, and cumulative dropped-frame counters. This metadata is diagnostic only and does not affect inference.

## Performance Rules

- The inference thread never performs full-resolution copying, color conversion, encoding, or disk writes for recording.
- The reader performs one unavoidable SHM-to-owned-memory copy per accepted recording frame.
- Ownership transfer from reader to queue uses `cv::Mat` move semantics and does not copy pixels.
- The queue capacity is one by default and uses latest-frame-wins replacement.
- Queue-full and unstable-SHM cases drop recording frames rather than blocking inference or SHM consumption.
- Full-resolution RGB buffers are released immediately after conversion/encoder submission.
- Software `libx264` is not the default and is rejected unless explicitly added as a separate deployment profile.
- Recording statistics include read frames, accepted frames, encoded frames, dropped unstable frames, replaced queue frames, encoder failures, and maximum queue age.

## Failure Handling

When recording is disabled, no raw reader, recording queue, encoder, or output directory work is started.

When recording is enabled:

- Invalid dimensions, FPS, bitrate, GOP, encoder name, output path, or queue capacity fail startup.
- SHM mapping failure fails startup.
- NVENC initialization failure fails startup.
- A transient unstable SHM slot drops only that recording frame.
- Queue replacement drops only the older pending recording frame.
- A segment open/write or encoder failure stops recording and causes the camera node to report a fatal recording error according to the competition launch policy.
- Stopping the node first stops input, then drains or closes the encoder, then releases the SHM mapping.

Inference remains independent while the recorder is healthy. Recording is an explicitly enabled competition dependency, so enabled-mode initialization failures must not be silently ignored.

## File Organization

The implementation belongs in `radar_camera`:

```text
include/radar_camera/raw_shm_reader.hpp
include/radar_camera/latest_frame_queue.hpp
include/radar_camera/raw_video_recorder.hpp
src/raw_shm_reader.cpp
src/latest_frame_queue.cpp
src/raw_video_recorder.cpp
test/test_latest_frame_queue.cpp
```

The existing `radar_camera_node.cpp` should only wire lifecycle and configuration. It should not become the owner of encoding details.

## Verification

Unit tests must cover:

- completed-slot selection and stable-frame acceptance;
- unstable frame retry and drop;
- queue move ownership and latest-only replacement;
- disabled mode creating no reader or encoder;
- invalid recording configuration rejection;
- encoder lifecycle and idempotent stop;
- segment naming and rollover;
- metadata counters and dropped-frame accounting.

Hardware acceptance must verify:

1. `ffmpeg -hide_banner -encoders` contains `h264_nvenc`.
2. The recorder writes a decodable full-resolution H.264 MPEG-TS segment.
3. `ffprobe` reports `5472x3648` and the expected frame rate.
4. Inference FPS and P95/P99 inference latency are measured with recording disabled and enabled.
5. `nvidia-smi dmon` confirms encoder use and observes GPU memory/utilization.
6. Disk throughput and segment rollover remain stable for a representative competition interval.
7. When the encoder is intentionally slowed, recording frames are dropped but inference latency does not accumulate.

## Acceptance Criteria

- SDK, driver, and SHM writer have no changes.
- Competition can run with raw recording disabled exactly as before.
- Enabled recording produces full-resolution unannotated files.
- Inference thread has no wait on recording operations.
- Backlog cannot create unbounded latency.
- Recording resource failures are observable and actionable.
