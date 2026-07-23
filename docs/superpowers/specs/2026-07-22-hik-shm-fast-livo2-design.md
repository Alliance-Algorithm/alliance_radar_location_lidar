# Hik SHM to FAST-LIVO2 Design

## Goal

Feed Hikrobot MV-CS200-10UC frames directly from the existing SHM-only camera driver into
`radar_fast_livo2`, without restoring a ROS image relay and without copying a full 5472x3648
BGR frame per consumer.

## Decisions

- The camera driver remains the sole camera owner and the sole SHM writer.
- The writer converts Bayer directly into one SHM slot. This is the only full-frame BGR write.
- SHM is a broadcast latest-frame buffer. Consumers never mutate a shared read cursor.
- Each slot has a process-shared read/write lock. A reader holds a read lease while OpenCV reads
  the slot; the writer takes a write lease before reusing it.
- A process-shared condition variable broadcasts each newly committed sequence to all readers.
- Each reader stores its own last consumed sequence locally and consumes a sequence at most once.
- `radar_fast_livo2` directly maps SHM and converts the leased BGR view to 2736x1824 mono8.
- `radar_bridge` uses an independent reader and continues JPEG/ZMQ output.
- No `sensor_msgs/Image` publisher or compatibility fallback is added.
- Camera device-clock calibration and rolling-shutter row timing are separate follow-up work.
  This implementation carries device ticks and host time but does not invent their conversion.

## SHM Protocol

`imageSHM` contains four `FrameSlot` objects and shared publication state.

Each slot stores:

- process-shared `pthread_rwlock_t`;
- committed sequence;
- frame ID;
- width, height, row stride, and pixel format;
- device timestamp ticks;
- host monotonic timestamp in nanoseconds;
- per-frame exposure time;
- one fixed-capacity pixel buffer.

The shared header stores:

- protocol magic and version;
- process-shared condition mutex and condition variable;
- atomically published latest sequence;
- initialized flag.

Writer order:

1. Select `next_sequence % SLOT_NUM`.
2. Acquire the slot write lock.
3. Ask MVS to convert Bayer directly into the slot pixel buffer.
4. Write metadata and slot sequence.
5. Release the slot lock.
6. Publish `latest_sequence` with release ordering.
7. Broadcast the condition variable.

Reader order:

1. Wait until `latest_sequence > local_last_sequence`.
2. Acquire a read lock on the slot for the observed latest sequence.
3. Verify the slot sequence still equals the observed sequence; retry with the newer latest
   sequence if the writer advanced before the lock was acquired.
4. Return an RAII frame lease containing a non-owning `cv::Mat` view and copied metadata.
5. Release the read lock when the lease is destroyed.

The writer may block only when all four slots are still leased. It never overwrites a slot being
read, so consumers never observe torn frames.

## FAST-LIVO2 Input

`LivMapperNode` owns a persistent `SharedFrameReader` and one reader thread. The thread converts
each new SHM sequence directly from BGR to a 2736x1824 mono8 image and stores the resulting small
owned grayscale frame with its metadata. The LIO processing callback consumes each grayscale
sequence at most once.

Default geometry:

- source: 5472x3648 BGR8;
- VIO image: 2736x1824 mono8;
- scale: 0.5 in both axes;
- intrinsics: calibrated native-resolution intrinsics multiplied by 0.5, unless calibration is
  performed directly at 2736x1824.

The host timestamp remains a monotonic post-conversion timestamp in this phase. `img_time_offset`
maps it to the LiDAR/IMU time domain. Device ticks are preserved for the later clock-calibration
phase.

## Scope

Modified components:

- `hikcamera_sdk`: SHM protocol, metadata, persistent reader, tests;
- `hikcamera_ros_driver`: writes the expanded frame metadata through the SDK API;
- `radar_bridge`: migrates to the persistent broadcast reader;
- `radar_fast_livo2`: direct SHM image input, grayscale/resize, configuration, tests.

Not included:

- rolling-shutter row-pose compensation;
- device-clock frequency discovery or host/device clock regression;
- ROS image publication;
- changes to the LIO/VIO estimator mathematics found in the separate algorithm review.

## Acceptance Criteria

- Two readers independently receive the same monotonically increasing sequences.
- A slow reader cannot expose a torn slot while the writer wraps the ring.
- No consumer clones a 5472x3648 BGR frame.
- FAST-LIVO2 produces exactly one 2736x1824 `CV_8UC1` frame for each consumed sequence.
- Frame sequence, timestamps, dimensions, and exposure metadata remain paired with their pixels.
- `hikcamera`, `hikcamera_ros_driver`, `radar_bridge`, and `radar_fast_livo2` build successfully.
