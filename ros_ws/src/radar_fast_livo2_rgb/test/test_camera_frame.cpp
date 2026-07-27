// test_camera_frame.cpp — Unit tests for camera-frame helpers
//
// Task 4 of the FAST-LIVO2 RGB map/replay plan.
// Tests timestamp conversion from SHM monotonic clock to ROS system time
// and BGR8 Image message construction from cv::Mat without pixel reordering.
//
// These tests exercise pure helper functions with no hardware or SHM dependency.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "radar_fast_livo2_rgb/camera_frame.hpp"
#include "radar_fast_livo2_rgb/pcd_trigger.hpp"

using namespace radar::fast_livo2::rgb;

// ════════════════════════════════════════════════════════════════════════
// Timestamp conversion (brief Step 1)
// ════════════════════════════════════════════════════════════════════════

TEST(CameraFrame, ConvertsSharedMetadataToRosStamp)
{
    const uint64_t host_monotonic_ns = 1'000'000'000ULL;
    const auto stamp = camera_stamp_from_shm(host_monotonic_ns, 1700000000.25);
    EXPECT_EQ(stamp.sec, 1700000001);
    EXPECT_EQ(stamp.nanosec, 250000000U);
}

// ════════════════════════════════════════════════════════════════════════
// BGR8 Image creation (brief Step 1)
// ════════════════════════════════════════════════════════════════════════

TEST(CameraFrame, CreatesBgr8ImageWithoutChangingPixelOrder)
{
    cv::Mat bgr(1, 1, CV_8UC3, cv::Scalar(3, 2, 1));
    const auto image = make_bgr8_image(bgr, builtin_interfaces::msg::Time{}, "camera");
    EXPECT_EQ(image.encoding, "bgr8");
    EXPECT_EQ(image.data, std::vector<uint8_t>({3, 2, 1}));
}

// ════════════════════════════════════════════════════════════════════════
// Additional: zero monotonic offset
// ════════════════════════════════════════════════════════════════════════

TEST(CameraFrame, ZeroOffsetReturnsMonotonicStamp)
{
    const uint64_t host_monotonic_ns = 500'000'000ULL;  // 0.5 s
    const auto stamp = camera_stamp_from_shm(host_monotonic_ns, 0.0);
    EXPECT_EQ(stamp.sec, 0);
    EXPECT_EQ(stamp.nanosec, 500000000U);
}

// ════════════════════════════════════════════════════════════════════════
// Additional: multi-channel BGR image preserves full row data
// ════════════════════════════════════════════════════════════════════════

TEST(CameraFrame, MultiPixelBgrImagePreservesRowMajorOrder)
{
    // 2×2 BGR image: row0={B0,G0,R0, B1,G1,R1}, row1={B2,G2,R2, B3,G3,R3}
    cv::Mat bgr(2, 2, CV_8UC3);
    bgr.at<cv::Vec3b>(0, 0) = cv::Vec3b(10, 20, 30);
    bgr.at<cv::Vec3b>(0, 1) = cv::Vec3b(40, 50, 60);
    bgr.at<cv::Vec3b>(1, 0) = cv::Vec3b(70, 80, 90);
    bgr.at<cv::Vec3b>(1, 1) = cv::Vec3b(100, 110, 120);

    builtin_interfaces::msg::Time stamp;
    stamp.sec = 42;
    stamp.nanosec = 123456789U;

    const auto image = make_bgr8_image(bgr, stamp, "test_frame");

    EXPECT_EQ(image.header.stamp.sec, 42);
    EXPECT_EQ(image.header.stamp.nanosec, 123456789U);
    EXPECT_EQ(image.header.frame_id, "test_frame");
    EXPECT_EQ(image.height, 2u);
    EXPECT_EQ(image.width, 2u);
    EXPECT_EQ(image.encoding, "bgr8");
    EXPECT_FALSE(image.is_bigendian);
    EXPECT_EQ(image.step, 6u);  // 2 cols × 3 channels

    // Raw data: BGR rows in row-major order
    const std::vector<uint8_t> expected = {
        10, 20, 30,  40, 50, 60,    // row 0
        70, 80, 90, 100, 110, 120   // row 1
    };
    EXPECT_EQ(image.data, expected);
}

// ════════════════════════════════════════════════════════════════════════
// Additional: sub-second-only offset
// ════════════════════════════════════════════════════════════════════════

TEST(CameraFrame, SubSecondOffsetDoesNotAdvanceSecondsBoundary)
{
    const uint64_t host_monotonic_ns = 100'000'000ULL;  // 0.1 s
    const auto stamp = camera_stamp_from_shm(host_monotonic_ns, 0.3);
    EXPECT_EQ(stamp.sec, 0);
    EXPECT_EQ(stamp.nanosec, 400000000U);
}

// ════════════════════════════════════════════════════════════════════════
// Additional: rollover from nanoseconds to seconds
// ════════════════════════════════════════════════════════════════════════

TEST(CameraFrame, NanosecondsRolloverToNextSecond)
{
    const uint64_t host_monotonic_ns = 800'000'000ULL;  // 0.8 s
    const auto stamp = camera_stamp_from_shm(host_monotonic_ns, 0.5);
    EXPECT_EQ(stamp.sec, 1);
    EXPECT_EQ(stamp.nanosec, 300000000U);
}

// ════════════════════════════════════════════════════════════════════════
// Regression: L4-6 — calibration rejects nonpositive cx/cy
// ════════════════════════════════════════════════════════════════════════

TEST(CameraFrame, CalibrationRejectsZeroCx)
{
    auto err = validate_calibration(
        100.0, 100.0, 0.0, 50.0,
        {0.1, 0.01, 0.005, -0.003, 0.001});
    ASSERT_TRUE(err.has_value());
    EXPECT_NE(err->find("cx"), std::string::npos);
}

TEST(CameraFrame, CalibrationRejectsZeroCy)
{
    auto err = validate_calibration(
        100.0, 100.0, 50.0, 0.0,
        {0.1, 0.01, 0.005, -0.003, 0.001});
    ASSERT_TRUE(err.has_value());
    EXPECT_NE(err->find("cy"), std::string::npos);
}

TEST(CameraFrame, CalibrationRejectsNegativeCx)
{
    auto err = validate_calibration(
        100.0, 100.0, -10.0, 50.0,
        {0.1, 0.01, 0.005, -0.003, 0.001});
    ASSERT_TRUE(err.has_value());
    EXPECT_NE(err->find("cx"), std::string::npos);
}

TEST(CameraFrame, CalibrationRejectsNegativeCy)
{
    auto err = validate_calibration(
        100.0, 100.0, 50.0, -10.0,
        {0.1, 0.01, 0.005, -0.003, 0.001});
    ASSERT_TRUE(err.has_value());
    EXPECT_NE(err->find("cy"), std::string::npos);
}

TEST(CameraFrame, CalibrationAcceptsValidPositiveValues)
{
    auto err = validate_calibration(
        100.0, 100.0, 50.0, 40.0,
        {0.1, 0.01, 0.005, -0.003, 0.001});
    EXPECT_FALSE(err.has_value());
}

// ════════════════════════════════════════════════════════════════════════
// Regression: M4-3 — overflow-safe image size validation
// ════════════════════════════════════════════════════════════════════════

TEST(CameraFrame, ImageSizeRejectsZeroWidth)
{
    auto err = validate_image_size(0, 100, 60'000'000);
    ASSERT_TRUE(err.has_value());
    EXPECT_NE(err->find("width"), std::string::npos);
}

TEST(CameraFrame, ImageSizeRejectsZeroHeight)
{
    auto err = validate_image_size(100, 0, 60'000'000);
    ASSERT_TRUE(err.has_value());
    EXPECT_NE(err->find("height"), std::string::npos);
}

TEST(CameraFrame, ImageSizeRejectsNegativeWidth)
{
    auto err = validate_image_size(-1, 100, 60'000'000);
    ASSERT_TRUE(err.has_value());
    EXPECT_NE(err->find("width"), std::string::npos);
}

TEST(CameraFrame, ImageSizeAcceptsValid5472x3648)
{
    // 5472 * 3648 * 3 = 59'885'568 ≤ 60'000'000
    auto err = validate_image_size(5472, 3648, 60'000'000);
    EXPECT_FALSE(err.has_value());
}

TEST(CameraFrame, ImageSizeRejectsExceedsMax)
{
    // 5472 * 3648 * 3 = 59'885'568 > 59'000'000
    auto err = validate_image_size(5472, 3648, 59'000'000);
    ASSERT_TRUE(err.has_value());
    EXPECT_NE(err->find("exceeds"), std::string::npos);
}

TEST(CameraFrame, ImageSizeAcceptsMaxIntWidth)
{
    // Extreme but valid: INT_MAX × 2 × 3 fits within a generous slot cap.
    // The overflow check in validate_image_size is defensive — for int
    // parameters w*h fits in size_t, so the check can't be triggered
    // through the public int interface.  This test verifies the function
    // does not crash or reject extreme valid inputs.
    auto err = validate_image_size(std::numeric_limits<int>::max(), 2,
        static_cast<size_t>(std::numeric_limits<int>::max()) * 2 * 3);
    EXPECT_FALSE(err.has_value());
}

// ════════════════════════════════════════════════════════════════════════
// F1: One-copy pipeline — fill_bgr8_image_metadata
// ════════════════════════════════════════════════════════════════════════

TEST(CameraFrame, FillMetadataSetsEncodingAndDimensions)
{
    sensor_msgs::msg::Image image;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = 99;
    stamp.nanosec = 888000000U;

    fill_bgr8_image_metadata(image, 1920, 1080, stamp, "cam_link");

    EXPECT_EQ(image.header.stamp.sec, 99);
    EXPECT_EQ(image.header.stamp.nanosec, 888000000U);
    EXPECT_EQ(image.header.frame_id, "cam_link");
    EXPECT_EQ(image.height, 1080u);
    EXPECT_EQ(image.width, 1920u);
    EXPECT_EQ(image.encoding, "bgr8");
    EXPECT_FALSE(image.is_bigendian);
    EXPECT_EQ(image.step, 5760u);  // 1920 * 3
}

TEST(CameraFrame, FillMetadataDoesNotTouchData)
{
    sensor_msgs::msg::Image image;
    image.data = {0xAA, 0xBB, 0xCC, 0xDD};  // pre-existing data

    fill_bgr8_image_metadata(image, 1, 1,
                             builtin_interfaces::msg::Time{}, "");

    // Metadata is set but data is untouched
    EXPECT_EQ(image.data, std::vector<uint8_t>({0xAA, 0xBB, 0xCC, 0xDD}));
    EXPECT_EQ(image.encoding, "bgr8");
}

TEST(CameraFrame, OneCopyPipelineRoundTrip)
{
    // Simulate the full one-copy path: pre-allocate, copy bytes in,
    // then fill metadata.  Verifies BGR byte order is preserved.
    const int w = 2, h = 2;
    sensor_msgs::msg::Image image;
    image.data.resize(static_cast<size_t>(w) * h * 3);

    // Fake "SHM data": row-major RGB (from PixelType_Gvsp_RGB8_Packed)
    const std::vector<uint8_t> shm_bytes = {
        10, 20, 30,  40, 50, 60,
        70, 80, 90, 100, 110, 120
    };
    std::memcpy(image.data.data(), shm_bytes.data(), shm_bytes.size());

    builtin_interfaces::msg::Time stamp;
    stamp.sec = 7;
    stamp.nanosec = 250000000U;
    fill_bgr8_image_metadata(image, w, h, stamp, "cam");

    EXPECT_EQ(image.header.stamp.sec, 7);
    EXPECT_EQ(image.header.stamp.nanosec, 250000000U);
    EXPECT_EQ(image.header.frame_id, "cam");
    EXPECT_EQ(image.height, 2u);
    EXPECT_EQ(image.width, 2u);
    EXPECT_EQ(image.encoding, "bgr8");
    EXPECT_EQ(image.step, 6u);
    EXPECT_EQ(image.data, shm_bytes);  // BGR byte order preserved
}

// ════════════════════════════════════════════════════════════════════════
// F2 race fix: completed-slot derivation from frame_counter
// ════════════════════════════════════════════════════════════════════════

TEST(CameraFrame, CompletedSlotIsCounterMinusOneModSlotNum)
{
    EXPECT_EQ(completed_slot_from_counter(1, 4), 0u);
    EXPECT_EQ(completed_slot_from_counter(2, 4), 1u);
    EXPECT_EQ(completed_slot_from_counter(3, 4), 2u);
    EXPECT_EQ(completed_slot_from_counter(4, 4), 3u);
}

TEST(CameraFrame, CompletedSlotWrapsAfterSlotNumFrames)
{
    EXPECT_EQ(completed_slot_from_counter(5, 4), 0u);
    EXPECT_EQ(completed_slot_from_counter(8, 4), 3u);
    EXPECT_EQ(completed_slot_from_counter(9, 4), 0u);
}

TEST(CameraFrame, CompletedSlotWithLargeCounter)
{
    EXPECT_EQ(completed_slot_from_counter(1'000'001, 4), 0u);
    EXPECT_EQ(completed_slot_from_counter(1'000'002, 4), 1u);
    EXPECT_EQ(completed_slot_from_counter(1'000'004, 4), 3u);
}

TEST(CameraFrame, CounterZeroIsInvalidFrame)
{
    EXPECT_FALSE(is_valid_frame_counter(0));
    EXPECT_TRUE(is_valid_frame_counter(1));
    EXPECT_TRUE(is_valid_frame_counter(999));
}

TEST(CameraFrame, FrameStableWhenCounterUnchanged)
{
    EXPECT_TRUE(is_frame_stable(42, 42));
    EXPECT_FALSE(is_frame_stable(42, 43));
    EXPECT_FALSE(is_frame_stable(42, 41));
}

// ════════════════════════════════════════════════════════════════════════
// Semaphore elimination: should_process_counter + state progression
// ════════════════════════════════════════════════════════════════════════

TEST(CameraFrame, ShouldProcessWhenCounterAdvanced)
{
    EXPECT_TRUE(should_process_counter(5, 4));
    EXPECT_TRUE(should_process_counter(1, 0));
    EXPECT_TRUE(should_process_counter(100, 99));
}

TEST(CameraFrame, ShouldNotProcessWhenCounterUnchanged)
{
    EXPECT_FALSE(should_process_counter(5, 5));
    EXPECT_FALSE(should_process_counter(42, 42));
}

TEST(CameraFrame, ShouldNotProcessWhenCounterIsZero)
{
    EXPECT_FALSE(should_process_counter(0, 0));
    EXPECT_FALSE(should_process_counter(0, 5));
}

TEST(CameraFrame, ShouldProcessWhenCounterRegressed)
{
    // Monotonic counter never regresses in normal operation, but if
    // it does (counter < last_seen), the value still differs from
    // last_seen — we should process to recover state.
    EXPECT_TRUE(should_process_counter(3, 5));
}

TEST(CameraFrame, StateProgressionAfterStableCopy)
{
    // Simulate: counter was 5, last_seen was 4.
    // After stable copy: last_seen advances to counter_after (5).
    uint64_t last_seen = 4;
    uint64_t counter = 5;
    ASSERT_TRUE(should_process_counter(counter, last_seen));

    uint64_t counter_before = counter;  // 5, seen in retry loop
    ASSERT_TRUE(is_valid_frame_counter(counter_before));
    // ... copy happens, post-copy check passes ...
    uint64_t counter_after = counter_before;  // unchanged
    ASSERT_TRUE(is_frame_stable(counter_before, counter_after));

    last_seen = counter_after;  // advance
    EXPECT_EQ(last_seen, 5u);
    EXPECT_FALSE(should_process_counter(5, last_seen));  // no re-process
}

TEST(CameraFrame, StateProgressionAfterUnstableRetry)
{
    // Simulate: counter is 7, but during copy writer advances to 9.
    // After retries exhausted, last_seen = counter_after (9),
    // skipping past the unstable region.
    uint64_t last_seen = 6;
    ASSERT_TRUE(should_process_counter(7, last_seen));

    // First retry: counter_before=7, writer advances to 9 during copy
    uint64_t cb1 = 7, ca1 = 9;
    EXPECT_FALSE(is_frame_stable(cb1, ca1));

    // Second retry: counter_before=9, but writer advances to 10
    uint64_t cb2 = 9, ca2 = 10;
    EXPECT_FALSE(is_frame_stable(cb2, ca2));

    // Third retry: also unstable
    uint64_t cb3 = 10, ca3 = 11;
    EXPECT_FALSE(is_frame_stable(cb3, ca3));

    // Retries exhausted: advance past latest seen
    last_seen = ca3;  // 11
    EXPECT_EQ(last_seen, 11u);
    EXPECT_FALSE(should_process_counter(11, last_seen));
}

// ════════════════════════════════════════════════════════════════════════
// M5: PCD trigger transition logic
// ════════════════════════════════════════════════════════════════════════

TEST(PcdTriggerTransition, NoTriggerReturnsFalse)
{
    EXPECT_FALSE(pcd_trigger_transition(false, 0));
    EXPECT_FALSE(pcd_trigger_transition(false, -1));
}

TEST(PcdTriggerTransition, SuccessResetsTrigger)
{
    EXPECT_FALSE(pcd_trigger_transition(true, 0));
}

TEST(PcdTriggerTransition, FailureRetainsTrigger)
{
    EXPECT_TRUE(pcd_trigger_transition(true, -1));
    EXPECT_TRUE(pcd_trigger_transition(true, 1));
    EXPECT_TRUE(pcd_trigger_transition(true, 42));
}

TEST(PcdTriggerTransition, IdempotentWhenAlreadyFalse)
{
    EXPECT_FALSE(pcd_trigger_transition(false, 0));
    EXPECT_FALSE(pcd_trigger_transition(false, 0));
    EXPECT_FALSE(pcd_trigger_transition(false, -1));
    EXPECT_FALSE(pcd_trigger_transition(false, -1));
}

// ════════════════════════════════════════════════════════════════════
// Regression: RGB SHM → BGR published byte-order via cvtColor
// ════════════════════════════════════════════════════════════════════

TEST(CameraFrame, ShmRGBtoPublishedBgrByteOrder)
{
    // Simulate the one-pass cvtColor pipeline used by the recorder:
    // SHM source is RGB (PixelType_Gvsp_RGB8_Packed), output is BGR.
    const int w = 2, h = 1;
    sensor_msgs::msg::Image image_msg;
    image_msg.data.resize(static_cast<size_t>(w) * h * 3);

    // Fake SHM RGB data: two pixels, R=10,G=20,B=30 | R=40,G=50,B=60
    // In SHM memory: [10, 20, 30, 40, 50, 60]  (R,G,B per pixel)
    const std::vector<uint8_t> shm_rgb = {10, 20, 30, 40, 50, 60};

    // Wrap SHM as cv::Mat (no copy)
    cv::Mat rgb_shm(h, w, CV_8UC3, const_cast<uint8_t*>(shm_rgb.data()));
    // Wrap preallocated ROS image data as cv::Mat (no copy)
    cv::Mat bgr_out(h, w, CV_8UC3, image_msg.data.data());
    // One-pass transform: RGB → BGR
    cv::cvtColor(rgb_shm, bgr_out, cv::COLOR_RGB2BGR);

    builtin_interfaces::msg::Time stamp;
    stamp.sec = 0; stamp.nanosec = 0;
    fill_bgr8_image_metadata(image_msg, w, h, stamp, "cam");

    EXPECT_EQ(image_msg.encoding, "bgr8");
    // After RGB→BGR: [B=30,G=20,R=10, B=60,G=50,R=40]
    const std::vector<uint8_t> expected_bgr = {30, 20, 10, 60, 50, 40};
    EXPECT_EQ(image_msg.data, expected_bgr);
}

TEST(CameraFrame, ShmRGBtoPublishedBgrByteOrder2x2)
{
    // 2×2 test: verifies row-major order is preserved through cvtColor.
    const int w = 2, h = 2;
    sensor_msgs::msg::Image image_msg;
    image_msg.data.resize(static_cast<size_t>(w) * h * 3);

    // SHM RGB data (row-major):
    // Row 0: R10,G20,B30 | R40,G50,B60
    // Row 1: R70,G80,B90 | R100,G110,B120
    const std::vector<uint8_t> shm_rgb = {
        10, 20, 30,  40, 50, 60,
        70, 80, 90, 100, 110, 120
    };

    cv::Mat rgb_shm(h, w, CV_8UC3, const_cast<uint8_t*>(shm_rgb.data()));
    cv::Mat bgr_out(h, w, CV_8UC3, image_msg.data.data());
    cv::cvtColor(rgb_shm, bgr_out, cv::COLOR_RGB2BGR);

    builtin_interfaces::msg::Time stamp;
    stamp.sec = 0; stamp.nanosec = 0;
    fill_bgr8_image_metadata(image_msg, w, h, stamp, "cam");

    EXPECT_EQ(image_msg.encoding, "bgr8");
    EXPECT_EQ(image_msg.step, 6u);
    // After RGB→BGR:
    // Row 0: B30,G20,R10 | B60,G50,R40
    // Row 1: B90,G80,R70 | B120,G110,R100
    const std::vector<uint8_t> expected_bgr = {
        30, 20, 10,  60, 50, 40,
        90, 80, 70, 120, 110, 100
    };
    EXPECT_EQ(image_msg.data, expected_bgr);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
