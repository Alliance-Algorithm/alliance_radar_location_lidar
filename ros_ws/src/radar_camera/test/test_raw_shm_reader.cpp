#include <cstdint>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "radar_camera/raw_shm_reader.hpp"

namespace radar_camera::recording {

TEST(RawShmReader, CounterZeroHasNoCompletedSlot) {
    EXPECT_FALSE(valid_frame_counter(0));
    EXPECT_TRUE(valid_frame_counter(1));
}

TEST(RawShmReader, CompletedSlotUsesThePreviouslyPublishedCounter) {
    EXPECT_EQ(completed_slot(1, 4), 0U);
    EXPECT_EQ(completed_slot(2, 4), 1U);
    EXPECT_EQ(completed_slot(5, 4), 0U);
    EXPECT_EQ(completed_slot(17, 4), 0U);
}

TEST(RawShmReader, StabilityRequiresAnUnchangedCounter) {
    EXPECT_TRUE(is_stable(12, 12));
    EXPECT_FALSE(is_stable(12, 13));
}

TEST(RawShmReader, ValidatesDimensionsAndRgbByteCount) {
    EXPECT_TRUE(validate_raw_frame_dimensions(4, 2));
    EXPECT_FALSE(validate_raw_frame_dimensions(0, 2));
    EXPECT_FALSE(validate_raw_frame_dimensions(4, -1));
    EXPECT_FALSE(validate_raw_frame_dimensions(5473, 3648));

    ASSERT_EQ(raw_frame_byte_count(4, 2), 24U);
    EXPECT_FALSE(raw_frame_byte_count(0, 2));
}

TEST(RawShmReader, RawFramePreservesImageAndMetadata) {
    cv::Mat image(2, 3, CV_8UC3, cv::Scalar(7, 8, 9));
    constexpr std::uint64_t sequence = 42;
    constexpr std::uint64_t host_ns = 123456789;

    RawFrame frame{ image.clone(), sequence, host_ns };

    EXPECT_EQ(frame.rgb.rows, 2);
    EXPECT_EQ(frame.rgb.cols, 3);
    const auto pixel = frame.rgb.at<cv::Vec3b>(0, 0);
    EXPECT_EQ(pixel[0], 7);
    EXPECT_EQ(pixel[1], 8);
    EXPECT_EQ(pixel[2], 9);
    EXPECT_EQ(frame.sequence, sequence);
    EXPECT_EQ(frame.host_monotonic_ns, host_ns);
}

} // namespace radar_camera::recording
