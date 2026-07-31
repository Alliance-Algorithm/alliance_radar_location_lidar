#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "radar_camera/raw_shm_reader.hpp"
#include "radar_camera/raw_video_recorder.hpp"
#include "radar_camera/recording_fifo.hpp"

namespace {

auto make_image(uchar value) -> cv::Mat {
    return cv::Mat(2, 2, CV_8UC3, cv::Scalar(value, value, value)).clone();
}

auto valid_config() -> radar_camera::recording::RecordingConfig {
    return {
        .enabled              = true,
        .output_dir           = "/tmp/recordings",
        .width                = 1920,
        .height               = 1080,
        .fps                  = 30,
        .bitrate              = 8000000,
        .gop                  = 30,
        .encoder              = "h264_nvenc",
        .segment_duration_sec = 60,
        .buffer_pool_frames   = 4,
        .max_buffer_bytes     = 1920ULL * 1080ULL * 3ULL * 4ULL,
    };
}

} // namespace

TEST(RecordingFifo, PreservesOrderAndMoveOwnership) {
    radar_camera::recording::RecordingFifo fifo(2);
    radar_camera::recording::RawFrame first{ make_image(1), 10, 100 };
    ASSERT_TRUE(fifo.try_push(std::move(first)));
    ASSERT_TRUE(first.rgb.empty());
    ASSERT_TRUE(fifo.try_push(radar_camera::recording::RawFrame{ make_image(2), 11, 101 }));
    ASSERT_EQ(fifo.pop()->sequence, 10);
    ASSERT_EQ(fifo.pop()->sequence, 11);
}

TEST(RecordingFifo, CapacityFailureTransitionsToOverrun) {
    radar_camera::recording::RecordingFifo fifo(1);
    ASSERT_TRUE(fifo.try_push(radar_camera::recording::RawFrame{ make_image(1), 1, 1 }));
    EXPECT_FALSE(fifo.try_push(radar_camera::recording::RawFrame{ make_image(2), 2, 2 }));
    EXPECT_TRUE(fifo.overrun());
    EXPECT_EQ(fifo.size(), 1U);
    EXPECT_EQ(fifo.pop()->sequence, 1U);
    EXPECT_FALSE(fifo.pop().has_value());
}

TEST(RecordingFifo, ExplicitOverrunReasonDoesNotDiscardAcceptedFrames) {
    radar_camera::recording::RecordingFifo fifo(2);
    ASSERT_TRUE(fifo.try_push(radar_camera::recording::RawFrame{ make_image(1), 1, 1 }));
    fifo.request_overrun("consumer stopped");
    EXPECT_TRUE(fifo.overrun());
    EXPECT_EQ(fifo.size(), 1U);
    EXPECT_EQ(fifo.pop()->sequence, 1U);
    EXPECT_FALSE(fifo.try_push(radar_camera::recording::RawFrame{ make_image(2), 2, 2 }));
}

TEST(RecordingFifo, ClosePreventsFurtherPushesAndAllowsDrain) {
    radar_camera::recording::RecordingFifo fifo(1);
    ASSERT_TRUE(fifo.try_push(radar_camera::recording::RawFrame{ make_image(1), 1, 1 }));
    fifo.close();
    EXPECT_FALSE(fifo.try_push(radar_camera::recording::RawFrame{ make_image(2), 2, 2 }));
    EXPECT_EQ(fifo.pop()->sequence, 1U);
    EXPECT_FALSE(fifo.pop().has_value());
}

TEST(RecordingConfig, AcceptsValidConfiguration) {
    EXPECT_TRUE(radar_camera::recording::validate_config(valid_config()).has_value());
}

TEST(RecordingConfig, RejectsInvalidScalarValues) {
    auto cfg = valid_config();
    cfg.width = 1919;
    EXPECT_FALSE(radar_camera::recording::validate_config(cfg));
    cfg = valid_config();
    cfg.fps = 0;
    EXPECT_FALSE(radar_camera::recording::validate_config(cfg));
    cfg = valid_config();
    cfg.encoder = "libx264";
    EXPECT_FALSE(radar_camera::recording::validate_config(cfg));
}

TEST(RecordingConfig, RejectsEmptyOutputAndInsufficientBuffer) {
    auto cfg = valid_config();
    cfg.output_dir.clear();
    EXPECT_FALSE(radar_camera::recording::validate_config(cfg));
    cfg = valid_config();
    cfg.max_buffer_bytes -= 1;
    EXPECT_FALSE(radar_camera::recording::validate_config(cfg));
}

TEST(RecordingConfig, RejectsBufferSizeArithmeticOverflow) {
    auto cfg = valid_config();
    cfg.width = std::numeric_limits<int>::max() - 1;
    cfg.height = std::numeric_limits<int>::max() - 1;
    cfg.buffer_pool_frames = std::numeric_limits<size_t>::max();
    EXPECT_FALSE(radar_camera::recording::validate_config(cfg));
}
