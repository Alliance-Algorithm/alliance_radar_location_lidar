#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "radar_camera/radar_camera_node.hpp"

namespace {

class CameraRecordingContract : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        int argc = 0;
        rclcpp::init(argc, nullptr);
    }

    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(CameraRecordingContract, DeclaresDisabledRecordingDefaults) {
    auto node = std::make_shared<rclcpp::Node>("camera_recording_contract");
    radar_camera::camera_config::CameraConfig camera;
    radar_camera::inference_config::InferenceConfig inference;
    radar_camera::projection_config::ProjectionConfig projection;
    radar_camera::recording::RecordingConfig recording { };

    ASSERT_TRUE(radar_camera::node::ConfigsLoader(*node, camera, inference, projection, recording));
    EXPECT_FALSE(recording.enabled);
    EXPECT_EQ(recording.output_dir, "/data/competition/recordings");
    EXPECT_EQ(recording.width, 5472);
    EXPECT_EQ(recording.height, 3648);
    EXPECT_EQ(recording.fps, 20);
    EXPECT_EQ(recording.bitrate, 40000000);
    EXPECT_EQ(recording.gop, 20);
    EXPECT_EQ(recording.encoder, "h264_nvenc");
    EXPECT_EQ(recording.segment_duration_sec, 60);
    EXPECT_EQ(recording.buffer_pool_frames, 8U);
    EXPECT_EQ(recording.max_buffer_bytes, 480000000U);
}

TEST_F(CameraRecordingContract, RejectsInvalidEnabledRecordingConfiguration) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({ rclcpp::Parameter("enable_raw_recording", true),
        rclcpp::Parameter("recording_output_dir", ""),
        rclcpp::Parameter("recording_encoder", "libx264") });
    auto node = std::make_shared<rclcpp::Node>("camera_recording_contract_invalid", options);

    radar_camera::camera_config::CameraConfig camera;
    radar_camera::inference_config::InferenceConfig inference;
    radar_camera::projection_config::ProjectionConfig projection;
    radar_camera::recording::RecordingConfig recording { };

    const auto result =
        radar_camera::node::ConfigsLoader(*node, camera, inference, projection, recording);
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("recording"), std::string::npos);
}

TEST_F(CameraRecordingContract, StartsInferenceBeforeRecordingLifecycle) {
    const std::vector<std::string> events =
        radar_camera::node::RadarCameraNode::recording_lifecycle_order_for_test();

    ASSERT_EQ(events, (std::vector<std::string> { "inference", "recorder", "reader", "monitor" }));
}

TEST_F(CameraRecordingContract, ConstructorCleanupStopsEveryStartedComponent) {
    const auto cleanup = radar_camera::node::RadarCameraNode::constructor_cleanup_for_test(
        { "shm", "inference", "recorder", "reader", "monitor" });

    EXPECT_EQ(cleanup,
        (std::vector<std::string> { "monitor", "reader", "recorder", "inference", "shm" }));
}

TEST_F(CameraRecordingContract, ConstructorCleanupClosesShmWhenInferenceNeverStarts) {
    const auto cleanup = radar_camera::node::RadarCameraNode::constructor_cleanup_for_test({ "sh"
                                                                                             "m" });

    EXPECT_EQ(cleanup, (std::vector<std::string> { "shm" }));
}

} // namespace
