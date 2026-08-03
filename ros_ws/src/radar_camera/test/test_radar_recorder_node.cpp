#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "radar_camera/radar_recorder_node.hpp"

namespace {

class RadarRecorderContract : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        int argc = 0;
        rclcpp::init(argc, nullptr);
    }

    static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(RadarRecorderContract, LoadsRecordingDefaults) {
    auto node = std::make_shared<rclcpp::Node>("radar_recorder_contract");
    radar_camera::recording::RecordingConfig recording { };
    std::string shm_name;
    int width  = 0;
    int height = 0;

    ASSERT_TRUE(radar_camera::recording_node::RecordingConfigsLoader(
        *node, recording, shm_name, width, height));
    EXPECT_FALSE(recording.enabled);
    EXPECT_EQ(recording.output_dir, "/workspace/model/video");
    EXPECT_EQ(recording.width, 5472);
    EXPECT_EQ(recording.height, 3648);
    EXPECT_EQ(recording.fps, 8);
    EXPECT_EQ(recording.bitrate, 25000000);
    EXPECT_EQ(recording.gop, 20);
    EXPECT_EQ(recording.encoder, "hevc_nvenc");
    EXPECT_EQ(recording.segment_duration_sec, 0);  // 0 = 整段录制
    EXPECT_EQ(recording.buffer_pool_frames, 8U);
    EXPECT_EQ(recording.max_buffer_bytes, 480000000U);
    EXPECT_EQ(shm_name, "/hikcamera_shm");
    EXPECT_EQ(width, 5472);
    EXPECT_EQ(height, 3648);
}

TEST_F(RadarRecorderContract, RejectsInvalidEnabledRecording) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({ rclcpp::Parameter("enable_raw_recording", true),
        rclcpp::Parameter("recording_output_dir", "") });
    auto node = std::make_shared<rclcpp::Node>("radar_recorder_contract_invalid", options);

    radar_camera::recording::RecordingConfig recording { };
    std::string shm_name;
    int width  = 0;
    int height = 0;

    const auto result = radar_camera::recording_node::RecordingConfigsLoader(
        *node, recording, shm_name, width, height);
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("recording"), std::string::npos);
}

TEST_F(RadarRecorderContract, StartsRecorderBeforeReaderBeforeMonitor) {
    const auto events = radar_camera::recording_node::lifecycle_order();

    ASSERT_EQ(events,
        (std::vector<radar_camera::recording_node::LifecycleComponent> {
            radar_camera::recording_node::LifecycleComponent::recorder,
            radar_camera::recording_node::LifecycleComponent::reader,
            radar_camera::recording_node::LifecycleComponent::monitor }));
}

TEST_F(RadarRecorderContract, CleanupStopsEveryStartedComponentInReverse) {
    const auto cleanup = radar_camera::recording_node::cleanup_order(
        { radar_camera::recording_node::LifecycleComponent::recorder,
            radar_camera::recording_node::LifecycleComponent::reader,
            radar_camera::recording_node::LifecycleComponent::monitor });

    EXPECT_EQ(cleanup,
        (std::vector<radar_camera::recording_node::LifecycleComponent> {
            radar_camera::recording_node::LifecycleComponent::monitor,
            radar_camera::recording_node::LifecycleComponent::reader,
            radar_camera::recording_node::LifecycleComponent::recorder }));
}

TEST_F(RadarRecorderContract, DisabledRecordingHasNoComponents) {
    radar_camera::recording::RecordingConfig config { };
    config.enabled = false;

    const auto components = radar_camera::recording_node::make_components(config, "unused");

    EXPECT_FALSE(components.fifo);
    EXPECT_FALSE(components.recorder);
    EXPECT_FALSE(components.reader);
}

TEST_F(RadarRecorderContract, EnabledRecordingBuildsAllComponents) {
    radar_camera::recording::RecordingConfig config { };
    config.enabled  = true;
    config.fps      = 8;
    config.encoder  = "libx264";
    config.width    = 1920;
    config.height   = 1080;
    config.bitrate  = 8000000;
    config.gop      = 30;

    const auto components = radar_camera::recording_node::make_components(config, "/hikcamera_shm");

    EXPECT_TRUE(components.fifo);
    EXPECT_TRUE(components.recorder);
    EXPECT_TRUE(components.reader);
}

} // namespace
