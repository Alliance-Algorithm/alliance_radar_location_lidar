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
    radar_camera::armor_refine::ArmorRefineConfig armor;
    radar_camera::armor_refine::NumberRefineConfig number;

    ASSERT_TRUE(radar_camera::node::ConfigsLoader(
        *node, camera, inference, projection, recording, armor, number));
    EXPECT_FALSE(recording.enabled);
    EXPECT_EQ(recording.output_dir, "/model/devio");
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
    radar_camera::armor_refine::ArmorRefineConfig armor;
    radar_camera::armor_refine::NumberRefineConfig number;

    const auto result = radar_camera::node::ConfigsLoader(
        *node, camera, inference, projection, recording, armor, number);
    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("recording"), std::string::npos);
}

TEST_F(CameraRecordingContract, StartsInferenceBeforeRecordingLifecycle) {
    const auto events = radar_camera::node::recording_lifecycle_order();

    ASSERT_EQ(events,
        (std::vector<radar_camera::node::LifecycleComponent> {
            radar_camera::node::LifecycleComponent::inference,
            radar_camera::node::LifecycleComponent::recorder,
            radar_camera::node::LifecycleComponent::reader,
            radar_camera::node::LifecycleComponent::monitor }));
}

TEST_F(CameraRecordingContract, ConstructorCleanupStopsEveryStartedComponent) {
    const auto cleanup =
        radar_camera::node::constructor_cleanup_order({ radar_camera::node::LifecycleComponent::shm,
            radar_camera::node::LifecycleComponent::inference,
            radar_camera::node::LifecycleComponent::recorder,
            radar_camera::node::LifecycleComponent::reader,
            radar_camera::node::LifecycleComponent::monitor });

    EXPECT_EQ(cleanup,
        (std::vector<radar_camera::node::LifecycleComponent> {
            radar_camera::node::LifecycleComponent::monitor,
            radar_camera::node::LifecycleComponent::reader,
            radar_camera::node::LifecycleComponent::recorder,
            radar_camera::node::LifecycleComponent::inference,
            radar_camera::node::LifecycleComponent::shm }));
}

TEST_F(CameraRecordingContract, ConstructorCleanupClosesShmWhenInferenceNeverStarts) {
    const auto cleanup = radar_camera::node::constructor_cleanup_order(
        { radar_camera::node::LifecycleComponent::shm });

    EXPECT_EQ(cleanup,
        (std::vector<radar_camera::node::LifecycleComponent> {
            radar_camera::node::LifecycleComponent::shm }));
}

TEST_F(CameraRecordingContract, DisabledRecordingHasNoComponentsWithoutHardware) {
    radar_camera::recording::RecordingConfig config { };
    config.enabled = false;

    const auto components = radar_camera::node::make_recording_components(config, "unused");

    EXPECT_FALSE(components.fifo);
    EXPECT_FALSE(components.recorder);
    EXPECT_FALSE(components.reader);
}

} // namespace
