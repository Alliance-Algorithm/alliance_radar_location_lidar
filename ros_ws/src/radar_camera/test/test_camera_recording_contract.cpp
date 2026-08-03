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

TEST_F(CameraRecordingContract, DoesNotDeclareRecordingParameters) {
    auto node = std::make_shared<rclcpp::Node>("camera_recording_contract");
    radar_camera::camera_config::CameraConfig camera;
    radar_camera::inference_config::InferenceConfig inference;
    radar_camera::projection_config::ProjectionConfig projection;
    radar_camera::armor_refine::ArmorRefineConfig armor;
    radar_camera::armor_refine::NumberRefineConfig number;

    ASSERT_TRUE(radar_camera::node::ConfigsLoader(
        *node, camera, inference, projection, armor, number));
    EXPECT_FALSE(node->has_parameter("enable_raw_recording"));
    EXPECT_FALSE(node->has_parameter("recording_output_dir"));
    EXPECT_FALSE(node->has_parameter("recording_fps"));
    EXPECT_FALSE(node->has_parameter("recording_encoder"));
}

TEST_F(CameraRecordingContract, StartsInferenceBeforeShmOpen) {
    const auto events = radar_camera::node::lifecycle_order();

    ASSERT_EQ(events,
        (std::vector<radar_camera::node::LifecycleComponent> {
            radar_camera::node::LifecycleComponent::inference,
            radar_camera::node::LifecycleComponent::shm }));
}

TEST_F(CameraRecordingContract, ConstructorCleanupStopsEveryStartedComponent) {
    const auto cleanup = radar_camera::node::constructor_cleanup_order(
        { radar_camera::node::LifecycleComponent::shm,
            radar_camera::node::LifecycleComponent::inference });

    EXPECT_EQ(cleanup,
        (std::vector<radar_camera::node::LifecycleComponent> {
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

} // namespace
