#include "radar_bridge/radar_bridge_node.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace radar_bridge::node {
namespace {

class RadarBridgeConfigTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) rclcpp::shutdown();
    }
};

TEST_F(RadarBridgeConfigTest, DeclaresExistingParametersWithLegacyDefaults) {
    auto node = std::make_shared<rclcpp::Node>("radar_bridge_config_defaults");
    BridgeConfig config;

    auto result = ConfigsLoader(*node, config);

    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(node->has_parameter("zmq_pub_address"));
    EXPECT_TRUE(node->has_parameter("zmq_sub_addresses"));
    EXPECT_TRUE(node->has_parameter("shm_name"));
    EXPECT_TRUE(node->has_parameter("video_pub_address"));
    EXPECT_TRUE(node->has_parameter("image_topic"));
    EXPECT_TRUE(node->has_parameter("video_width"));
    EXPECT_TRUE(node->has_parameter("video_height"));
    EXPECT_TRUE(node->has_parameter("enable_video_stream"));
    EXPECT_EQ(config.zmq_pub_address, "tcp://*:5555");
    EXPECT_EQ(config.zmq_sub_addresses, std::vector<std::string> { "tcp://localhost:5556" });
    EXPECT_EQ(config.shm_name, "/hikcamera_shm");
    EXPECT_EQ(config.video_pub_address, "tcp://*:5557");
    EXPECT_EQ(config.image_topic, "/hikcamera_image");
    EXPECT_EQ(config.video_width, 4096);
    EXPECT_EQ(config.video_height, 3000);
    EXPECT_TRUE(config.enable_video_stream);
}

TEST_F(RadarBridgeConfigTest, LoadsParameterOverrides) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("zmq_pub_address", "tcp://127.0.0.1:6001"),
        rclcpp::Parameter("zmq_sub_addresses", std::vector<std::string> { "tcp://host:6002" }),
        rclcpp::Parameter("shm_name", "/test_shm"),
        rclcpp::Parameter("video_pub_address", "tcp://127.0.0.1:6003"),
        rclcpp::Parameter("image_topic", "/test/image"),
        rclcpp::Parameter("video_width", 1920),
        rclcpp::Parameter("video_height", 1080),
        rclcpp::Parameter("enable_video_stream", false),
    });
    auto node = std::make_shared<rclcpp::Node>("radar_bridge_config_overrides", options);
    BridgeConfig config;

    auto result = ConfigsLoader(*node, config);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(config.zmq_pub_address, "tcp://127.0.0.1:6001");
    EXPECT_EQ(config.zmq_sub_addresses, std::vector<std::string> { "tcp://host:6002" });
    EXPECT_EQ(config.shm_name, "/test_shm");
    EXPECT_EQ(config.video_pub_address, "tcp://127.0.0.1:6003");
    EXPECT_EQ(config.image_topic, "/test/image");
    EXPECT_EQ(config.video_width, 1920);
    EXPECT_EQ(config.video_height, 1080);
    EXPECT_FALSE(config.enable_video_stream);
}

TEST_F(RadarBridgeConfigTest, RejectsInvalidZmqConfigurationWhenVideoIsDisabled) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("zmq_pub_address", ""),
        rclcpp::Parameter("enable_video_stream", false),
    });
    auto node = std::make_shared<rclcpp::Node>("radar_bridge_invalid_zmq", options);
    BridgeConfig config;

    auto result = ConfigsLoader(*node, config);

    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("zmq_pub_address"), std::string::npos);
}

TEST_F(RadarBridgeConfigTest, RejectsInvalidVideoConfigurationWhenVideoIsEnabled) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("video_width", 0),
        rclcpp::Parameter("enable_video_stream", true),
    });
    auto node = std::make_shared<rclcpp::Node>("radar_bridge_invalid_video", options);
    BridgeConfig config;

    auto result = ConfigsLoader(*node, config);

    ASSERT_FALSE(result.has_value());
    EXPECT_NE(result.error().find("video_width"), std::string::npos);
}

TEST_F(RadarBridgeConfigTest, IgnoresInvalidVideoConfigurationWhenVideoIsDisabled) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("shm_name", ""),
        rclcpp::Parameter("video_pub_address", ""),
        rclcpp::Parameter("image_topic", ""),
        rclcpp::Parameter("video_width", 0),
        rclcpp::Parameter("video_height", 0),
        rclcpp::Parameter("enable_video_stream", false),
    });
    auto node = std::make_shared<rclcpp::Node>("radar_bridge_video_disabled", options);
    BridgeConfig config;

    auto result = ConfigsLoader(*node, config);

    ASSERT_TRUE(result.has_value());
    EXPECT_FALSE(config.enable_video_stream);
}

} // namespace
} // namespace radar_bridge::node
