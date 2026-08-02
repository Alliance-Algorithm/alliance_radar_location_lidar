#include "radar_bridge/videostream_bridge.hpp"
#include "radar_bridge/zmq_bridge.hpp"
#include "radar_bridge/zmq_data_format.hpp"
#include "radar_interfaces/msg/game_state.hpp"
#include "radar_interfaces/msg/lidar_location.hpp"
#include <expected>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "radar_camera/armor_infer.hpp"

namespace radar_bridge::node {

struct BridgeConfig {
    std::string zmq_pub_address;
    std::vector<std::string> zmq_sub_addresses;
    std::string shm_name;
    std::string video_pub_address;
    std::string image_topic;
    bool enable_inference = false;
    std::string model_dir = "/workspace/ros_ws/src/radar_camera/model";
    float l1_conf         = radar_camera::armor_infer::kL1Conf;
    float l2_conf         = radar_camera::armor_infer::kL2Conf;
    float l3_conf         = radar_camera::armor_infer::kL3Conf;
};

auto ConfigsLoader(rclcpp::Node& node, BridgeConfig& config) -> std::expected<void, std::string>;

class RadarBridgeNode final : public rclcpp::Node {
public:
    RadarBridgeNode();
    ~RadarBridgeNode() override;
    auto sub_lidar_pose_callback(const radar_interfaces::msg::LidarLocation& msg)
        -> std::expected<void, std::string>;
    auto pub_game_state_callback() -> std::expected<void, std::string>;

private:
    rclcpp::Publisher<radar_interfaces::msg::GameState>::SharedPtr game_state_publisher_;
    rclcpp::Subscription<radar_interfaces::msg::LidarLocation>::SharedPtr lidar_pose_subscription_;
    rclcpp::TimerBase::SharedPtr zmq_timer_;

    radar_bridge::zmqdata::pub::LidarLocation lidar_location_ { };
    radar_bridge::zmqdata::sub::TransmitGameState game_state_ { };

    // 官方通信协议 0x0305（选手端小地图雷达坐标）频率上限 5Hz；
    // /lidar/location 由 fusion 双路径发布（~17Hz），转发前限频到 5Hz。
    static constexpr double kLocationMaxHz = 5.0;
    std::chrono::steady_clock::time_point last_location_send_ { };

    BridgeConfig config_ { };
    radar_bridge::zmq_bridge::ZmqBridge zmq_bridge_ { };
    radar_bridge::videostream_bridge::VideoBridge video_bridge_ { };
};

} // namespace radar_bridge::node
