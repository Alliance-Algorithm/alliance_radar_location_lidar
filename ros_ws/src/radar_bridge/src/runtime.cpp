#include "radar_bridge/radar_bridge_node.hpp"
#include "radar_bridge/videostream_bridge.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radar_bridge::node::RadarBridgeNode>();
    const auto& cfg = node->get_config();

    radar_bridge::videostream_bridge::VideoBridge video(
        cfg.shm_name, cfg.video_pub_address,
        cfg.video_width, cfg.video_height);
    auto ret = video.video_init();
    if (!ret.has_value()) {
        RCLCPP_ERROR(rclcpp::get_logger("runtime"), "VideoBridge init failed: %s",
            ret.error().c_str());
        rclcpp::shutdown();
        return 1;
    }
    video.video_thread();

    rclcpp::spin(node);

    video.video_thread_stop();
    rclcpp::shutdown();
    return 0;
}
