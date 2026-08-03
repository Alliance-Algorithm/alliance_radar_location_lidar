#include "radar_bridge/zmq_bridge.hpp"

namespace radar_bridge::zmq_bridge {

auto ZmqBridge::zmqpub_init(const std::string& pub_address) -> std::expected<void, std::string> {
    try {
        publisher_ = zmq::socket_t(context_, zmq::socket_type::pub);
        // linger=0：socket 关闭时立即释放端口，避免异常退出后孤儿 socket 占用
        // （libzmq 4.3.5 无 ZMQ_REUSEADDR，linger 0 是等效的标准做法）
        publisher_.set(zmq::sockopt::linger, 0);
        publisher_.bind(pub_address.data());
    } catch (const zmq::error_t& e) {
        return std::unexpected(std::string("zmqpub_init failed: ") + e.what());
    }
    return { };
}

auto ZmqBridge::zmqsub_init(const std::vector<std::string>& sub_addresses)
    -> std::expected<void, std::string> {
    try {
        subscriber_ = zmq::socket_t(context_, zmq::socket_type::sub);
        for (const auto& address : sub_addresses) {
            subscriber_.connect(address.data());
        }
        subscriber_.set(zmq::sockopt::subscribe, "");
    } catch (const zmq::error_t& e) {
        return std::unexpected(std::string("zmqsub_init failed: ") + e.what());
    }
    return { };
}

auto ZmqBridge::zmqpub(const radar_bridge::zmqdata::pub::LidarLocation& lidarlocation_data)
    -> std::expected<void, std::string> {
    auto message = zmq_json_encode(lidarlocation_data);
    zmq::message_t zmq_message(message.data(), message.size());
    auto result = publisher_.send(zmq_message, zmq::send_flags::dontwait);
    if (!result.has_value()) {
        return std::unexpected("Failed to send message");
    }
    return { };
}

auto ZmqBridge::zmqsub(radar_bridge::zmqdata::sub::TransmitGameState& game_state_)
    -> std::expected<void, std::string> {
    zmq::message_t zmq_message;
    auto recv_result = subscriber_.recv(zmq_message, zmq::recv_flags::dontwait);
    if (!recv_result.has_value()) {
        return std::unexpected("No data available");
    }
    try {
        auto json   = nlohmann::json::parse(zmq_message.to_string());
        game_state_ = zmq_json_decode<radar_bridge::zmqdata::sub::TransmitGameState>(json);
    } catch (const std::exception& e) {
        return std::unexpected(std::string("zmqsub parse failed: ") + e.what());
    }
    return { };
}

} // namespace radar_bridge::zmq_bridge
