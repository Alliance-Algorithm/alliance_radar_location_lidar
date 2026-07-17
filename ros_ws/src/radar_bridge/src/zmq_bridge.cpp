#include "radar_bridge/zmq_bridge.hpp"

namespace radar_bridge::zmq_bridge {

auto ZmqBridge::zmqpub_init(const std::string& pub_address) -> std::expected<void, std::string> {
    publisher_ = zmq::socket_t(context_, zmq::socket_type::pub);
    publisher_.bind(pub_address.data());
    return { };
}

auto ZmqBridge::zmqsub_init(const std::vector<std::string>& sub_addresses)
    -> std::expected<void, std::string> {
    subscriber_ = zmq::socket_t(context_, zmq::socket_type::sub);
    for (const auto& address : sub_addresses) {
        subscriber_.connect(address.data());
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
    auto json = nlohmann::json::parse(zmq_message.to_string());
    game_state_ = zmq_json_decode<radar_bridge::zmqdata::sub::TransmitGameState>(json);
    return { };
}

} // namespace radar_bridge::zmq_bridge
