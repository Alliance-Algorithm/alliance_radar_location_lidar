#include "radar_bridge/zmq_bridge.hpp"
#include <chrono>
#include <iostream>

namespace radar_bridge::zmq_bridge {

ZmqBridge::~ZmqBridge() {
    zmqpub_thread_stop();
    zmqsub_thread_stop();
}

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
    std::string message;
    std::unique_lock<std::mutex> lock(zmq_mutex_);
    message = zmq_json_encode(lidarlocation_data);
    lock.unlock();
    zmq::message_t zmq_message(message.data(), message.size());
    auto result = publisher_.send(zmq_message, zmq::send_flags::none);
    if (!result.has_value()) {
        return std::unexpected("Failed to send message");
    }
    return { };
}

auto ZmqBridge::zmqsub(radar_bridge::zmqdata::sub::TransmitGameState& game_state_)
    -> std::expected<void, std::string> {
    zmq::message_t zmq_message;
    auto recv_result = subscriber_.recv(zmq_message, zmq::recv_flags::none);
    if (!recv_result.has_value()) {
        return std::unexpected("Failed to receive message");
    }

    auto json = nlohmann::json::parse(zmq_message.to_string());
    std::unique_lock<std::mutex> lock(zmq_mutex_);
    game_state_ = zmq_json_decode<radar_bridge::zmqdata::sub::TransmitGameState>(json);
    lock.unlock();
    return { };
}

auto ZmqBridge::zmqpub_thread(const radar_bridge::zmqdata::pub::LidarLocation& lidarlocation_data)
    -> std::expected<void, std::string> {
    zmqpub_thread_running_ = true;
    zmqpub_thread_         = std::thread([this, &lidarlocation_data]() {
        while (zmqpub_thread_running_) {
            auto result = zmqpub(lidarlocation_data);
            if (!result.has_value()) {
                std::cerr << "zmqpub error: " << result.error() << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    return { };
}

auto ZmqBridge::zmqsub_thread(radar_bridge::zmqdata::sub::TransmitGameState& game_state_)
    -> std::expected<void, std::string> {
    zmqsub_thread_running_ = true;
    zmqsub_thread_         = std::thread([this, &game_state_]() {
        while (zmqsub_thread_running_) {
            auto result = zmqsub(game_state_);
            if (!result.has_value()) {
                std::cerr << "zmqsub error: " << result.error() << std::endl;
            }
        }
    });
    return { };
}

auto ZmqBridge::zmqpub_thread_stop() -> std::expected<void, std::string> {
    zmqpub_thread_running_ = false;
    if (zmqpub_thread_.joinable()) {
        zmqpub_thread_.join();
    }
    return { };
}

auto ZmqBridge::zmqsub_thread_stop() -> std::expected<void, std::string> {
    zmqsub_thread_running_ = false;
    if (zmqsub_thread_.joinable()) {
        zmqsub_thread_.join();
    }
    return { };
}

} // namespace radar_bridge::zmq_bridge
