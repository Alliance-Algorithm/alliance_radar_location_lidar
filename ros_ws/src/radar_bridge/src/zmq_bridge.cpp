#include "radar_bridge/zmq_bridge.hpp"
#include "radar_bridge/zmq_data_format.hpp"
#include <cmath>
#include <iostream>
#include <nlohmann/json.hpp>
#include <zmq.hpp>

namespace radar_bridge::zmq_bridge {

ZmqBridge::ZmqBridge(const std::vector<std::string>& sub_address, const std::string& pub_address) {
    sub_address_.reserve(sub_address.size());
    for (const auto& address : sub_address) {
        sub_address_.emplace_back(address);
    }
    pub_address_ = pub_address;
}

ZmqBridge::~ZmqBridge() {
    // ⚠️ 调用方必须在析构 ZmqBridge 前先置 running flag = false，
    //    否则 join() 永久阻塞（while(running) 循环不退出）。
    if (zmqpub_thread_.joinable()) {
        zmqpub_thread_.join();
    }
    if (zmqsub_thread_.joinable()) {
        zmqsub_thread_.join();
    }
}

auto ZmqBridge::zmq_init() -> std::expected<void, std::string> {
    publisher_ = zmq::socket_t(context_, zmq::socket_type::pub);
    publisher_.bind(pub_address_.data());
    subscriber_ = zmq::socket_t(context_, zmq::socket_type::sub);
    for (const auto& address : sub_address_) {
        subscriber_.connect(address.data());
    }
    return { };
}

auto ZmqBridge::zmqpub(
    const std::shared_ptr<radar_bridge::zmqdata::pub::LidarLocation>& lidarlocation_data)
    -> std::expected<void, std::string> {
    std::string message;
    std::unique_lock<std::mutex> lock(zmq_mutex_);
    message = zmq_json_encode(*lidarlocation_data);
    lock.unlock();
    zmq::message_t zmq_message(message.data(), message.size());
    auto result = publisher_.send(zmq_message, zmq::send_flags::none);
    if (!result.has_value()) {
        return std::unexpected("Failed to send message");
    }
    return { };
}

auto ZmqBridge::zmqsub(
    const std::shared_ptr<radar_bridge::zmqdata::sub::TransmitGameState>& game_state_)
    -> std::expected<void, std::string> {
    zmq::message_t zmq_message;
    auto recv_result = subscriber_.recv(zmq_message, zmq::recv_flags::none);
    if (!recv_result.has_value()) {
        return std::unexpected("Failed to receive message");
    }

    auto json = nlohmann::json::parse(zmq_message.to_string());
    std::unique_lock<std::mutex> lock(zmq_mutex_);
    *game_state_ = zmq_json_decode<radar_bridge::zmqdata::sub::TransmitGameState>(json);
    lock.unlock();
    return { };
}

auto ZmqBridge::zmqsub_thread(std::atomic<bool>& zmqsub_thread_running_,
    const std::shared_ptr<radar_bridge::zmqdata::sub::TransmitGameState>& game_state_)
    -> std::expected<void, std::string> {
    zmqsub_thread_running_ = true;
    zmqsub_thread_         = std::thread([this, &zmqsub_thread_running_, game_state_]() {
        while (zmqsub_thread_running_) {
            auto result = zmqsub(game_state_);
            if (!result.has_value()) {
                std::cerr << "zmqsub error: " << result.error() << std::endl;
            }
        }
    });
    return { };
}

auto ZmqBridge::zmqpub_thread_stop(std::atomic<bool>& zmqpub_thread_running_)
    -> std::expected<void, std::string> {
    zmqpub_thread_running_ = false;
    if (zmqpub_thread_.joinable()) {
        zmqpub_thread_.join();
    }
    return { };
}

auto ZmqBridge::zmqsub_thread_stop(std::atomic<bool>& zmqsub_thread_running_)
    -> std::expected<void, std::string> {
    zmqsub_thread_running_ = false;
    if (zmqsub_thread_.joinable()) {
        zmqsub_thread_.join();
    }
    return { };
}

} // namespace radar_bridge::zmq_bridge
