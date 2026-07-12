#pragma once
#include "radar_bridge/zmq_data_format.hpp"
#include <atomic>
#include <expected>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <zmq.hpp>
namespace radar_bridge::zmq_bridge {

class ZmqBridge final {
public:
    ZmqBridge() = default;
    ZmqBridge& operator=(const ZmqBridge&) = delete;
    ZmqBridge(const ZmqBridge&)            = delete;
    ~ZmqBridge();
    auto zmqpub_init(const std::string& pub_address) -> std::expected<void, std::string>;
    auto zmqsub_init(const std::vector<std::string>& sub_addresses) -> std::expected<void, std::string>;
    auto zmqpub(const radar_bridge::zmqdata::pub::LidarLocation& lidarlocation_data)
        -> std::expected<void, std::string>;
    auto zmqsub(radar_bridge::zmqdata::sub::TransmitGameState& game_state_)
        -> std::expected<void, std::string>;
    auto zmqsub_thread(std::atomic<bool>& zmqsub_thread_running_,
        radar_bridge::zmqdata::sub::TransmitGameState& game_state_)
        -> std::expected<void, std::string>;
    auto zmqpub_thread(std::atomic<bool>& zmqpub_thread_running_,
        const radar_bridge::zmqdata::pub::LidarLocation& lidarlocation_data)
        -> std::expected<void, std::string>;
    auto zmqpub_thread_stop(std::atomic<bool>& zmqpub_thread_running_)
        -> std::expected<void, std::string>;
    auto zmqsub_thread_stop(std::atomic<bool>& zmqsub_thread_running_)
        -> std::expected<void, std::string>;

private:
    std::mutex zmq_mutex_;
    std::thread zmqpub_thread_;
    std::thread zmqsub_thread_;

    zmq::context_t context_;
    zmq::socket_t publisher_;
    zmq::socket_t subscriber_;
};
}