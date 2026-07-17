#pragma once
#include "radar_bridge/zmq_data_format.hpp"
#include <expected>
#include <string>
#include <vector>
#include <zmq.hpp>
namespace radar_bridge::zmq_bridge {

class ZmqBridge final {
public:
    ZmqBridge()                            = default;
    ZmqBridge& operator=(const ZmqBridge&) = delete;
    ZmqBridge(const ZmqBridge&)            = delete;
    ~ZmqBridge() = default;
    auto zmqpub_init(const std::string& pub_address) -> std::expected<void, std::string>;
    auto zmqsub_init(const std::vector<std::string>& sub_addresses)
        -> std::expected<void, std::string>;
    auto zmqpub(const radar_bridge::zmqdata::pub::LidarLocation& lidarlocation_data)
        -> std::expected<void, std::string>;
    auto zmqsub(radar_bridge::zmqdata::sub::TransmitGameState& game_state_)
        -> std::expected<void, std::string>;

private:
    zmq::context_t context_;
    zmq::socket_t publisher_;
    zmq::socket_t subscriber_;
};
}