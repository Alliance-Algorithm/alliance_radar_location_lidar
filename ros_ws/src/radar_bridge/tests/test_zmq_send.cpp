#include "radar_bridge/zmq_bridge.hpp"
#include "radar_bridge/zmq_data_format.hpp"

#include <chrono>
#include <iostream>
#include <thread>

int main() {
    using namespace radar_bridge::zmqdata::pub;

    LidarLocation data;
    data.opponent_hero_x       = 1500;
    data.opponent_hero_y       = 800;
    data.opponent_engineer_x   = 1200;
    data.opponent_engineer_y   = 600;
    data.opponent_infantry_3_x = 800;
    data.opponent_infantry_3_y = 400;
    data.opponent_infantry_4_x = 600;
    data.opponent_infantry_4_y = 200;
    data.opponent_aerial_x     = 2000;
    data.opponent_aerial_y     = 0;
    data.opponent_sentry_x     = 2500;
    data.opponent_sentry_y     = 500;
    data.ally_hero_x           = 100;
    data.ally_hero_y           = 100;
    data.ally_engineer_x       = 200;
    data.ally_engineer_y       = 150;
    data.ally_infantry_3_x     = 300;
    data.ally_infantry_3_y     = 250;
    data.ally_infantry_4_x     = 400;
    data.ally_infantry_4_y     = 350;
    data.ally_aerial_x         = 500;
    data.ally_aerial_y         = 450;
    data.ally_sentry_x         = 600;
    data.ally_sentry_y         = 550;

    std::string json = zmq_json_encode(data);
    std::cout << "== JSON output ==" << std::endl;
    std::cout << json << std::endl;

    radar_bridge::zmq_bridge::ZmqBridge bridge;
    auto init_ret = bridge.zmqpub_init("tcp://*:5555");
    if (!init_ret) {
        std::cerr << "zmqpub_init failed: " << init_ret.error() << std::endl;
        return 1;
    }
    std::cout << "\n== ZMQ PUB bound to tcp://*:5555, sending..." << std::endl;

    auto send_ret = bridge.zmqpub(data);
    if (!send_ret) {
        std::cerr << "zmqpub failed: " << send_ret.error() << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "send done." << std::endl;
    return 0;
}
