#pragma once

#include <atomic>
#include <chrono>
#include <expected>
#include <string>
#include <thread>
#include <zmq.hpp>

#include <opencv2/core.hpp>

namespace radar_bridge::videostream_bridge {

class VideoBridge final {
public:
    VideoBridge() = default;
    ~VideoBridge();

    auto video_init(const std::string& shm_name, const std::string& pub_address,
                    int width, int height) -> std::expected<void, std::string>;
    auto video_thread()      -> std::expected<void, std::string>;
    auto video_thread_stop() -> std::expected<void, std::string>;

private:
    int shm_fd_ = -1;
    void* shm_ptr_ = nullptr;
    std::string shm_name_;
    std::string pub_address_;
    int width_ = 0;
    int height_ = 0;

    zmq::context_t ctx_{1};
    zmq::socket_t pub_{ctx_, zmq::socket_type::pub};
    std::thread video_thread_;
    std::atomic<bool> video_thread_running_{false};
};

} // namespace radar_bridge::videostream_bridge