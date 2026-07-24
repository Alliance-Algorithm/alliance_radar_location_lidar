#pragma once

#include <hikcamera/shm.hpp>

#include <atomic>
#include <expected>
#include <string>
#include <thread>
#include <zmq.hpp>

namespace radar_bridge::videostream_bridge {

class VideoBridge final {
public:
    VideoBridge() = default;
    ~VideoBridge();

    auto video_init(const std::string& shm_name, const std::string& pub_address,
        int image_width = 4096, int image_height = 3000) -> std::expected<void, std::string>;
    auto video_thread() -> std::expected<void, std::string>;
    auto video_thread_stop() -> std::expected<void, std::string>;

private:
    int shm_fd_                   = -1;
    hikcamera::imageSHM* shm_ptr_ = nullptr;
    int image_width_              = 0;
    int image_height_             = 0;
    std::string pub_address_;
    std::string shm_name_;
    zmq::context_t ctx_ { 1 };
    zmq::socket_t pub_ { ctx_, zmq::socket_type::pub };
    std::thread video_thread_;
    std::atomic<bool> video_thread_running_ { false };
};

} // namespace radar_bridge::videostream_bridge
