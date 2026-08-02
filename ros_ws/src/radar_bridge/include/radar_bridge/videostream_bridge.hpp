#pragma once

#include <hikcamera/shared_frame_reader.hpp>

#include <atomic>
#include <expected>
#include <memory>
#include <string>
#include <thread>
#include <zmq.hpp>

#include "radar_camera/armor_infer.hpp"

namespace radar_bridge::videostream_bridge {

class VideoBridge final {
public:
    VideoBridge() = default;
    ~VideoBridge();

    auto video_init(const std::string& shm_name, const std::string& pub_address,
        std::shared_ptr<radar_camera::armor_infer::ArmorInfer> infer)
        -> std::expected<void, std::string>;
    auto video_thread() -> std::expected<void, std::string>;
    auto video_thread_stop() -> std::expected<void, std::string>;

private:
    void draw_overlay(
        cv::Mat& bgr, const std::vector<radar_camera::armor_infer::ArmorResult>& results);

    hikcamera::SharedFrameReader reader_;
    std::shared_ptr<radar_camera::armor_infer::ArmorInfer> infer_;
    std::string pub_address_;
    std::string shm_name_;
    zmq::context_t ctx_ { 1 };
    zmq::socket_t pub_ { ctx_, zmq::socket_type::pub };
    std::thread video_thread_;
    std::atomic<bool> video_thread_running_ { false };
};

} // namespace radar_bridge::videostream_bridge
