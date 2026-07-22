#include "radar_bridge/videostream_bridge.hpp"

#include <iostream>

#include <opencv2/imgcodecs.hpp>

namespace radar_bridge::videostream_bridge {

VideoBridge::~VideoBridge() { auto _ = video_thread_stop(); }

auto VideoBridge::video_init(const std::string& shm_name,
    const std::string& pub_address) -> std::expected<void, std::string> {
    shm_name_    = shm_name;
    pub_address_ = pub_address;

    auto open_ret = reader_.open(shm_name_.c_str());
    if (!open_ret.has_value())
        return std::unexpected("SharedFrameReader::open failed: " + open_ret.error());

    try {
        pub_.bind(pub_address_);
    } catch (const zmq::error_t& e) {
        return std::unexpected("zmq bind failed: " + std::string(e.what()));
    }
    pub_.set(zmq::sockopt::conflate, 1);
    return {};
}

auto VideoBridge::video_thread() -> std::expected<void, std::string> {
    if (!reader_.is_open()) return std::unexpected("not initialized");
    video_thread_running_ = true;
    video_thread_         = std::thread([this]() {
        constexpr auto kWaitTimeout = std::chrono::milliseconds{500};
        while (video_thread_running_) {
            auto frame_result = reader_.wait_next(kWaitTimeout);
            if (!frame_result.has_value()) {
                if (frame_result.error() == "Timeout waiting for next frame") {
                    continue;  // normal: no new frame, bounded by running_ flag
                }
                std::cerr << "[VideoBridge] reader error: " << frame_result.error() << "\n";
                continue;
            }
            auto& frame = frame_result.value();

            std::vector<uchar> jpeg;
            if (!cv::imencode(".jpg", frame.mat(), jpeg,
                    {cv::IMWRITE_JPEG_QUALITY, 85})
                || jpeg.empty()) {
                std::cerr << "[VideoBridge] JPEG encode failed\n";
                video_thread_running_ = false;
                break;
            }

            try {
                auto send_ret = pub_.send(
                    zmq::message_t(jpeg.data(), jpeg.size()),
                    zmq::send_flags::none);
                if (!send_ret.has_value()) {
                    std::cerr << "[VideoBridge] ZMQ send failed\n";
                    video_thread_running_ = false;
                    break;
                }
            } catch (const zmq::error_t& e) {
                std::cerr << "[VideoBridge] ZMQ send error: " << e.what()
                          << "\n";
                video_thread_running_ = false;
                break;
            }

            // SharedFrame lease released here (scope end) → slot unlocked
        }
    });
    return {};
}

auto VideoBridge::video_thread_stop() -> std::expected<void, std::string> {
    video_thread_running_ = false;
    if (video_thread_.joinable()) video_thread_.join();
    // reader_ destructor automatically unmaps the SHM segment
    return {};
}

} // namespace radar_bridge::videostream_bridge
