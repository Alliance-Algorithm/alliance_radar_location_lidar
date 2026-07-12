#include "radar_bridge/videostream_bridge.hpp"
#include <hikcamera/shm.hpp>

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>

namespace radar_bridge::videostream_bridge {

VideoBridge::~VideoBridge() { auto _ = video_thread_stop(); }

auto VideoBridge::video_init(const std::string& shm_name, const std::string& pub_address,
                             int width, int height) -> std::expected<void, std::string> {
    shm_name_    = shm_name;
    pub_address_ = pub_address;
    width_       = width;
    height_      = height;
    shm_fd_ = shm_open(shm_name_.c_str(), O_RDONLY, 0666);
    if (shm_fd_ == -1)
        return std::unexpected("shm_open failed: " + shm_name_);

    try {
        pub_.bind(pub_address_);
    } catch (const zmq::error_t& e) {
        close(shm_fd_);
        shm_fd_ = -1;
        return std::unexpected("zmq bind failed: " + std::string(e.what()));
    }
    pub_.set(zmq::sockopt::conflate, 1);
    return { };
}

auto VideoBridge::video_thread() -> std::expected<void, std::string> {
    if (shm_fd_ == -1) return std::unexpected("not initialized");
    video_thread_running_ = true;
    video_thread_ = std::thread([this]() {
        while (video_thread_running_) {
            cv::Mat mat;
            std::chrono::steady_clock::time_point ts;
            auto ret = hikcamera::SHMRead(shm_fd_, mat, ts, width_, height_);
            if (!ret.has_value()) continue;

            std::vector<uchar> jpeg;
            cv::imencode(".jpg", mat, jpeg, {cv::IMWRITE_JPEG_QUALITY, 85});

            pub_.send(zmq::message_t(jpeg.data(), jpeg.size()), zmq::send_flags::none);
        }
    });
    return { };
}

auto VideoBridge::video_thread_stop() -> std::expected<void, std::string> {
    video_thread_running_ = false;
    if (video_thread_.joinable()) video_thread_.join();
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_fd_ = -1;
    }
    return { };
}

} // namespace radar_bridge::videostream_bridge
