#include "radar_bridge/videostream_bridge.hpp"

#include <iostream>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace radar_bridge::videostream_bridge {

VideoBridge::~VideoBridge() { auto _ = video_thread_stop(); }

auto VideoBridge::video_init(const std::string& shm_name, const std::string& pub_address,
    int image_width, int image_height) -> std::expected<void, std::string> {
    shm_name_     = shm_name;
    pub_address_  = pub_address;
    image_width_  = image_width;
    image_height_ = image_height;

    auto fd_ret = hikcamera::SHMInit(shm_name_, sizeof(hikcamera::imageSHM));
    if (!fd_ret) return std::unexpected("SHMInit failed: " + fd_ret.error());
    shm_fd_ = *fd_ret;

    auto ptr_ret = hikcamera::SHMGetPtr(shm_fd_);
    if (!ptr_ret) {
        hikcamera::SHMClose(shm_fd_);
        return std::unexpected("SHMGetPtr failed: " + ptr_ret.error());
    }
    shm_ptr_ = *ptr_ret;

    try {
        pub_.bind(pub_address_);
    } catch (const zmq::error_t& e) {
        return std::unexpected("zmq bind failed: " + std::string(e.what()));
    }
    pub_.set(zmq::sockopt::conflate, 1);
    return { };
}

auto VideoBridge::video_thread() -> std::expected<void, std::string> {
    if (!shm_ptr_) return std::unexpected("not initialized");
    video_thread_running_ = true;
    video_thread_         = std::thread([this]() {
        constexpr auto kWaitTimeout = std::chrono::milliseconds { 500 };
        while (video_thread_running_) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += kWaitTimeout.count() * 1'000'000;
            if (ts.tv_nsec >= 1'000'000'000) {
                ts.tv_sec += 1;
                ts.tv_nsec -= 1'000'000'000;
            }

            if (sem_timedwait(&shm_ptr_->sem, &ts) != 0)
                continue;

            pthread_mutex_lock(&shm_ptr_->mutex);
            auto idx   = shm_ptr_->write_index % SLOT_NUM;
            cv::Mat rgb(image_height_, image_width_, CV_8UC3, shm_ptr_->imagedata[idx]);
            auto mat   = rgb.clone();
            pthread_mutex_unlock(&shm_ptr_->mutex);

            cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

            std::vector<uchar> jpeg;
            if (!cv::imencode(".jpg", mat, jpeg, { cv::IMWRITE_JPEG_QUALITY, 85 })
                || jpeg.empty()) {
                std::cerr << "[VideoBridge] JPEG encode failed\n";
                video_thread_running_ = false;
                break;
            }

            try {
                auto send_ret =
                    pub_.send(zmq::message_t(jpeg.data(), jpeg.size()), zmq::send_flags::none);
                if (!send_ret) {
                    std::cerr << "[VideoBridge] ZMQ send failed\n";
                    video_thread_running_ = false;
                    break;
                }
            } catch (const zmq::error_t& e) {
                std::cerr << "[VideoBridge] ZMQ send error: " << e.what() << "\n";
                video_thread_running_ = false;
                break;
            }
        }
    });
    return { };
}

auto VideoBridge::video_thread_stop() -> std::expected<void, std::string> {
    video_thread_running_ = false;
    if (video_thread_.joinable()) video_thread_.join();
    if (shm_ptr_) {
        std::ignore = hikcamera::SHMReleasePtr(shm_ptr_);
        shm_ptr_ = nullptr;
    }
    if (shm_fd_ != -1) {
        std::ignore = hikcamera::SHMClose(shm_fd_);
        shm_fd_ = -1;
    }
    return { };
}

} // namespace radar_bridge::videostream_bridge
