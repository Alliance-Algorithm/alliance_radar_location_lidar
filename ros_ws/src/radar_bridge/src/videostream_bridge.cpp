#include "radar_bridge/videostream_bridge.hpp"
#include "hikcamera_ros_driver/shm.hpp"

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
    shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0666);
    if (shm_fd_ == -1)
        return std::unexpected("shm_open failed: " + shm_name_);

    shm_ptr_ = mmap(nullptr, sizeof(hikcamera_ros_driver::imageSHM),
                    PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (shm_ptr_ == MAP_FAILED) {
        close(shm_fd_);
        shm_fd_ = -1;
        return std::unexpected("mmap failed");
    }

    try {
        pub_.bind(pub_address_);
    } catch (const zmq::error_t& e) {
        munmap(shm_ptr_, sizeof(hikcamera_ros_driver::imageSHM));
        shm_ptr_ = nullptr;
        close(shm_fd_);
        shm_fd_ = -1;
        return std::unexpected("zmq bind failed: " + std::string(e.what()));
    }
    pub_.set(zmq::sockopt::conflate, 1);
    return { };
}

auto VideoBridge::video_thread() -> std::expected<void, std::string> {
    if (!shm_ptr_) return std::unexpected("not initialized");
    video_thread_running_ = true;
    video_thread_ = std::thread([this]() {
        auto* shm = static_cast<hikcamera_ros_driver::imageSHM*>(shm_ptr_);

        while (video_thread_running_) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 1;

            if (sem_timedwait(&shm->sem, &ts) != 0) continue;

            pthread_mutex_lock(&shm->mutex);
            if (shm->read_index >= shm->write_index)
                shm->read_index = shm->write_index;
            else
                shm->read_index++;

            auto& frame = shm->imagedata[shm->read_index];
            cv::Mat mat(height_, width_, CV_8UC3, frame);
            auto ts_frame = shm->timestamp[shm->read_index];
            pthread_mutex_unlock(&shm->mutex);

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
    if (shm_ptr_) {
        munmap(shm_ptr_, sizeof(hikcamera_ros_driver::imageSHM));
        shm_ptr_ = nullptr;
    }
    if (shm_fd_ != -1) {
        close(shm_fd_);
        shm_fd_ = -1;
    }
    return { };
}

} // namespace radar_bridge::videostream_bridge
