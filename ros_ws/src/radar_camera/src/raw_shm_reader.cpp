#include "radar_camera/raw_shm_reader.hpp"

#include <chrono>
#include <cstring>
#include <limits>

#include "radar_camera/recording_fifo.hpp"

namespace radar_camera::recording {

auto valid_frame_counter(std::uint64_t counter) -> bool { return counter != 0; }

auto completed_slot(std::uint64_t counter, unsigned int slot_num) -> unsigned int {
    if (counter == 0 || slot_num == 0) {
        return 0;
    }
    return static_cast<unsigned int>((counter - 1) % static_cast<std::uint64_t>(slot_num));
}

auto is_stable(std::uint64_t before, std::uint64_t after) -> bool { return before == after; }

auto validate_raw_frame_dimensions(int width, int height) -> bool {
    if (width <= 0 || height <= 0) {
        return false;
    }
    const auto pixels = static_cast<std::uint64_t>(width)
                      * static_cast<std::uint64_t>(height);
    return pixels <= static_cast<std::uint64_t>(MAX_IMAGE_SIZE) / 3;
}

auto raw_frame_byte_count(int width, int height) -> std::optional<std::size_t> {
    if (!validate_raw_frame_dimensions(width, height)) {
        return std::nullopt;
    }
    constexpr auto channels = std::size_t{3};
    const auto max_size = std::numeric_limits<std::size_t>::max();
    const auto width_size = static_cast<std::size_t>(width);
    const auto height_size = static_cast<std::size_t>(height);
    if (width_size > max_size / height_size) {
        return std::nullopt;
    }
    const auto pixels = width_size * height_size;
    if (pixels > max_size / channels) {
        return std::nullopt;
    }
    return pixels * channels;
}

RawShmReader::RawShmReader(std::string shm_name, int width, int height, RecordingFifo& fifo)
    : shm_name_(std::move(shm_name)),
      width_(width),
      height_(height),
      image_bytes_(raw_frame_byte_count(width, height).value_or(0)),
      fifo_(fifo) {}

RawShmReader::~RawShmReader() { stop(); }

auto RawShmReader::start() -> std::expected<void, std::string> {
    if (!validate_raw_frame_dimensions(width_, height_) || image_bytes_ == 0) {
        return std::unexpected("raw frame dimensions are invalid");
    }
    if (shm_name_.empty()) {
        return std::unexpected("SHM name must not be empty");
    }
    if (running_.exchange(true, std::memory_order_acq_rel)) {
        return std::unexpected("reader is already running");
    }

    auto fd_ret = hikcamera::SHMInit(shm_name_, sizeof(hikcamera::imageSHM));
    if (!fd_ret) {
        running_.store(false, std::memory_order_release);
        return std::unexpected("SHMInit failed: " + fd_ret.error());
    }
    shm_fd_ = *fd_ret;

    auto ptr_ret = hikcamera::SHMGetPtr(shm_fd_);
    if (!ptr_ret) {
        std::ignore = hikcamera::SHMClose(shm_fd_);
        shm_fd_ = -1;
        running_.store(false, std::memory_order_release);
        return std::unexpected("SHMGetPtr failed: " + ptr_ret.error());
    }
    shm_ptr_ = *ptr_ret;
    {
        std::lock_guard lock(mutex_);
        state_ = ReaderState::running;
        stats_ = {};
    }
    thread_ = std::thread(&RawShmReader::loop, this);
    return {};
}

void RawShmReader::stop() {
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
    close_shm();
    std::lock_guard lock(mutex_);
    if (state_ == ReaderState::running) {
        state_ = ReaderState::stopped;
    }
}

auto RawShmReader::state() const -> ReaderState {
    std::lock_guard lock(mutex_);
    return state_;
}

auto RawShmReader::stats() const -> ReaderStats {
    std::lock_guard lock(mutex_);
    return stats_;
}

void RawShmReader::close_shm() {
    if (shm_ptr_ != nullptr) {
        std::ignore = hikcamera::SHMReleasePtr(shm_ptr_);
        shm_ptr_ = nullptr;
    }
    if (shm_fd_ != -1) {
        std::ignore = hikcamera::SHMClose(shm_fd_);
        shm_fd_ = -1;
    }
}

void RawShmReader::loop() {
    constexpr auto slot_count = 4U;
    constexpr auto poll_interval = std::chrono::milliseconds(1);
    constexpr auto max_copy_retries = 3U;
    std::uint64_t last_seen = 0;

    while (running_.load(std::memory_order_acquire)) {
        const auto counter = shm_ptr_->frame_counter.load(std::memory_order_acquire);
        if (!valid_frame_counter(counter) || counter == last_seen) {
            std::this_thread::sleep_for(poll_interval);
            continue;
        }

        auto latest_seen = counter;
        bool accepted = false;
        for (unsigned int retry = 0; retry < max_copy_retries; ++retry) {
            const auto counter_before = shm_ptr_->frame_counter.load(std::memory_order_acquire);
            if (!valid_frame_counter(counter_before)) {
                break;
            }
            latest_seen = counter_before;
            const auto slot = completed_slot(counter_before, slot_count);
            cv::Mat rgb(height_, width_, CV_8UC3);
            std::memcpy(rgb.data, shm_ptr_->imagedata[slot], image_bytes_);
            const auto timestamp = shm_ptr_->timestamp[slot];
            const auto counter_after = shm_ptr_->frame_counter.load(std::memory_order_acquire);
            latest_seen = counter_after;

            if (!is_stable(counter_before, counter_after)) {
                continue;
            }

            RawFrame frame{
                std::move(rgb), counter_before,
                static_cast<std::uint64_t>(timestamp.time_since_epoch().count())};
            {
                std::lock_guard lock(mutex_);
                ++stats_.observed;
            }
            if (!fifo_.try_push(std::move(frame))) {
                fifo_.request_overrun("raw SHM reader could not submit frame");
                running_.store(false, std::memory_order_release);
                std::lock_guard lock(mutex_);
                state_ = ReaderState::overrun;
                break;
            }
            {
                std::lock_guard lock(mutex_);
                ++stats_.accepted;
            }
            accepted = true;
            break;
        }

        last_seen = latest_seen;
        if (!accepted && running_.load(std::memory_order_acquire)) {
            fifo_.request_overrun("raw SHM frame counter was unstable");
            running_.store(false, std::memory_order_release);
            std::lock_guard lock(mutex_);
            ++stats_.unstable;
            state_ = ReaderState::overrun;
        }
    }
}

} // namespace radar_camera::recording
