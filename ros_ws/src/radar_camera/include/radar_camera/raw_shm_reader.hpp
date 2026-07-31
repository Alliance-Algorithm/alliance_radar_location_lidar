#pragma once

#include <cstddef>
#include <cstdint>
#include <atomic>
#include <expected>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/core/mat.hpp>

#include "hikcamera/shm.hpp"

namespace radar_camera::recording {

class RecordingFifo;

struct RawFrame {
    cv::Mat rgb;
    std::uint64_t sequence;
    std::uint64_t host_monotonic_ns;
};

enum class ReaderState {
    stopped,
    running,
    overrun,
};

struct ReaderStats {
    std::uint64_t observed = 0;
    std::uint64_t accepted = 0;
    std::uint64_t unstable = 0;
};

[[nodiscard]] auto valid_frame_counter(std::uint64_t counter) -> bool;
[[nodiscard]] auto completed_slot(std::uint64_t counter, unsigned int slot_num) -> unsigned int;
[[nodiscard]] auto is_stable(std::uint64_t before, std::uint64_t after) -> bool;
[[nodiscard]] auto validate_raw_frame_dimensions(int width, int height) -> bool;
[[nodiscard]] auto raw_frame_byte_count(int width, int height) -> std::optional<std::size_t>;

class RawShmReader final {
public:
    RawShmReader(std::string shm_name, int width, int height, RecordingFifo& fifo);
    ~RawShmReader();

    RawShmReader(const RawShmReader&) = delete;
    auto operator=(const RawShmReader&) -> RawShmReader& = delete;

    auto start() -> std::expected<void, std::string>;
    void stop();
    [[nodiscard]] auto state() const -> ReaderState;
    [[nodiscard]] auto stats() const -> ReaderStats;

private:
    void loop();
    void close_shm();

    const std::string shm_name_;
    const int width_;
    const int height_;
    const std::size_t image_bytes_;
    RecordingFifo& fifo_;
    std::atomic<bool> running_{false};
    mutable std::mutex mutex_;
    ReaderState state_ = ReaderState::stopped;
    ReaderStats stats_;
    std::thread thread_;
    int shm_fd_ = -1;
    hikcamera::imageSHM* shm_ptr_ = nullptr;
};

} // namespace radar_camera::recording
