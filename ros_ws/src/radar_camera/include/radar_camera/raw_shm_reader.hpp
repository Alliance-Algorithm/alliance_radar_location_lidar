#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <opencv2/core/mat.hpp>

#include "hikcamera/shared_frame_reader.hpp"

namespace radar_camera::recording {

class RecordingFifo;

struct RawFrame {
    cv::Mat rgb;
    std::uint64_t sequence;
    std::uint64_t host_monotonic_ns;
    // Keeps the reader's preallocated image storage alive while queued.
    std::shared_ptr<cv::Mat> storage;
};

enum class ReaderState {
    stopped,
    running,
    failed,
    overrun,
};

struct ReaderStats {
    std::uint64_t observed = 0;
    std::uint64_t accepted = 0;
    std::uint64_t unstable = 0;
    std::uint64_t dropped  = 0;  // fifo 满丢帧（录制编码跟不上输入帧率）
};

[[nodiscard]] auto valid_frame_counter(std::uint64_t counter) -> bool;
[[nodiscard]] auto completed_slot(std::uint64_t counter, unsigned int slot_num) -> unsigned int;
[[nodiscard]] auto is_stable(std::uint64_t before, std::uint64_t after) -> bool;
[[nodiscard]] auto is_contiguous_counter(std::uint64_t last_seen, std::uint64_t current) -> bool;
[[nodiscard]] auto is_counter_reset(std::uint64_t last_seen, std::uint64_t current) -> bool;
[[nodiscard]] auto validate_raw_frame_dimensions(int width, int height) -> bool;
[[nodiscard]] auto raw_frame_byte_count(int width, int height) -> std::optional<std::size_t>;

class RawShmReader final {
public:
    RawShmReader(std::string shm_name, int width, int height, RecordingFifo& fifo,
        double target_fps = 0.0);
    ~RawShmReader();

    RawShmReader(const RawShmReader&)                    = delete;
    auto operator=(const RawShmReader&) -> RawShmReader& = delete;

    auto start() -> std::expected<void, std::string>;
    void stop();
    [[nodiscard]] auto state() const -> ReaderState;
    [[nodiscard]] auto stats() const -> ReaderStats;
    [[nodiscard]] auto failure_reason() const -> std::string;

private:
    void loop();
    void join_finished_thread();

    const std::string shm_name_;
    const int width_;
    const int height_;
    RecordingFifo& fifo_;
    const double target_fps_ = 0.0;  // 0 = 不过滤；>0 时按该帧率采样推帧（省 60MB clone）
    std::chrono::steady_clock::time_point last_push_ { };
    std::atomic<bool> running_ { false };
    mutable std::mutex lifecycle_mutex_;
    mutable std::mutex mutex_;
    ReaderState state_ = ReaderState::stopped;
    ReaderStats stats_;
    std::string failure_reason_;
    std::thread thread_;
    hikcamera::SharedFrameReader reader_;

    void fail(std::string reason, bool overrun = false);
};

} // namespace radar_camera::recording
