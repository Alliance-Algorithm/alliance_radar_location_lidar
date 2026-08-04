#include "radar_camera/raw_shm_reader.hpp"

#include <chrono>
#include <limits>
#include <stdexcept>
#include <sys/resource.h>
#include <utility>

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

auto is_contiguous_counter(std::uint64_t last_seen, std::uint64_t current) -> bool {
    if (last_seen == 0) {
        return current != 0;
    }
    if (last_seen == std::numeric_limits<std::uint64_t>::max()) {
        return false;
    }
    return current == last_seen + 1;
}

auto is_counter_reset(std::uint64_t last_seen, std::uint64_t current) -> bool {
    return last_seen != 0 && current == 0;
}

auto validate_raw_frame_dimensions(int width, int height) -> bool {
    if (width <= 0 || height <= 0) {
        return false;
    }
    const auto pixels = static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height);
    return pixels <= static_cast<std::uint64_t>(hikcamera::kShmMaxPixelBytes) / 3;
}

auto raw_frame_byte_count(int width, int height) -> std::optional<std::size_t> {
    if (!validate_raw_frame_dimensions(width, height)) {
        return std::nullopt;
    }
    constexpr auto channels = std::size_t { 3 };
    const auto max_size     = std::numeric_limits<std::size_t>::max();
    const auto width_size   = static_cast<std::size_t>(width);
    const auto height_size  = static_cast<std::size_t>(height);
    if (width_size > max_size / height_size) {
        return std::nullopt;
    }
    const auto pixels = width_size * height_size;
    if (pixels > max_size / channels) {
        return std::nullopt;
    }
    return pixels * channels;
}

RawShmReader::RawShmReader(
    std::string shm_name, int width, int height, RecordingFifo& fifo, double target_fps)
    : shm_name_(std::move(shm_name))
    , width_(width)
    , height_(height)
    , fifo_(fifo)
    , target_fps_(target_fps) { }

RawShmReader::~RawShmReader() { stop(); }

auto RawShmReader::start() -> std::expected<void, std::string> {
    std::lock_guard lifecycle_lock(lifecycle_mutex_);
    if (!validate_raw_frame_dimensions(width_, height_)) {
        return std::unexpected("raw frame dimensions are invalid");
    }
    if (shm_name_.empty()) {
        return std::unexpected("SHM name must not be empty");
    }
    join_finished_thread();
    if (running_.exchange(true, std::memory_order_acq_rel)) {
        return std::unexpected("reader is already running");
    }

    // SHM 重试：相机驱动可能晚于本节点启动（并发 launch）。
    // 与 infer_thread 的 30s 超时一致；超时仍失败才报错。
    constexpr auto kOpenTimeout = std::chrono::seconds { 30 };
    const auto open_start       = std::chrono::steady_clock::now();
    auto open_ret               = reader_.open(shm_name_.c_str());
    while (!open_ret) {
        if (std::chrono::steady_clock::now() - open_start > kOpenTimeout) {
            running_.store(false, std::memory_order_release);
            std::lock_guard lock(mutex_);
            state_          = ReaderState::failed;
            failure_reason_ = "could not open SHM: " + open_ret.error();
            return std::unexpected(failure_reason_);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        open_ret = reader_.open(shm_name_.c_str());
    }
    {
        std::lock_guard lock(mutex_);
        state_ = ReaderState::running;
        stats_ = { };
        failure_reason_.clear();
    }
    try {
        thread_ = std::thread(&RawShmReader::loop, this);
        // 录制线程低优先级：CPU 紧张时优先保障推理线程（nice +10）。
        if (thread_.joinable()) {
            setpriority(PRIO_PROCESS, static_cast<id_t>(thread_.native_handle()), 10);
        }
    } catch (const std::exception& error) {
        running_.store(false, std::memory_order_release);
        std::lock_guard lock(mutex_);
        state_ = ReaderState::stopped;
        return std::unexpected(std::string("could not start reader thread: ") + error.what());
    }
    return { };
}

void RawShmReader::stop() {
    std::lock_guard lifecycle_lock(lifecycle_mutex_);
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
    std::lock_guard lock(mutex_);
    if (state_ == ReaderState::running) {
        state_ = ReaderState::stopped;
    }
}

auto RawShmReader::state() const -> ReaderState {
    std::lock_guard lifecycle_lock(lifecycle_mutex_);
    const_cast<RawShmReader*>(this)->join_finished_thread();
    std::lock_guard lock(mutex_);
    return state_;
}

auto RawShmReader::stats() const -> ReaderStats {
    std::lock_guard lock(mutex_);
    return stats_;
}

auto RawShmReader::failure_reason() const -> std::string {
    std::lock_guard lock(mutex_);
    return failure_reason_;
}

void RawShmReader::fail(std::string reason, bool overrun) {
    fifo_.request_overrun(reason);
    running_.store(false, std::memory_order_release);
    std::lock_guard lock(mutex_);
    state_          = overrun ? ReaderState::overrun : ReaderState::failed;
    failure_reason_ = std::move(reason);
}

void RawShmReader::join_finished_thread() {
    if (thread_.joinable() && !running_.load(std::memory_order_acquire)
        && thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
}

void RawShmReader::loop() {
    constexpr auto read_timeout = std::chrono::milliseconds { 2000 };

    while (running_.load(std::memory_order_acquire)) {
        auto frame = reader_.wait_next(read_timeout);
        if (!frame) {
            if (frame.error().code != hikcamera::FrameReadErrorCode::Timeout) {
                fail("raw SHM read failed: " + frame.error().message);
                break;
            }
            continue;
        }
        if (!frame->valid()) {
            fail("raw SHM frame failed integrity check");
            break;
        }
        {
            std::lock_guard lock(mutex_);
            ++stats_.observed;
        }
        // 采样节流：按 target_fps 只推 1/间隔 帧，其余丢弃——
        // 60MB clone 是 CPU/内存带宽大头，录制只需 target_fps，
        // 不采样则 20fps 输入全部 clone，拖慢推理线程。
        if (target_fps_ > 0.0) {
            const auto now      = std::chrono::steady_clock::now();
            const auto interval = std::chrono::duration<double>(1.0 / target_fps_);
            if (now - last_push_ < interval) {
                continue;
            }
            last_push_ = now;
        }
        RawFrame raw_frame { frame->mat().clone(), frame->sequence(),
            frame->metadata().host_monotonic_ns };
        if (!fifo_.try_push(std::move(raw_frame))) {
            // 录制编码跟不上（如 5472x3648 HEVC ~13fps < 相机 20fps）时丢帧保实时，
            // 不再 OVERRUN 停整条推理链路；丢帧只降低录像帧率，不影响检测。
            {
                std::lock_guard lock(mutex_);
                ++stats_.dropped;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        {
            std::lock_guard lock(mutex_);
            ++stats_.accepted;
        }
    }
}

} // namespace radar_camera::recording
