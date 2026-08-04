#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "radar_camera/raw_shm_reader.hpp"

namespace radar_camera::recording {

class RecordingFifo {
public:
    explicit RecordingFifo(std::size_t capacity)
        : capacity_(capacity) { }

    [[nodiscard]] auto capacity() const -> std::size_t { return capacity_; }

    auto try_push(RawFrame&& frame) -> bool {
        std::lock_guard lock(mutex_);
        if (closed_ || overrun_ || queue_.size() >= capacity_) {
            // 满时直接返回 false：由 RawShmReader 丢帧（计入 dropped），
            // 不置 overrun（否则一次瞬时满就永久 fail 停掉整条推理链路）。
            return false;
        }
        queue_.push_back(std::move(frame));
        return true;
    }

    auto pop() -> std::optional<RawFrame> {
        std::lock_guard lock(mutex_);
        if (queue_.empty()) {
            return std::nullopt;
        }
        RawFrame frame = std::move(queue_.front());
        queue_.pop_front();
        return frame;
    }

    void request_overrun(std::string reason) {
        std::lock_guard lock(mutex_);
        request_overrun_locked(std::move(reason));
    }

    [[nodiscard]] auto overrun() const -> bool {
        std::lock_guard lock(mutex_);
        return overrun_;
    }

    [[nodiscard]] auto size() const -> std::size_t {
        std::lock_guard lock(mutex_);
        return queue_.size();
    }

    void close() {
        std::lock_guard lock(mutex_);
        closed_ = true;
    }

    void reset() {
        std::lock_guard lock(mutex_);
        queue_.clear();
        overrun_ = false;
        closed_  = false;
        overrun_reason_.clear();
    }

private:
    void request_overrun_locked(std::string reason) {
        overrun_        = true;
        overrun_reason_ = std::move(reason);
    }

    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<RawFrame> queue_;
    bool overrun_ = false;
    bool closed_  = false;
    std::string overrun_reason_;
};

} // namespace radar_camera::recording
