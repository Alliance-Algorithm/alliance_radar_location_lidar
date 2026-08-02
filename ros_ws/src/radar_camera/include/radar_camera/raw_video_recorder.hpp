#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include "radar_camera/recording_fifo.hpp"

namespace radar_camera::recording {

struct RecordingConfig {
    bool enabled;
    std::string output_dir;
    int width;
    int height;
    int fps;
    int bitrate;
    int gop;
    std::string encoder;
    int segment_duration_sec;
    std::size_t buffer_pool_frames;
    std::size_t max_buffer_bytes;
};

inline auto validate_config(const RecordingConfig& config) -> std::expected<void, std::string> {
    if (config.output_dir.empty()) {
        return std::unexpected("output_dir must not be empty");
    }
    if (config.width <= 0 || config.width % 2 != 0) {
        return std::unexpected("width must be a positive even number");
    }
    if (config.height <= 0 || config.height % 2 != 0) {
        return std::unexpected("height must be a positive even number");
    }
    if (config.fps <= 0) {
        return std::unexpected("fps must be positive");
    }
    if (config.bitrate <= 0) {
        return std::unexpected("bitrate must be positive");
    }
    if (config.gop <= 0) {
        return std::unexpected("gop must be positive");
    }
    if (config.encoder != "h264_nvenc" && config.encoder != "hevc_nvenc"
        && config.encoder != "libx264") {
        return std::unexpected("encoder must be h264_nvenc, hevc_nvenc or libx264");
    }
    if (config.segment_duration_sec <= 0) {
        return std::unexpected("segment_duration_sec must be positive");
    }
    const auto fps      = static_cast<std::uint64_t>(config.fps);
    const auto duration = static_cast<std::uint64_t>(config.segment_duration_sec);
    if (fps > std::numeric_limits<std::uint64_t>::max() / duration) {
        return std::unexpected("segment frame count calculation overflows");
    }
    if (config.buffer_pool_frames == 0) {
        return std::unexpected("buffer_pool_frames must be positive");
    }

    constexpr auto max_size = std::numeric_limits<std::size_t>::max();
    const auto width        = static_cast<std::size_t>(config.width);
    const auto height       = static_cast<std::size_t>(config.height);
    if (width > max_size / height) {
        return std::unexpected("buffer size calculation overflows");
    }
    auto bytes = width * height;
    if (bytes > max_size / 3) {
        return std::unexpected("buffer size calculation overflows");
    }
    bytes *= 3;
    if (bytes > max_size / config.buffer_pool_frames) {
        return std::unexpected("buffer size calculation overflows");
    }
    if (bytes * config.buffer_pool_frames > config.max_buffer_bytes) {
        return std::unexpected("max_buffer_bytes is too small for the buffer pool");
    }
    return { };
}

enum class RecorderState { stopped, running, failed, overrun };

struct RecorderStats {
    std::uint64_t queued   = 0;
    std::uint64_t encoded  = 0;
    std::uint64_t segments = 0;
    std::uint64_t overruns = 0;
    std::uint64_t errors   = 0;
    std::uint64_t dropped  = 0;  // 主动丢帧（编码跟不上输入帧率）
};

[[nodiscard]] auto segment_path(const std::filesystem::path& output_dir,
    std::chrono::system_clock::time_point session_start, std::size_t segment_index)
    -> std::filesystem::path;

class RawVideoRecorder final {
public:
    RawVideoRecorder(RecordingConfig config, RecordingFifo& fifo);
    ~RawVideoRecorder();

    RawVideoRecorder(const RawVideoRecorder&)                    = delete;
    auto operator=(const RawVideoRecorder&) -> RawVideoRecorder& = delete;

    auto start() -> std::expected<void, std::string>;
    void stop();
    [[nodiscard]] auto state() const -> RecorderState;
    [[nodiscard]] auto stats() const -> RecorderStats;
    [[nodiscard]] auto failure_reason() const -> std::string;

private:
    void loop();
    void fail(std::string reason, bool overrun);

    const RecordingConfig config_;
    RecordingFifo& fifo_;
    std::atomic<bool> running_ { false };
    std::atomic<bool> stop_requested_ { false };
    mutable std::mutex lifecycle_mutex_;
    mutable std::mutex mutex_;
    RecorderState state_ = RecorderState::stopped;
    RecorderStats stats_;
    std::string failure_reason_;
    std::thread thread_;
    std::chrono::steady_clock::time_point last_encoded_ { };
};

} // namespace radar_camera::recording
