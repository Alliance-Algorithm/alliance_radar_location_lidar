#include "radar_camera/radar_recorder_node.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace radar_camera::recording_node {

auto lifecycle_order() -> std::vector<LifecycleComponent> {
    return { LifecycleComponent::recorder, LifecycleComponent::reader,
        LifecycleComponent::monitor };
}

auto cleanup_order(const std::vector<LifecycleComponent>& started)
    -> std::vector<LifecycleComponent> {
    std::vector<LifecycleComponent> cleanup;
    if (std::find(started.begin(), started.end(), LifecycleComponent::monitor) != started.end()) {
        cleanup.emplace_back(LifecycleComponent::monitor);
    }
    for (auto it = started.rbegin(); it != started.rend(); ++it) {
        if (*it == LifecycleComponent::reader || *it == LifecycleComponent::recorder) {
            cleanup.push_back(*it);
        }
    }
    return cleanup;
}

auto make_components(
    const recording::RecordingConfig& config, const std::string& shm_name) -> RecorderComponents {
    if (!config.enabled) return { };
    auto fifo     = std::make_unique<recording::RecordingFifo>(config.buffer_pool_frames);
    auto recorder = std::make_unique<recording::RawVideoRecorder>(config, *fifo);
    auto reader   = std::make_unique<recording::RawShmReader>(
        shm_name, config.width, config.height, *fifo, config.fps);
    return { std::move(fifo), std::move(recorder), std::move(reader) };
}

auto RecordingConfigsLoader(rclcpp::Node& node, recording::RecordingConfig& recording,
    std::string& shm_name, int& width, int& height) -> std::expected<void, std::string> {
    try {
        node.declare_parameter("shm_name", std::string("/hikcamera_shm"));
        node.declare_parameter("width", 5472);
        node.declare_parameter("height", 3648);
        node.declare_parameter("enable_raw_recording", false);
        node.declare_parameter("recording_output_dir", std::string("/workspace/model/video"));
        node.declare_parameter("recording_width", 5472);
        node.declare_parameter("recording_height", 3648);
        node.declare_parameter("recording_fps", 8);
        node.declare_parameter("recording_bitrate", 25000000);
        node.declare_parameter("recording_gop", 20);
        node.declare_parameter("recording_encoder", std::string("hevc_nvenc"));
        node.declare_parameter("recording_segment_duration_sec", 0);  // 0 = 整段
        node.declare_parameter("recording_buffer_pool_frames", 8);
        node.declare_parameter("recording_max_buffer_bytes", 480000000);

        node.get_parameter("shm_name", shm_name);
        node.get_parameter("width", width);
        node.get_parameter("height", height);
        node.get_parameter("enable_raw_recording", recording.enabled);
        node.get_parameter("recording_output_dir", recording.output_dir);
        node.get_parameter("recording_width", recording.width);
        node.get_parameter("recording_height", recording.height);
        node.get_parameter("recording_fps", recording.fps);
        node.get_parameter("recording_bitrate", recording.bitrate);
        node.get_parameter("recording_gop", recording.gop);
        node.get_parameter("recording_encoder", recording.encoder);
        node.get_parameter("recording_segment_duration_sec", recording.segment_duration_sec);
        std::int64_t buffer_pool_frames = 0;
        std::int64_t max_buffer_bytes   = 0;
        node.get_parameter("recording_buffer_pool_frames", buffer_pool_frames);
        node.get_parameter("recording_max_buffer_bytes", max_buffer_bytes);
        if (buffer_pool_frames < 0 || max_buffer_bytes < 0) {
            return std::unexpected("recording buffer parameters must not be negative");
        }
        recording.buffer_pool_frames = static_cast<std::size_t>(buffer_pool_frames);
        recording.max_buffer_bytes   = static_cast<std::size_t>(max_buffer_bytes);
        if (recording.enabled) {
            const auto recording_ret = recording::validate_config(recording);
            if (!recording_ret) {
                return std::unexpected(
                    "recording configuration invalid: " + recording_ret.error());
            }
        }
    } catch (const std::exception& e) {
        return std::unexpected(std::string("Error loading configuration: ") + e.what());
    }
    return { };
}

} // namespace radar_camera::recording_node
