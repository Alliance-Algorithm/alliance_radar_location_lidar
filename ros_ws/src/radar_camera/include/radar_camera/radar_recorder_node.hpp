#pragma once
#include <atomic>
#include <expected>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "radar_camera/raw_shm_reader.hpp"
#include "radar_camera/raw_video_recorder.hpp"

namespace radar_camera::recording_node {

enum class LifecycleComponent { recorder, reader, monitor };

struct RecorderComponents {
    std::unique_ptr<recording::RecordingFifo> fifo;
    std::unique_ptr<recording::RawVideoRecorder> recorder;
    std::unique_ptr<recording::RawShmReader> reader;
};

[[nodiscard]] auto lifecycle_order() -> std::vector<LifecycleComponent>;
[[nodiscard]] auto cleanup_order(const std::vector<LifecycleComponent>& started)
    -> std::vector<LifecycleComponent>;
[[nodiscard]] auto make_components(
    const recording::RecordingConfig& config, const std::string& shm_name) -> RecorderComponents;

auto RecordingConfigsLoader(rclcpp::Node& node, recording::RecordingConfig& recording,
    std::string& shm_name, int& width, int& height) -> std::expected<void, std::string>;

/// 独立录制进程：只做 SHM 采样 + HEVC/NVENC 编码，与推理进程完全隔离。
/// 录制失败只退出本进程，不触碰推理链路。
class RadarRecorderNode final : public rclcpp::Node {
public:
    RadarRecorderNode();
    ~RadarRecorderNode() override;

private:
    auto constructor_cleanup() noexcept -> void;
    auto monitor_start() -> void;
    auto monitor_stop() -> void;

    recording::RecordingConfig recording_config_ { };
    std::string shm_name_;
    int width_  = 0;
    int height_ = 0;
    std::unique_ptr<recording::RecordingFifo> fifo_;
    std::unique_ptr<recording::RawVideoRecorder> recorder_;
    std::unique_ptr<recording::RawShmReader> reader_;
    std::atomic<bool> monitor_running_ { false };
    std::thread monitor_thread_;
};

} // namespace radar_camera::recording_node
