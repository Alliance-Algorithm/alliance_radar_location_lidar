#include "radar_camera/radar_recorder_node.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>

namespace radar_camera::recording_node {

namespace {
    class ConstructorCleanupGuard {
    public:
        explicit ConstructorCleanupGuard(std::function<void()> cleanup)
            : cleanup_(std::move(cleanup)) { }
        ~ConstructorCleanupGuard() { cleanup_(); }
        void release() noexcept {
            cleanup_ = [] { };
        }

    private:
        std::function<void()> cleanup_;
    };
} // namespace

RadarRecorderNode::RadarRecorderNode()
    : Node("radar_recorder_node") {
    auto cleanup_guard = ConstructorCleanupGuard([this]() noexcept { constructor_cleanup(); });
    auto ret = RecordingConfigsLoader(*this, recording_config_, shm_name_, width_, height_);
    if (!ret) {
        RCLCPP_ERROR(get_logger(), "RecordingConfigsLoader failed: %s", ret.error().c_str());
        throw std::runtime_error("RecordingConfigsLoader failed: " + ret.error());
    }
    RCLCPP_INFO(get_logger(), "RecordingConfigsLoader succeeded");

    if (!recording_config_.enabled) {
        RCLCPP_INFO(get_logger(), "Raw recording disabled; recorder node idle");
        cleanup_guard.release();
        return;
    }

    auto components = make_components(recording_config_, shm_name_);
    fifo_           = std::move(components.fifo);
    recorder_       = std::move(components.recorder);
    reader_         = std::move(components.reader);

    for (const auto component : lifecycle_order()) {
        switch (component) {
        case LifecycleComponent::recorder: {
            auto recorder_ret = recorder_->start();
            if (!recorder_ret) {
                throw std::runtime_error("RawVideoRecorder start failed: " + recorder_ret.error());
            }
            break;
        }
        case LifecycleComponent::reader: {
            auto reader_ret = reader_->start();
            if (!reader_ret) {
                throw std::runtime_error("RawShmReader start failed: " + reader_ret.error());
            }
            break;
        }
        case LifecycleComponent::monitor:
            monitor_start();
            break;
        }
    }
    RCLCPP_INFO(get_logger(), "Raw recording started: %s", recording_config_.output_dir.c_str());
    cleanup_guard.release();
}

RadarRecorderNode::~RadarRecorderNode() {
    monitor_stop();
    if (reader_) reader_->stop();
    if (recorder_) recorder_->stop();
}

auto RadarRecorderNode::monitor_start() -> void {
    monitor_running_ = true;
    monitor_thread_  = std::thread([this]() {
        while (monitor_running_.load(std::memory_order_acquire)) {
            const auto recorder_state = recorder_->state();
            const auto reader_state   = reader_->state();
            if (recorder_state == recording::RecorderState::failed
                || recorder_state == recording::RecorderState::overrun
                || reader_state == recording::ReaderState::failed
                || reader_state == recording::ReaderState::overrun) {
                const auto recorder_failed = recorder_state == recording::RecorderState::failed
                    || recorder_state == recording::RecorderState::overrun;
                const auto reason = recorder_failed ? recorder_->failure_reason()
                                                    : reader_->failure_reason();
                RCLCPP_ERROR(get_logger(),
                    "Raw recording failed (state=%s): %s; recorder process exiting (inference "
                    "unaffected)",
                    recorder_state == recording::RecorderState::overrun
                            || reader_state == recording::ReaderState::overrun
                        ? "OVERRUN"
                        : "FAILED",
                    reason.c_str());
                rclcpp::shutdown();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });
}

auto RadarRecorderNode::monitor_stop() -> void {
    monitor_running_ = false;
    if (monitor_thread_.joinable()) monitor_thread_.join();
}

auto RadarRecorderNode::constructor_cleanup() noexcept -> void {
    std::vector<LifecycleComponent> started;
    if (recorder_) started.emplace_back(LifecycleComponent::recorder);
    if (reader_) started.emplace_back(LifecycleComponent::reader);
    if (monitor_thread_.joinable() || monitor_running_)
        started.emplace_back(LifecycleComponent::monitor);

    for (const auto component : cleanup_order(started)) {
        switch (component) {
        case LifecycleComponent::monitor:
            monitor_stop();
            break;
        case LifecycleComponent::reader:
            if (reader_) reader_->stop();
            break;
        case LifecycleComponent::recorder:
            if (recorder_) recorder_->stop();
            break;
        }
    }
}

} // namespace radar_camera::recording_node

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<radar_camera::recording_node::RadarRecorderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
