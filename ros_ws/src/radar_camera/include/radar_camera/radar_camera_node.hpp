#pragma once
#include <atomic>
#include <chrono>
#include <expected>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include <hikcamera/shm.hpp>

#include "radar_camera/data_format.hpp"
#include "radar_camera/model_inference.hpp"
#include "radar_camera/projector.hpp"
#include "radar_camera/raw_shm_reader.hpp"
#include "radar_camera/raw_video_recorder.hpp"
#include "radar_interfaces/msg/camera_detection_pose.hpp"

namespace radar_camera::node {

enum class NodeStatus { starting, running, failed, stopped };
enum class LifecycleComponent { shm, inference, recorder, reader, monitor };

struct RecordingComponents {
    std::unique_ptr<recording::RecordingFifo> fifo;
    std::unique_ptr<recording::RawVideoRecorder> recorder;
    std::unique_ptr<recording::RawShmReader> reader;
};

[[nodiscard]] auto recording_lifecycle_order() -> std::vector<LifecycleComponent>;
[[nodiscard]] auto constructor_cleanup_order(const std::vector<LifecycleComponent>& started)
    -> std::vector<LifecycleComponent>;
[[nodiscard]] auto make_recording_components(
    const recording::RecordingConfig& config, const std::string& shm_name) -> RecordingComponents;

auto ConfigsLoader(rclcpp::Node& node, camera_config::CameraConfig& camera,
    inference_config::InferenceConfig& inference, projection_config::ProjectionConfig& projection,
    recording::RecordingConfig& recording) -> std::expected<void, std::string>;

class RadarCameraNode final : public rclcpp::Node {
public:
    RadarCameraNode();
    ~RadarCameraNode() override;

    auto PublishCallback(const robot_pose::RobotPose& robot_poses) -> void;
    [[nodiscard]] auto status() const -> NodeStatus;
    [[nodiscard]] auto failure_reason() const -> std::string;

private:
    auto constructor_cleanup() noexcept -> void;
    auto infer_thread_start() -> std::expected<void, std::string>;
    auto infer_thread_stop() -> void;
    auto recording_monitor_start() -> void;
    auto recording_monitor_stop() -> void;

    std::chrono::steady_clock::time_point capture_timestamp_;
    rclcpp::Publisher<radar_interfaces::msg::CameraDetectionPose>::SharedPtr pose_publisher_;

    int shm_fd_ = -1;
    std::atomic<bool> infer_running_ { false };
    std::thread infer_thread_;

    camera_config::CameraConfig camera_config_;
    inference_config::InferenceConfig inference_config_;
    projection_config::ProjectionConfig projection_config_;
    robot_pose::RobotPose robot_poses_;
    std::unique_ptr<model_inference::ModelInference> model_inference_;
    projection::Projector projector_;
    recording::RecordingConfig recording_config_ { };
    std::unique_ptr<recording::RecordingFifo> recording_fifo_;
    std::unique_ptr<recording::RawVideoRecorder> raw_video_recorder_;
    std::unique_ptr<recording::RawShmReader> raw_shm_reader_;
    std::atomic<bool> recording_monitor_running_ { false };
    std::thread recording_monitor_thread_;
    std::atomic<NodeStatus> status_ { NodeStatus::starting };
    mutable std::mutex status_mutex_;
    std::string failure_reason_;
};

} // namespace radar_camera::node
