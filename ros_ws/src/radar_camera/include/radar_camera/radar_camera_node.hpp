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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "hikcamera/shared_frame_reader.hpp"

#include "radar_camera/armor_refine.hpp"
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
    recording::RecordingConfig& recording, armor_refine::ArmorRefineConfig& armor,
    armor_refine::NumberRefineConfig& number) -> std::expected<void, std::string>;

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

    /// @brief 查询 TF map→camera_optical_frame 更新投影外参（GICP + 安装外参）
    void update_camera_extrinsic_from_tf();

    std::chrono::steady_clock::time_point capture_timestamp_;
    rclcpp::Publisher<radar_interfaces::msg::CameraDetectionPose>::SharedPtr pose_publisher_;

    hikcamera::SharedFrameReader shm_reader_;
    std::atomic<bool> infer_running_ { false };
    std::thread infer_thread_;

    // TF: map→radar_base（radar_lidar GICP 发布）+ radar_base→camera_optical（static）
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool tf_ready_             = false;
    std::uint64_t frame_count_ = 0;

    camera_config::CameraConfig camera_config_;
    inference_config::InferenceConfig inference_config_;
    projection_config::ProjectionConfig projection_config_;
    armor_refine::ArmorRefineConfig armor_refine_config_;
    armor_refine::NumberRefineConfig number_refine_config_;
    robot_pose::RobotPose robot_poses_;
    std::unique_ptr<model_inference::ModelInference> model_inference_;
    armor_refine::ArmorRefiner armor_refiner_;
    bool armor_refine_enabled_ = false;
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
