// camera_recorder_node.cpp — Live camera RGB→BGR8 recorder via HIK SHM
//
// Task 4 of the FAST-LIVO2 RGB map/replay plan.
//
// Reads full-resolution RGB frames from the hikcamera shared-memory ring
// buffer (PixelType_Gvsp_RGB8_Packed), converts to BGR8 in one cvtColor
// pass directly into preallocated ROS image data, and publishes as
// sensor_msgs/Image (bgr8) + latched CameraInfo.
//
// Dependencies: hikcamera SDK (SHM functions), OpenCV, rclcpp, sensor_msgs.
//
// Does NOT modify HIK SDK/driver/submodules, the SHM writer, FAST-LIVO2
// code, the colorizer node, launch files, or docs.

#include <hikcamera/shm.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "radar_fast_livo2_rgb/camera_frame.hpp"

namespace {

using namespace std::chrono_literals;

// ════════════════════════════════════════════════════════════════════════
// CameraInfo construction from measured calibration
// ════════════════════════════════════════════════════════════════════════

auto build_camera_info(
    uint32_t width, uint32_t height,
    double fx, double fy, double cx, double cy,
    const std::vector<double>& distortion,
    const std::string& frame_id) -> sensor_msgs::msg::CameraInfo
{
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frame_id;
    info.height = height;
    info.width = width;
    info.distortion_model = "plumb_bob";

    // D: [k1, k2, p1, p2, k3]
    info.d.assign(distortion.begin(), distortion.end());

    // K: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    info.k.fill(0.0);
    info.k[0] = fx;  info.k[2] = cx;
    info.k[4] = fy;  info.k[5] = cy;
    info.k[8] = 1.0;

    // R: identity (no rectification)
    info.r.fill(0.0);
    info.r[0] = 1.0;
    info.r[4] = 1.0;
    info.r[8] = 1.0;

    // P: same as K with zero last column
    info.p.fill(0.0);
    info.p[0] = fx;  info.p[2] = cx;
    info.p[5] = fy;  info.p[6] = cy;
    info.p[10] = 1.0;

    return info;
}

} // anonymous namespace

// ════════════════════════════════════════════════════════════════════════
// CameraRecorderNode
//
// Fix H4-1: Uses a persistent SHM mapping (SHMGetPtr) instead of the
// per-frame SHMRead which maps/unmaps on every iteration.  The pointer
// is released in the destructor after joining the recorder thread.
// ════════════════════════════════════════════════════════════════════════

namespace radar::fast_livo2::rgb {

class CameraRecorderNode final : public rclcpp::Node {
public:
    CameraRecorderNode()
        : Node("camera_recorder_node")
    {
        load_parameters();
        open_shm();
        create_publishers();

        running_.store(true, std::memory_order_release);
        record_thread_ = std::thread(&CameraRecorderNode::record_loop, this);

        RCLCPP_INFO(get_logger(),
            "CameraRecorderNode started: %dx%d RGB→BGR8 via SHM '%s', "
            "publishing to %s (image) and %s (camera_info)",
            width_, height_, shm_name_.c_str(),
            image_pub_->get_topic_name(),
            camera_info_pub_->get_topic_name());
    }

    ~CameraRecorderNode() override
    {
        running_.store(false, std::memory_order_release);
        if (record_thread_.joinable()) {
            record_thread_.join();
        }

        // Release pointer first (order matters: unmap before close)
        if (shm_ptr_ != nullptr) {
            std::ignore = hikcamera::SHMReleasePtr(shm_ptr_);
            shm_ptr_ = nullptr;
        }
        if (shm_fd_ != -1) {
            std::ignore = hikcamera::SHMClose(shm_fd_);
            shm_fd_ = -1;
        }
    }

private:
    // ── Parameter loading ──────────────────────────────────────────────

    void load_parameters()
    {
        declare_parameter("width", 5472);
        declare_parameter("height", 3648);
        declare_parameter("img_time_offset", 0.0);
        declare_parameter("shm_name", "/hikcamera_shm");
        declare_parameter("camera_frame_id", "camera");

        // Calibration (measured, must be non-placeholder)
        declare_parameter("fx", 0.0);
        declare_parameter("fy", 0.0);
        declare_parameter("cx", 0.0);
        declare_parameter("cy", 0.0);
        declare_parameter("distortion", std::vector<double>{});

        width_ = get_parameter("width").as_int();
        height_ = get_parameter("height").as_int();
        img_time_offset_ = get_parameter("img_time_offset").as_double();
        shm_name_ = get_parameter("shm_name").as_string();
        camera_frame_id_ = get_parameter("camera_frame_id").as_string();

        // ── Fix M4-3: validate image fits in SHM slot ──
        if (auto err = validate_image_size(
                width_, height_,
                static_cast<size_t>(MAX_IMAGE_SIZE))) {
            RCLCPP_FATAL(get_logger(), "Image size rejected: %s", err->c_str());
            throw std::runtime_error("Image size validation failed: " + *err);
        }

        double fx = get_parameter("fx").as_double();
        double fy = get_parameter("fy").as_double();
        double cx = get_parameter("cx").as_double();
        double cy = get_parameter("cy").as_double();
        auto distortion = get_parameter("distortion").as_double_array();

        if (auto err = validate_calibration(fx, fy, cx, cy, distortion)) {
            RCLCPP_FATAL(get_logger(), "Calibration rejected: %s", err->c_str());
            throw std::runtime_error("Calibration validation failed: " + *err);
        }

        camera_info_msg_ = build_camera_info(
            static_cast<uint32_t>(width_), static_cast<uint32_t>(height_),
            fx, fy, cx, cy, distortion, camera_frame_id_);

        RCLCPP_INFO(get_logger(),
            "Calibration loaded: fx=%.2f fy=%.2f cx=%.2f cy=%.2f, "
            "distortion=[%.5f %.5f %.5f %.5f %.5f]",
            fx, fy, cx, cy,
            distortion[0], distortion[1], distortion[2],
            distortion[3], distortion[4]);

        // Pre-compute the image byte count for the record loop
        image_bytes_ = static_cast<size_t>(width_) * static_cast<size_t>(height_) * 3;
    }

    // ── SHM open (Fix H4-1: persistent mapping) ────────────────────────

    void open_shm()
    {
        auto fd_ret = hikcamera::SHMInit(shm_name_, sizeof(hikcamera::imageSHM));
        if (!fd_ret.has_value()) {
            throw std::runtime_error(
                "SHMInit('" + shm_name_ + "') failed: " + fd_ret.error());
        }
        shm_fd_ = fd_ret.value();

        auto ptr_ret = hikcamera::SHMGetPtr(shm_fd_);
        if (!ptr_ret.has_value()) {
            std::ignore = hikcamera::SHMClose(shm_fd_);
            shm_fd_ = -1;
            throw std::runtime_error(
                "SHMGetPtr failed: " + ptr_ret.error());
        }
        shm_ptr_ = ptr_ret.value();

        RCLCPP_INFO(get_logger(),
            "SHM persistent mapping: fd=%d ptr=%p name='%s'",
            shm_fd_, static_cast<void*>(shm_ptr_), shm_name_.c_str());
    }

    // ── Publishers ─────────────────────────────────────────────────────

    void create_publishers()
    {
        // BGR frames: bounded queue, best-effort so a slow consumer
        // cannot block acquisition.
        auto image_qos = rclcpp::QoS(rclcpp::KeepLast(2))
            .best_effort()
            .durability_volatile();

        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/fast_livo2/camera/bgr8", image_qos);

        // CameraInfo: transient-local reliable — new subscribers
        // receive the latest calibration automatically.
        auto cinfo_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliable()
            .transient_local();

        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/fast_livo2/camera/camera_info", cinfo_qos);
    }

    // ── Recording loop (semaphore-free polling via frame_counter) ───
    //
    // The SHM semaphore is a single-consumer resource already consumed
    // by existing readers (radar_bridge via SHMRead).  This recorder
    // polls frame_counter (atomic acquire) instead, sleeping 1 ms
    // between checks when no new frame is available.
    //
    // Post-copy stability validation with bounded retry (kMaxCopyRetries=3)
    // detects concurrent writer advancement during the copy.  After the
    // retry loop, last_seen advances past the latest observed counter
    // whether stable or not, preventing re-processing of stale frames.

    void record_loop()
    {
        sensor_msgs::msg::Image image_msg;
        image_msg.data.resize(image_bytes_);

        constexpr int kPollIntervalMs = 1;
        constexpr int kMaxCopyRetries = 3;

        uint64_t last_seen = 0;

        while (running_.load(std::memory_order_acquire)) {
            uint64_t counter =
                shm_ptr_->frame_counter.load(std::memory_order_acquire);

            if (!should_process_counter(counter, last_seen)) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(kPollIntervalMs));
                continue;
            }

            // Counter advanced: at least one new completed frame.
            // Try to capture a stable copy with bounded retry.
            bool published = false;
            uint64_t latest_seen = counter;

            for (int retry = 0; retry < kMaxCopyRetries; ++retry) {
                uint64_t counter_before =
                    shm_ptr_->frame_counter.load(std::memory_order_acquire);
                if (!is_valid_frame_counter(counter_before)) break;
                latest_seen = counter_before;

                unsigned int slot = completed_slot_from_counter(
                    counter_before, SLOT_NUM);

                // ── Copy directly from SHM slot, converting RGB→BGR ──
                // SHM stores PixelType_Gvsp_RGB8_Packed (R,G,B per pixel).
                // We publish bgr8 (B,G,R), so one cvtColor pass transforms
                // directly into preallocated image_msg.data with no
                // intermediate full-frame buffer.
                {
                    cv::Mat rgb_shm(height_, width_, CV_8UC3,
                                    shm_ptr_->imagedata[slot]);  // no-copy header
                    cv::Mat bgr_out(height_, width_, CV_8UC3,
                                    image_msg.data.data());      // preallocated
                    cv::cvtColor(rgb_shm, bgr_out, cv::COLOR_RGB2BGR);
                }
                auto ts_monotonic_copy = shm_ptr_->timestamp[slot];

                uint64_t counter_after =
                    shm_ptr_->frame_counter.load(std::memory_order_acquire);
                latest_seen = counter_after;

                if (is_frame_stable(counter_before, counter_after)) {
                    uint64_t host_ns = static_cast<uint64_t>(
                        ts_monotonic_copy.time_since_epoch().count());
                    auto ros_stamp = camera_stamp_from_shm(
                        host_ns, img_time_offset_);

                    fill_bgr8_image_metadata(
                        image_msg, width_, height_,
                        ros_stamp, camera_frame_id_);
                    image_pub_->publish(image_msg);

                    camera_info_msg_.header.stamp = ros_stamp;
                    camera_info_pub_->publish(camera_info_msg_);

                    published = true;
                    break;
                }
            }

            // Advance past the latest observed counter whether stable
            // or not — prevents re-processing stale frames.
            last_seen = latest_seen;

            if (!published) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "frame_counter unstable after %d copy retries; "
                    "skipping to counter %lu",
                    kMaxCopyRetries,
                    static_cast<unsigned long>(latest_seen));
            }
        }
    }

    // ── Members ────────────────────────────────────────────────────────

    int width_ = 5472;
    int height_ = 3648;
    size_t image_bytes_ = 0;          // pre-computed: width * height * 3
    double img_time_offset_ = 0.0;
    std::string shm_name_;
    std::string camera_frame_id_;

    sensor_msgs::msg::CameraInfo camera_info_msg_;

    int shm_fd_ = -1;
    hikcamera::imageSHM* shm_ptr_ = nullptr;  // Fix H4-1: persistent mapping
    std::atomic<bool> running_{false};
    std::thread record_thread_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
};

} // namespace radar::fast_livo2::rgb

// ════════════════════════════════════════════════════════════════════════
// main
// ════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<radar::fast_livo2::rgb::CameraRecorderNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("camera_recorder_node"),
            "Startup failed: %s", e.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
