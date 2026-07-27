// rgb_colorizer_node.cpp — RGB colorizer: live SHM or replay ROS image input
//
// Task 5 of the FAST-LIVO2 RGB map/replay plan.
//
// Consumes:
//   /fast_livo2/cloud_world    — sensor_msgs::PointCloud2  (world-frame)
//   /fast_livo2/odom           — nav_msgs::Odometry         (T_world_lidar)
//   RGB frames from SHM (live) or /fast_livo2/camera/bgr8 (replay)
//
// Produces:
//   /fast_livo2/cloud_rgb_map  — sensor_msgs::PointCloud2  (odom-frame, PointXYZRGB)
//   Binary PCD files on trigger polling

#include <hikcamera/shm.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "radar_fast_livo2_rgb/camera_frame.hpp"
#include "radar_fast_livo2_rgb/color_voxel_map.hpp"
#include "radar_fast_livo2_rgb/pcd_trigger.hpp"
#include "radar_fast_livo2_rgb/rgb_colorizer.hpp"
#include "radar_fast_livo2_rgb/rgb_projection.hpp"

namespace radar::fast_livo2::rgb {

namespace {

using namespace std::chrono_literals;

// ════════════════════════════════════════════════════════════════════════
// Timestamped wrappers for the image queue and odometry cache.
// ════════════════════════════════════════════════════════════════════════

struct TimestampedImage {
    cv::Mat data;          // owned (BGR for replay, RGB for live SHM)
    rclcpp::Time stamp;
    bool source_is_rgb = false;  // true if data came from live SHM (RGB byte order)
};

struct TimestampedOdom {
    Eigen::Isometry3d pose;  // T_world_lidar
    rclcpp::Time stamp;
};

// ════════════════════════════════════════════════════════════════════════
// Odometry → Isometry3d conversion
// ════════════════════════════════════════════════════════════════════════

auto odom_to_isometry(const nav_msgs::msg::Odometry& msg) -> Eigen::Isometry3d
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z);
    Eigen::Quaterniond q(
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z);
    pose.linear() = q.toRotationMatrix();
    return pose;
}

} // anonymous namespace

// ════════════════════════════════════════════════════════════════════════
// RgbColorizerNode
// ════════════════════════════════════════════════════════════════════════

class RgbColorizerNode final : public rclcpp::Node {
public:
    RgbColorizerNode()
        : Node("rgb_colorizer_node")
    {
        load_parameters();

        // ── Calibration ────────────────────────────────────────────────
        calibration_.fx = fx_;
        calibration_.fy = fy_;
        calibration_.cx = cx_;
        calibration_.cy = cy_;
        calibration_.rotation_lidar_camera = rotation_lidar_camera_;
        calibration_.translation_lidar_camera = translation_lidar_camera_;
        calibration_.quality_weights = quality_weights_;

        // Construct the color map AFTER all parameters are loaded.
        color_map_ = std::make_unique<ColorVoxelMap>(voxel_size_);

        if (!validate_calibration(calibration_)) {
            RCLCPP_FATAL(get_logger(),
                "Calibration rejected: intrinsics or extrinsics invalid");
            throw std::runtime_error("Calibration validation failed");
        }

        // ── Subscribers ────────────────────────────────────────────────
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/fast_livo2/cloud_world",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                on_cloud_world(std::move(msg));
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/fast_livo2/odom",
            rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                on_odometry(std::move(msg));
            });

        // ── Camera input ───────────────────────────────────────────────
        start_camera_input();

        // ── Publishers ─────────────────────────────────────────────────
        auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliable()
            .durability_volatile();

        cloud_rgb_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/fast_livo2/cloud_rgb_map", cloud_qos);

        // ── Timers ─────────────────────────────────────────────────────
        publish_timer_ = create_wall_timer(
            std::chrono::milliseconds(publish_interval_ms_),
            [this]() { on_publish_timer(); });

        pcd_timer_ = create_wall_timer(
            std::chrono::milliseconds(pcd_save_interval_ms_),
            [this]() { on_pcd_timer(); });

        RCLCPP_INFO(get_logger(),
            "RgbColorizerNode ready: mode=%s voxel=%.2fm "
            "%dx%d fx=%.1f fy=%.1f",
            camera_input_mode_.c_str(), voxel_size_,
            camera_width_, camera_height_, fx_, fy_);
    }

    ~RgbColorizerNode() override
    {
        publish_timer_->cancel();
        pcd_timer_->cancel();
        stop_camera_input();
    }

private:
    // ── Parameters ─────────────────────────────────────────────────────

    void load_parameters()
    {
        declare_parameter("camera_input_mode", "ros_image");
        declare_parameter("shm_name", "/hikcamera_shm");
        declare_parameter("img_time_offset", 0.0);
        declare_parameter("camera_image_topic", "/fast_livo2/camera/bgr8");
        declare_parameter("camera_width", 5472);
        declare_parameter("camera_height", 3648);

        // Intrinsics
        declare_parameter("fx", 0.0);
        declare_parameter("fy", 0.0);
        declare_parameter("cx", 0.0);
        declare_parameter("cy", 0.0);

        // LiDAR→camera extrinsics (rotation matrix, translation vector)
        declare_parameter("rotation_lidar_camera",
            std::vector<double>{
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0});
        declare_parameter("translation_lidar_camera",
            std::vector<double>{0.0, 0.0, 0.0});

        // Quality weights
        declare_parameter("quality_axis_alignment", 1.0);
        declare_parameter("quality_inverse_distance", 1.0);
        declare_parameter("quality_image_center", 0.5);
        declare_parameter("quality_gradient", 0.25);

        // Map parameters
        declare_parameter("voxel_size", 0.10);
        declare_parameter("max_image_queue", 5);
        declare_parameter("max_odom_cache", 200);
        declare_parameter("time_tolerance_ms", 50);
        declare_parameter("publish_interval_ms", 1000);
        declare_parameter("pcd_save_interval_ms", 5000);
        declare_parameter("pcd_save_dir", "/tmp");
        declare_parameter("pcd_trigger", false);

        camera_input_mode_ = get_parameter("camera_input_mode").as_string();
        shm_name_ = get_parameter("shm_name").as_string();
        img_time_offset_ = get_parameter("img_time_offset").as_double();
        camera_image_topic_ = get_parameter("camera_image_topic").as_string();
        camera_width_ = get_parameter("camera_width").as_int();
        camera_height_ = get_parameter("camera_height").as_int();

        fx_ = get_parameter("fx").as_double();
        fy_ = get_parameter("fy").as_double();
        cx_ = get_parameter("cx").as_double();
        cy_ = get_parameter("cy").as_double();

        auto rot = get_parameter("rotation_lidar_camera").as_double_array();
        auto trans = get_parameter("translation_lidar_camera").as_double_array();
        if (rot.size() != 9 || trans.size() != 3) {
            throw std::runtime_error(
                "rotation_lidar_camera must have 9 elements, "
                "translation_lidar_camera must have 3");
        }
        rotation_lidar_camera_ << rot[0], rot[1], rot[2],
                                  rot[3], rot[4], rot[5],
                                  rot[6], rot[7], rot[8];
        translation_lidar_camera_ << trans[0], trans[1], trans[2];

        quality_weights_.axis_alignment = get_parameter("quality_axis_alignment").as_double();
        quality_weights_.inverse_distance = get_parameter("quality_inverse_distance").as_double();
        quality_weights_.image_center = get_parameter("quality_image_center").as_double();
        quality_weights_.gradient = get_parameter("quality_gradient").as_double();

        voxel_size_ = get_parameter("voxel_size").as_double();

        if (!is_finite_positive(voxel_size_)) {
            throw std::runtime_error(
                "voxel_size must be finite and strictly positive, got "
                + std::to_string(voxel_size_));
        }

        max_image_queue_ = static_cast<size_t>(get_parameter("max_image_queue").as_int());
        max_odom_cache_ = static_cast<size_t>(get_parameter("max_odom_cache").as_int());
        time_tolerance_ms_ = get_parameter("time_tolerance_ms").as_int();
        publish_interval_ms_ = get_parameter("publish_interval_ms").as_int();
        pcd_save_interval_ms_ = get_parameter("pcd_save_interval_ms").as_int();
        pcd_save_dir_ = get_parameter("pcd_save_dir").as_string();

        // Validate mode
        if (camera_input_mode_ != "shm" && camera_input_mode_ != "ros_image") {
            throw std::runtime_error(
                "camera_input_mode must be 'shm' or 'ros_image', got: "
                + camera_input_mode_);
        }

        // Validate intrinsic/extrinsic placeholders
        if (!is_finite_positive(fx_)) {
            throw std::runtime_error("fx must be finite and positive");
        }
        if (!is_finite_positive(fy_)) {
            throw std::runtime_error("fy must be finite and positive");
        }
        if (!is_finite_positive(cx_)) {
            throw std::runtime_error("cx must be finite and positive");
        }
        if (!is_finite_positive(cy_)) {
            throw std::runtime_error("cy must be finite and positive");
        }
        if (!rotation_lidar_camera_.allFinite()) {
            throw std::runtime_error("rotation_lidar_camera contains non-finite values");
        }
        if (!translation_lidar_camera_.allFinite()) {
            throw std::runtime_error("translation_lidar_camera contains non-finite values");
        }

        // Validate PCD save directory exists and is a directory.
        {
            std::error_code ec;
            if (!std::filesystem::is_directory(pcd_save_dir_, ec)) {
                if (ec) {
                    throw std::runtime_error(
                        "PCD save directory check failed: "
                        + pcd_save_dir_ + " — " + ec.message());
                }
                throw std::runtime_error(
                    "PCD save directory does not exist or is not a directory: "
                    + pcd_save_dir_);
            }
        }
    }

    // ── Camera input startup ───────────────────────────────────────────

    void start_camera_input()
    {
        if (camera_input_mode_ == "shm") {
            open_shm();
            shm_running_.store(true, std::memory_order_release);
            shm_thread_ = std::thread(&RgbColorizerNode::shm_loop, this);
        } else {
            // ros_image mode: subscribe to BGR8 topic
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                camera_image_topic_,
                rclcpp::SensorDataQoS(),
                [this](sensor_msgs::msg::Image::SharedPtr msg) {
                    on_image(msg);
                });
            RCLCPP_INFO(get_logger(),
                "Subscribed to camera image topic: %s",
                camera_image_topic_.c_str());
        }
    }

    void stop_camera_input()
    {
        if (shm_thread_.joinable()) {
            shm_running_.store(false, std::memory_order_release);
            shm_thread_.join();
        }
        if (shm_ptr_ != nullptr) {
            std::ignore = hikcamera::SHMReleasePtr(shm_ptr_);
            shm_ptr_ = nullptr;
        }
        if (shm_fd_ != -1) {
            std::ignore = hikcamera::SHMClose(shm_fd_);
            shm_fd_ = -1;
        }
    }

    // ── SHM (live mode) ────────────────────────────────────────────────

    void open_shm()
    {
        auto fd_ret = hikcamera::SHMInit(
            shm_name_, sizeof(hikcamera::imageSHM));
        if (!fd_ret.has_value()) {
            throw std::runtime_error(
                "SHMInit('" + shm_name_ + "') failed: " + fd_ret.error());
        }
        shm_fd_ = fd_ret.value();

        auto ptr_ret = hikcamera::SHMGetPtr(shm_fd_);
        if (!ptr_ret.has_value()) {
            std::ignore = hikcamera::SHMClose(shm_fd_);
            shm_fd_ = -1;
            throw std::runtime_error("SHMGetPtr failed: " + ptr_ret.error());
        }
        shm_ptr_ = ptr_ret.value();

        image_bytes_ = static_cast<size_t>(camera_width_)
                     * static_cast<size_t>(camera_height_) * 3;

        RCLCPP_INFO(get_logger(),
            "SHM opened: fd=%d ptr=%p name='%s' %dx%d (%zu bytes)",
            shm_fd_, static_cast<void*>(shm_ptr_), shm_name_.c_str(),
            camera_width_, camera_height_, image_bytes_);
    }

    /// Semaphore-free polling loop — same protocol as Task 4 recorder.
    /// Does NOT consume imageSHM::sem (preserves it for radar_bridge).
    /// Does NOT lock the SHM mutex (writer doesn't participate).
    /// Makes ONE full RGB copy per frame into an owned cv::Mat.
    void shm_loop()
    {
        constexpr int kPollIntervalMs = 1;
        constexpr int kMaxCopyRetries = 3;
        uint64_t last_seen = 0;

        while (shm_running_.load(std::memory_order_acquire)) {
            uint64_t counter =
                shm_ptr_->frame_counter.load(std::memory_order_acquire);

            if (!should_process_counter(counter, last_seen)) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(kPollIntervalMs));
                continue;
            }

            uint64_t latest_seen = counter;
            bool pushed = false;

            for (int retry = 0; retry < kMaxCopyRetries; ++retry) {
                uint64_t counter_before =
                    shm_ptr_->frame_counter.load(std::memory_order_acquire);
                if (!is_valid_frame_counter(counter_before)) break;
                latest_seen = counter_before;

                unsigned int slot = completed_slot_from_counter(
                    counter_before, SLOT_NUM);

                // One full copy into owned cv::Mat.
                cv::Mat rgb(camera_height_, camera_width_, CV_8UC3);
                std::memcpy(rgb.data,
                            shm_ptr_->imagedata[slot],
                            image_bytes_);

                auto ts_monotonic = shm_ptr_->timestamp[slot];
                uint64_t host_ns = static_cast<uint64_t>(
                    ts_monotonic.time_since_epoch().count());
                auto ros_stamp = camera_stamp_from_shm(
                    host_ns, img_time_offset_);

                uint64_t counter_after =
                    shm_ptr_->frame_counter.load(std::memory_order_acquire);
                latest_seen = counter_after;

                if (is_frame_stable(counter_before, counter_after)) {
                    push_image(std::move(rgb), ros_stamp, /*source_is_rgb=*/true);
                    pushed = true;
                    break;
                }
            }

            last_seen = latest_seen;

            if (!pushed) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "SHM frame_counter unstable after %d retries; "
                    "skipping to counter %lu",
                    kMaxCopyRetries,
                    static_cast<unsigned long>(latest_seen));
            }
        }
    }

    // ── ROS image callback (replay mode) ───────────────────────────────

    void on_image(sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Odin driver firmware may omit Image.encoding for this stream. The
        // payload is still the BGR8 decoded image created by publishRgb().
        if (!msg->encoding.empty() && msg->encoding != "bgr8" && msg->encoding != "rgb8") {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Expected bgr8 or rgb8 encoding, got '%s'", msg->encoding.c_str());
            return;
        }
        if (msg->data.empty()) return;

        // Copy row by row because sensor_msgs/Image may include padding in
        // step. The empty encoding used by some Odin firmware is the same
        // BGR8 payload emitted by publishRgb().
        const size_t row_bytes = static_cast<size_t>(msg->width) * 3U;
        if (msg->width == 0 || msg->height == 0 || msg->step < row_bytes) return;
        const size_t required_bytes =
            (static_cast<size_t>(msg->height) - 1U) * msg->step + row_bytes;
        if (msg->data.size() < required_bytes) return;

        cv::Mat color(
            static_cast<int>(msg->height),
            static_cast<int>(msg->width),
            CV_8UC3);
        for (size_t row = 0; row < msg->height; ++row) {
            std::memcpy(
                color.ptr(static_cast<int>(row)),
                msg->data.data() + row * msg->step,
                row_bytes);
        }

        rclcpp::Time stamp(msg->header.stamp);
        bool is_rgb = (msg->encoding == "rgb8");
        push_image(std::move(color), stamp, is_rgb);
    }

    // ── Image queue ────────────────────────────────────────────────────

    void push_image(cv::Mat color, const rclcpp::Time& stamp, bool source_is_rgb = false)
    {
        std::lock_guard<std::mutex> lock(image_queue_mutex_);
        image_queue_.push_back({std::move(color), stamp, source_is_rgb});
        while (image_queue_.size() > max_image_queue_) {
            image_queue_.pop_front();
        }
    }

    auto find_nearest_image(const rclcpp::Time& target) const
        -> std::optional<TimestampedImage>
    {
        // Phase 1 (under lock): find best match, capture a shallow
        // cv::Mat copy (ref-counted shared data) — no deep copy here.
        cv::Mat snapshot;
        rclcpp::Time best_stamp;
        bool source_is_rgb = false;
        {
            std::lock_guard<std::mutex> lock(image_queue_mutex_);
            if (image_queue_.empty()) return std::nullopt;

            auto tolerance = rclcpp::Duration::from_nanoseconds(
                static_cast<int64_t>(time_tolerance_ms_) * 1'000'000LL);

            const TimestampedImage* best = nullptr;
            rclcpp::Duration best_diff = rclcpp::Duration::from_nanoseconds(
                std::numeric_limits<int64_t>::max());

            for (const auto& img : image_queue_) {
                auto abs_ns = std::abs((target - img.stamp).nanoseconds());
                auto abs_dur = rclcpp::Duration::from_nanoseconds(abs_ns);
                if (abs_dur <= tolerance && abs_dur < best_diff) {
                    best_diff = abs_dur;
                    best = &img;
                }
            }

            if (best == nullptr) return std::nullopt;

            // Shallow copy: cv::Mat ref-counts underlying data, so the
            // pixel buffer stays alive even after the lock is released
            // and the deque entry is potentially evicted by another thread.
            snapshot = best->data;
            best_stamp = best->stamp;
            source_is_rgb = best->source_is_rgb;
        }

        // Phase 2 (no lock): deep-clone into an owned cv::Mat.
        // snapshot still holds a ref count on the pixel data.
        return TimestampedImage{
            .data = snapshot.clone(),
            .stamp = best_stamp,
            .source_is_rgb = source_is_rgb
        };
    }

    // ── Odometry cache ─────────────────────────────────────────────────

    void on_odometry(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_cache_mutex_);
        odom_cache_.push_back({
            odom_to_isometry(*msg),
            rclcpp::Time(msg->header.stamp)
        });
        while (odom_cache_.size() > max_odom_cache_) {
            odom_cache_.pop_front();
        }
    }

    auto find_nearest_odom(const rclcpp::Time& target) const
        -> std::optional<TimestampedOdom>
    {
        std::lock_guard<std::mutex> lock(odom_cache_mutex_);
        if (odom_cache_.empty()) return std::nullopt;

        auto tolerance_ns = static_cast<int64_t>(time_tolerance_ms_) * 1'000'000LL;

        const TimestampedOdom* best = nullptr;
        rclcpp::Duration best_diff = rclcpp::Duration::from_nanoseconds(
            std::numeric_limits<int64_t>::max());

        for (const auto& odom : odom_cache_) {
            auto abs_ns = std::abs((target - odom.stamp).nanoseconds());
            if (!within_tolerance_ns(abs_ns, tolerance_ns)) continue;
            auto abs_dur = rclcpp::Duration::from_nanoseconds(abs_ns);
            if (abs_dur < best_diff) {
                best_diff = abs_dur;
                best = &odom;
            }
        }

        if (best == nullptr) return std::nullopt;
        return *best;
    }

    // ── Cloud world callback ───────────────────────────────────────────

    void on_cloud_world(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Extract world points from PointCloud2.
        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*msg, pcl_cloud);

        std::vector<Eigen::Vector3d> world_points;
        world_points.reserve(pcl_cloud.size());
        for (const auto& pt : pcl_cloud.points) {
            world_points.emplace_back(pt.x, pt.y, pt.z);
        }

        if (world_points.empty()) return;

        rclcpp::Time cloud_stamp(msg->header.stamp);

        // Find nearest image within tolerance.
        auto image_opt = find_nearest_image(cloud_stamp);
        if (!image_opt.has_value()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "No BGR image within %d ms of cloud stamp %lu.%09lu",
                time_tolerance_ms_,
                static_cast<unsigned long>(cloud_stamp.seconds()),
                static_cast<unsigned long>(cloud_stamp.nanoseconds()));
            return;
        }

        // Find nearest odometry.
        auto odom_opt = find_nearest_odom(cloud_stamp);
        if (!odom_opt.has_value()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "No odometry entry near cloud stamp %lu.%09lu",
                static_cast<unsigned long>(cloud_stamp.seconds()),
                static_cast<unsigned long>(cloud_stamp.nanoseconds()));
            return;
        }

        // Colorize.
        ColorFormat fmt = image_opt->source_is_rgb
            ? ColorFormat::RGB : ColorFormat::BGR;
        auto map = color_world_points(
            world_points,
            image_opt->data,
            odom_opt->pose,
            calibration_,
            quality_weights_,
            voxel_size_,
            fmt);

        // Merge into the shared map, preserving per-voxel quality scores.
        {
            std::lock_guard<std::mutex> lock(color_map_mutex_);
            map.for_each_voxel(
                [this](const Eigen::Vector3d& pos,
                       uint32_t rgb, double quality) {
                    color_map_->insert_if_better(pos, rgb, quality);
                });
        }
    }

    // ── Periodic publish ───────────────────────────────────────────────

    void on_publish_timer()
    {
        std::lock_guard<std::mutex> lock(color_map_mutex_);
        publish_current_map();
    }

    void publish_current_map()
    {
        auto cloud = color_map_->to_point_cloud();
        if (cloud.empty()) return;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = now();
        msg.header.frame_id = "odom";
        cloud_rgb_pub_->publish(msg);
    }

    // ── PCD save trigger (polling) ─────────────────────────────────────

    void on_pcd_timer()
    {
        bool trigger = get_parameter("pcd_trigger").as_bool();
        if (!trigger) return;

        int save_result = 0;
        {
            std::lock_guard<std::mutex> lock(color_map_mutex_);
            save_result = save_pcd_impl("trigger");
        }

        bool new_trigger = pcd_trigger_transition(trigger, save_result);
        set_parameter(rclcpp::Parameter("pcd_trigger", new_trigger));

        if (new_trigger) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                "PCD save failed; trigger retained for retry");
        }
    }

    int save_pcd_impl(const std::string& tag)
    {
        int64_t total_ns = now().nanoseconds();
        int32_t sec = static_cast<int32_t>(total_ns / 1'000'000'000LL);
        uint32_t nsec = static_cast<uint32_t>(total_ns % 1'000'000'000LL);

        char filename[512];
        std::snprintf(filename, sizeof(filename),
            "%s/cloud_rgb_%s_%d_%09u.pcd",
            pcd_save_dir_.c_str(),
            tag.c_str(),
            sec,
            nsec);

        int ret = color_map_->save_binary_pcd(filename);
        if (ret == 0) {
            RCLCPP_INFO(get_logger(),
                "PCD saved: %s (%zu voxels)",
                filename, color_map_->size());
        } else {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000,
                "PCD save failed: %s (code %d)", filename, ret);
        }
        return ret;
    }

    // ── Members ────────────────────────────────────────────────────────

    // Parameters
    std::string camera_input_mode_;
    std::string shm_name_;
    double img_time_offset_ = 0.0;
    std::string camera_image_topic_;
    int camera_width_ = 5472;
    int camera_height_ = 3648;
    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
    Eigen::Matrix3d rotation_lidar_camera_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation_lidar_camera_ = Eigen::Vector3d::Zero();
    QualityWeights quality_weights_;
    double voxel_size_ = 0.10;
    size_t max_image_queue_ = 5;
    size_t max_odom_cache_ = 200;
    int time_tolerance_ms_ = 50;
    int publish_interval_ms_ = 1000;
    int pcd_save_interval_ms_ = 5000;
    std::string pcd_save_dir_;

    // Calibration (built from parameters at startup)
    Calibration calibration_;

    // Color voxel map (thread-safe) — constructed after load_parameters()
    std::unique_ptr<ColorVoxelMap> color_map_;
    mutable std::mutex color_map_mutex_;

    // Image queue (bounded)
    mutable std::mutex image_queue_mutex_;
    std::deque<TimestampedImage> image_queue_;

    // Odometry cache
    mutable std::mutex odom_cache_mutex_;
    std::deque<TimestampedOdom> odom_cache_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_rgb_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr pcd_timer_;

    // SHM (live mode)
    int shm_fd_ = -1;
    hikcamera::imageSHM* shm_ptr_ = nullptr;
    std::atomic<bool> shm_running_{false};
    std::thread shm_thread_;
    size_t image_bytes_ = 0;
};

} // namespace radar::fast_livo2::rgb

// ════════════════════════════════════════════════════════════════════════
// main
// ════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<
            radar::fast_livo2::rgb::RgbColorizerNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("rgb_colorizer_node"),
            "Startup failed: %s", e.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
