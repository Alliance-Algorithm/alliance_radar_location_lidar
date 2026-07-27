// camera_frame.hpp — Camera-frame helpers for live SHM → ROS bridge
//
// Task 4 of the FAST-LIVO2 RGB map/replay plan.
// Pure helpers that convert hikcamera SHM metadata to ROS messages:
//   - camera_stamp_from_shm():  monotonic → system-time timestamp
//   - make_bgr8_image():         cv::Mat → sensor_msgs/Image (deep copy)
//   - fill_bgr8_image_metadata(): metadata on preallocated Image (for one-copy pipeline)
//   - validate_calibration():    reject placeholder intrinsics / distortion
//   - validate_image_size():     overflow-safe RGB size ≤ max slot capacity
//
// The SHM stores PixelType_Gvsp_RGB8_Packed (R,G,B per pixel).
// The recorder uses cvtColor(COLOR_RGB2BGR) to produce genuine bgr8 output.

#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace radar::fast_livo2::rgb {

/// Convert SHM monotonic clock timestamp to ROS system time.
///
/// host_monotonic_ns: steady_clock::time_point in nanoseconds since epoch
/// img_time_offset:   configured offset in seconds (e.g. 1700000000.25)
///                    that maps the monotonic epoch to system-clock epoch.
///
/// Splits offset into integer-seconds and fractional-seconds to avoid
/// double-precision loss when the offset is a large Unix epoch value.
inline auto camera_stamp_from_shm(
    uint64_t host_monotonic_ns, double img_time_offset)
    -> builtin_interfaces::msg::Time
{
    constexpr double kNanosPerSec = 1'000'000'000.0;

    // Split into integer and fractional seconds so the uint64
    // multiplication of the integer part is exact.
    double int_part = 0.0;
    double frac_part = std::modf(img_time_offset, &int_part);

    uint64_t sec_ns = static_cast<uint64_t>(int_part) * 1'000'000'000ULL;
    uint64_t frac_ns = static_cast<uint64_t>(frac_part * kNanosPerSec);
    uint64_t offset_ns = sec_ns + frac_ns;
    uint64_t total_ns = host_monotonic_ns + offset_ns;

    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(total_ns / 1'000'000'000ULL);
    stamp.nanosec = static_cast<uint32_t>(total_ns % 1'000'000'000ULL);
    return stamp;
}

/// Create a bgr8 sensor_msgs::msg::Image from a BGR cv::Mat.
///
/// The pixel data is deep-copied, so the caller may release or reuse the
/// original cv::Mat immediately after the call returns.  Pixel order is
/// preserved byte-for-byte: B, G, R in memory becomes the same B, G, R
/// bytes in image.data.
inline auto make_bgr8_image(
    const cv::Mat& bgr,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id) -> sensor_msgs::msg::Image
{
    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = frame_id;
    image.height = static_cast<uint32_t>(bgr.rows);
    image.width = static_cast<uint32_t>(bgr.cols);
    image.encoding = "bgr8";
    image.is_bigendian = false;
    image.step = static_cast<uint32_t>(bgr.cols) * 3;  // 3 channels × 1 byte

    std::size_t total_bytes = bgr.total() * bgr.elemSize();
    image.data.assign(bgr.data, bgr.data + total_bytes);
    return image;
}

/// Fill BGR8 metadata on a pre-allocated sensor_msgs::msg::Image.
///
/// The caller is responsible for sizing image.data and copying pixel
/// bytes into it before calling this function.  This helper writes
/// header, dimensions, encoding, and step — everything except data.
/// Together with a single memcpy from SHM into image.data, this enables
/// a one-copy pipeline with no intermediate cv::Mat.
inline void fill_bgr8_image_metadata(
    sensor_msgs::msg::Image& image,
    int width, int height,
    const builtin_interfaces::msg::Time& stamp,
    const std::string& frame_id)
{
    image.header.stamp = stamp;
    image.header.frame_id = frame_id;
    image.height = static_cast<uint32_t>(height);
    image.width = static_cast<uint32_t>(width);
    image.encoding = "bgr8";
    image.is_bigendian = false;
    image.step = static_cast<uint32_t>(width) * 3;
}

// ════════════════════════════════════════════════════════════════════════
// Calibration validation (Fix L4-6: reject nonpositive cx/cy)
// ════════════════════════════════════════════════════════════════════════

inline auto is_finite_positive(double v) -> bool
{
    return std::isfinite(v) && v > 0.0;
}

inline auto validate_calibration(
    double fx, double fy, double cx, double cy,
    const std::vector<double>& distortion) -> std::optional<std::string>
{
    if (!is_finite_positive(fx))
        return "fx must be finite and positive, got " + std::to_string(fx);
    if (!is_finite_positive(fy))
        return "fy must be finite and positive, got " + std::to_string(fy);
    if (!is_finite_positive(cx))
        return "cx must be finite and positive, got " + std::to_string(cx);
    if (!is_finite_positive(cy))
        return "cy must be finite and positive, got " + std::to_string(cy);
    if (distortion.size() != 5)
        return "distortion must have exactly 5 coefficients (plumb_bob)";

    bool all_zero = true;
    for (auto d : distortion) {
        if (!std::isfinite(d))
            return "distortion coefficient is non-finite";
        if (d != 0.0) all_zero = false;
    }
    if (all_zero)
        return "distortion coefficients are all zero (placeholder)";
    return std::nullopt;
}

// ════════════════════════════════════════════════════════════════════════
// Image size validation (Fix M4-3: overflow-safe, MAX_IMAGE_SIZE check)
// ════════════════════════════════════════════════════════════════════════

inline auto validate_image_size(
    int width, int height, size_t max_image_bytes) -> std::optional<std::string>
{
    if (width <= 0)
        return "width must be positive, got " + std::to_string(width);
    if (height <= 0)
        return "height must be positive, got " + std::to_string(height);

    auto w = static_cast<size_t>(width);
    auto h = static_cast<size_t>(height);
    constexpr size_t kChannels = 3;

    size_t pixels = w * h;
    if (pixels / h != w)
        return "width * height overflow";

    size_t image_bytes = pixels * kChannels;
    if (image_bytes / kChannels != pixels)
        return "width * height * 3 overflow";

    if (image_bytes > max_image_bytes)
        return "BGR image size " + std::to_string(image_bytes)
             + " exceeds max " + std::to_string(max_image_bytes);

    return std::nullopt;
}

// ════════════════════════════════════════════════════════════════════════
// F2 race fix: completed-slot protocol using frame_counter
//
// The writer (SHMWrite) increments write_index BEFORE writing data but
// release-stores frame_counter AFTER data+timestamp are complete and
// before sem_post.  After sem_wait, frame_counter (acquire) reveals the
// fully-written completed slot = (counter - 1) % slot_num, eliminating
// the write_index TOCTOU race.
// ════════════════════════════════════════════════════════════════════════

inline auto completed_slot_from_counter(uint64_t frame_counter, unsigned int slot_num) -> unsigned int
{
    return static_cast<unsigned int>((frame_counter - 1) % static_cast<uint64_t>(slot_num));
}

inline auto is_valid_frame_counter(uint64_t frame_counter) -> bool
{
    return frame_counter > 0;
}

inline auto is_frame_stable(uint64_t before, uint64_t after) -> bool
{
    return before == after;
}

/// Decide whether frame_counter has advanced past last_seen and is
/// non-zero, meaning at least one new completed frame is available.
/// Used by the polling recorder to avoid consuming the single-consumer
/// SHM semaphore shared with other readers (e.g. radar_bridge).
inline auto should_process_counter(uint64_t current, uint64_t last_seen) -> bool
{
    return current > 0 && current != last_seen;
}

} // namespace radar::fast_livo2::rgb
