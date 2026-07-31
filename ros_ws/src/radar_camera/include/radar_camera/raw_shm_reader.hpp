#pragma once

#include <cstdint>

#include <opencv2/core/mat.hpp>

namespace radar_camera::recording {

struct RawFrame {
    cv::Mat rgb;
    std::uint64_t sequence;
    std::uint64_t host_monotonic_ns;
};

} // namespace radar_camera::recording
