#include "radar_lidar/frame_accumulator.hpp"

namespace radar {

void FrameAccumulator::push(types::PointCloud points) {
    frames_.push_back(std::move(points));
    while (frames_.size() > window_size_) {
        frames_.pop_front();
    }
}

auto FrameAccumulator::all_points() const -> types::PointCloud {
    types::PointCloud result;
    size_t total = 0;
    for (const auto& frame : frames_) {
        total += frame.size();
    }
    result.reserve(total);
    for (const auto& frame : frames_) {
        result.insert(result.end(), frame.begin(), frame.end());
    }
    return result;
}

} // namespace radar
