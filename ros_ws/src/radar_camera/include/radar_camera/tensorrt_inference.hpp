#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace radar_camera::model_inference {

/// TensorRT wrapper with double-buffered pipeline support.
///
/// start()/start_u8() enqueue on one slot, wait() returns the *previous*
/// enqueued slot's output (FIFO). With two slots the CPU can prepare frame
/// N+1 while the GPU still runs frame N. Serial start→wait callers (L2/L3)
/// work unchanged: start(A), wait(A), start(B), wait(B), ...
class TensorRtInference final {
public:
    TensorRtInference();
    ~TensorRtInference();

    TensorRtInference(const TensorRtInference&)            = delete;
    TensorRtInference& operator=(const TensorRtInference&) = delete;
    TensorRtInference(TensorRtInference&&)                 = delete;
    TensorRtInference& operator=(TensorRtInference&&)      = delete;

    auto init(const std::string& engine_path) -> std::expected<void, std::string>;
    auto start(const float* input, std::size_t input_elements) -> std::expected<void, std::string>;
    /// Enqueue from an RGB u8 HWC image: uploads u8 (4x less PCIe traffic than
    /// float) and normalizes to planar f32/255 on the GPU.
    auto start_u8(const std::uint8_t* rgb, int width, int height)
        -> std::expected<void, std::string>;
    /// Wait for the oldest pending enqueue and return its output.
    auto wait() -> std::expected<std::reference_wrapper<const std::vector<float>>, std::string>;

    [[nodiscard]] auto input_elements() const noexcept -> std::size_t;
    [[nodiscard]] auto output_elements() const noexcept -> std::size_t;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace radar_camera::model_inference
