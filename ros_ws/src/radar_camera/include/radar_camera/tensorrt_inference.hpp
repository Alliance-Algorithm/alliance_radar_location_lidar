#pragma once

#include <cstddef>
#include <expected>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace radar_camera::model_inference {

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
    auto wait() -> std::expected<std::reference_wrapper<const std::vector<float>>, std::string>;

    [[nodiscard]] auto input_elements() const noexcept -> std::size_t;
    [[nodiscard]] auto output_elements() const noexcept -> std::size_t;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace radar_camera::model_inference
