#include "radar_camera/image_normalize.hpp"

#include <cuda_runtime_api.h>

namespace radar_camera::model_inference {
namespace {

__global__ void normalize_rgb_u8_to_planar_f32_kernel(
    const std::uint8_t* __restrict__ src, float* __restrict__ dst, int pixels) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= pixels) return;
    const float r = src[i * 3 + 0] * (1.0f / 255.0f);
    const float g = src[i * 3 + 1] * (1.0f / 255.0f);
    const float b = src[i * 3 + 2] * (1.0f / 255.0f);
    dst[i]              = r;
    dst[pixels + i]     = g;
    dst[2 * pixels + i] = b;
}

} // namespace

void normalize_rgb_u8_to_planar_f32(
    const std::uint8_t* src, float* dst, int width, int height, void* stream) {
    constexpr int threads = 256;
    const int pixels      = width * height;
    const int blocks      = (pixels + threads - 1) / threads;
    normalize_rgb_u8_to_planar_f32_kernel<<<blocks, threads, 0, static_cast<cudaStream_t>(stream)>>>(
        src, dst, pixels);
}

} // namespace radar_camera::model_inference
