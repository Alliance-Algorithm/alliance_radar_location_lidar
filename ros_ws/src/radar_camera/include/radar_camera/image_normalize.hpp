#pragma once

#include <cstddef>
#include <cstdint>

/// CUDA kernels for camera preprocessing (host-side declarations).
///
/// The implementations live in image_normalize.cu and are compiled only when
/// the TensorRT backend is enabled (RADAR_CAMERA_ENABLE_TENSORRT).
namespace radar_camera::model_inference {

/// Normalize one RGB u8 HWC image into a planar f32 CHW tensor (each channel
/// divided by 255), matching cv::dnn::blobFromImage(scale=1/255, swapRB=false,
/// crop=false). `stream` is the raw cudaStream_t to launch on; it is passed as
/// void* so this header stays CUDA-free.
void normalize_rgb_u8_to_planar_f32(const std::uint8_t* src, float* dst, int width, int height,
    void* stream);

} // namespace radar_camera::model_inference
