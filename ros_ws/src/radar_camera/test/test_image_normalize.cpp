#include <cmath>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

#include "radar_camera/image_normalize.hpp"

#ifdef RADAR_CAMERA_HAS_TENSORRT
#include <cuda_runtime_api.h>

namespace {

TEST(ImageNormalize, MatchesBlobFromImageOnGpu) {
    int device_count = 0;
    ASSERT_EQ(cudaGetDeviceCount(&device_count), cudaSuccess);
    if (device_count == 0) {
        GTEST_SKIP() << "no CUDA device";
    }

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, 255);
    constexpr int width  = 37;
    constexpr int height = 23;
    cv::Mat image(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            cv::Vec3b& px = image.at<cv::Vec3b>(y, x);
            px[0]         = static_cast<uchar>(dist(rng));
            px[1]         = static_cast<uchar>(dist(rng));
            px[2]         = static_cast<uchar>(dist(rng));
        }
    }

    cv::Mat blob =
        cv::dnn::blobFromImage(image, 1.0 / 255.0, cv::Size(), cv::Scalar(), false, false);
    const size_t elements = blob.total();

    void* device_src = nullptr;
    void* device_dst = nullptr;
    ASSERT_EQ(cudaMalloc(&device_src, static_cast<size_t>(width) * height * 3), cudaSuccess);
    ASSERT_EQ(cudaMalloc(&device_dst, elements * sizeof(float)), cudaSuccess);
    ASSERT_EQ(cudaMemcpy(device_src, image.ptr(), static_cast<size_t>(width) * height * 3,
                  cudaMemcpyHostToDevice),
        cudaSuccess);

    void* stream = nullptr;
    ASSERT_EQ(cudaStreamCreate(reinterpret_cast<cudaStream_t*>(&stream)), cudaSuccess);
    radar_camera::model_inference::normalize_rgb_u8_to_planar_f32(
        static_cast<const std::uint8_t*>(device_src), static_cast<float*>(device_dst), width,
        height, stream);
    ASSERT_EQ(cudaStreamSynchronize(reinterpret_cast<cudaStream_t>(stream)), cudaSuccess);

    std::vector<float> actual(elements);
    ASSERT_EQ(
        cudaMemcpy(actual.data(), device_dst, elements * sizeof(float), cudaMemcpyDeviceToHost),
        cudaSuccess);

    cudaStreamDestroy(reinterpret_cast<cudaStream_t>(stream));
    cudaFree(device_src);
    cudaFree(device_dst);

    for (size_t i = 0; i < elements; ++i) {
        EXPECT_NEAR(actual[i], blob.ptr<float>()[i], 1e-6f)
            << "channel/plane element " << i << " differs";
    }
}

} // namespace

#endif // RADAR_CAMERA_HAS_TENSORRT
