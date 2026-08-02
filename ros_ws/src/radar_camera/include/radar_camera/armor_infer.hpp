#pragma once

#include <cstddef>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "radar_camera/tensorrt_inference.hpp"

namespace radar_camera::armor_infer {

inline constexpr float kL1Conf = 0.20f;
inline constexpr float kL2Conf = 0.50f;
inline constexpr float kL3Conf = 0.80f;
inline constexpr float kL2Nms  = 0.30f;
inline constexpr int kSideL1   = 1280;
inline constexpr int kSideL2   = 640;
inline constexpr int kSideL3   = 224;

struct Det {
    int id;
    float conf;
    cv::Rect2f box;
};
struct Plate {
    cv::Rect2f box;
    std::vector<cv::Point2f> corners;
    int genre;
    int color;
    float conf;
};
struct Number {
    int index;
    float conf;
};

struct ArmorResult {
    int l1_id;
    float l1_conf;
    cv::Rect2f l1_box;
    std::optional<Plate> l2;
    std::optional<Number> l3;
    int final_id;
    std::string decision;    // "L1" | "L2" | "L3-plate"
    std::string match_state; // "MATCH" | "MISS"
};

auto sigmoid(float x) -> float;
auto letterbox(const cv::Mat& src, int side, bool center, float& scale, int& px, int& py)
    -> cv::Mat;
auto blob(const cv::Mat& rgb) -> std::vector<float>;
auto iou(const cv::Rect2f& a, const cv::Rect2f& b) -> float;
auto decode_l1(const std::vector<float>& raw, float scale, float l1_conf = kL1Conf)
    -> std::vector<Det>;
auto decode_l2(const std::vector<float>& raw, const cv::Rect2f& roi, float scale, int px, int py,
    float l2_conf = kL2Conf, float l2_nms = kL2Nms) -> std::optional<Plate>;
auto l2_id(int genre, int color) -> std::optional<int>;
auto l3_id(int index) -> std::optional<int>;
auto l1_names(int id) -> const char*;
auto l3_names(int index) -> const char*;

class ArmorInfer final {
public:
    ArmorInfer()  = default;
    ~ArmorInfer() = default;

    ArmorInfer(const ArmorInfer&)                = delete;
    ArmorInfer& operator=(const ArmorInfer&)     = delete;
    ArmorInfer(ArmorInfer&&) noexcept            = default;
    ArmorInfer& operator=(ArmorInfer&&) noexcept = default;

    static auto create(const std::string& model_dir, float l1_conf = kL1Conf,
        float l2_conf = kL2Conf, float l3_conf = kL3Conf)
        -> std::expected<std::shared_ptr<ArmorInfer>, std::string>;

    auto infer(const cv::Mat& frame_rgb) -> std::vector<ArmorResult>;

private:
    ArmorInfer(std::unique_ptr<class ArmorInferImpl> impl);
    std::unique_ptr<class ArmorInferImpl> impl_;
};

} // namespace radar_camera::armor_infer
