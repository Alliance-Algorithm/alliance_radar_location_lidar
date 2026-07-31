#pragma once
#include <cstdint>
#include <expected>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include "radar_camera/data_format.hpp"
#include "radar_camera/tensorrt_inference.hpp"

namespace radar_camera::armor_refine {

// Armor plate color decoded from the L2 model / pixel BGR fallback.
enum class ArmorColor : std::uint8_t { UNKNOWN = 0, RED = 1, BLUE = 2 };

// L2 (shenzhen-0708) plate detector configuration.
struct ArmorRefineConfig {
    std::string armor_model_path; // shenzhen-0708_fp16.engine
    int model_input       = 640;  // square letterbox side
    float score_threshold = 0.8f; // plate confidence gate
    float nms_threshold   = 0.3f; // IoU NMS threshold
};

// L3 (armor-number) number classifier configuration.
struct NumberRefineConfig {
    std::string number_model_path; // armor-number_fp16.engine
    int model_input      = 224;    // square letterbox side
    float conf_threshold = 0.8f;   // number confidence gate
};

// One decoded L2 armor plate, coordinates mapped back to the source frame.
struct ArmorPlate {
    cv::Rect2f bbox;      // plate box in src-image pixels
    int genre        = 0; // L2 genre index (0=unk,1=hero,...)
    ArmorColor color = ArmorColor::UNKNOWN;
    float confidence = 0.0f;
};

// Refines L1 detections with L2 plate + L3 number cues.
//
// Fusion is pure priority: L3 (conf >= number.conf_threshold) overrides
// L2 (conf >= armor.score_threshold), which overrides the L1 class id.
// Position always stays from L1. Drone detections skip L2/L3 entirely.
class ArmorRefiner {
public:
    ArmorRefiner()  = default;
    ~ArmorRefiner() = default;

    auto init(const ArmorRefineConfig& armor_config, const NumberRefineConfig& number_config)
        -> std::expected<void, std::string>;

    // Refines det.id in place. drone_class_ids are skipped (id untouched).
    // orig_frame is the full-resolution RGB source frame (e.g. 5472×3648).
    // det.bbox is in L1 model-input space; scale_x/scale_y map it to orig_frame.
    auto refine(const cv::Mat& orig_frame, detection::Detection& det,
        const std::vector<std::int64_t>& drone_class_ids,
        float scale_x, float scale_y) -> void;

    // Maps an L2 (genre, color) pair to an absolute L1 class id (0-11).
    // Returns nullopt for unmappable genres/colors.
    static auto l2_to_class_id(int genre, ArmorColor color) -> std::optional<int>;

    // Maps an L3 class index (0-8: B1,B2,B3,B4,BS,R1,R2,R3,R4) to a class id.
    static auto l3_idx_to_class_id(int idx) -> std::optional<int>;

private:
    // Runs L2 on the robot ROI, returns the best plate above score_threshold.
    auto run_l2(const cv::Mat& frame, const cv::Rect2f& roi) -> std::optional<ArmorPlate>;

    // Runs L3 on the plate crop, returns (class_idx, confidence) above threshold.
    auto run_l3(const cv::Mat& frame, const cv::Rect2f& plate)
        -> std::optional<std::pair<int, float>>;

    // Mean-BGR channel-difference color fallback over the given ROI.
    auto pixel_color(const cv::Mat& frame, const cv::Rect2f& roi) -> ArmorColor;

    ArmorRefineConfig armor_config_;
    NumberRefineConfig number_config_;
    bool initialized_ = false;

    model_inference::TensorRtInference l2_trt_;
    model_inference::TensorRtInference l3_trt_;
};

} // namespace radar_camera::armor_refine
