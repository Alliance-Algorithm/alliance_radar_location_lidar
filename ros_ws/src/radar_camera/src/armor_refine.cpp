#include "radar_camera/armor_refine.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace radar_camera::armor_refine {

namespace {

    constexpr int kL2Stride  = 22; // 8 corner coords + conf + 4 color + 9 genre
    constexpr int kColorBase = 9;  // cols 9-12: color logits
    constexpr int kGenreBase = 13; // cols 13-21: genre logits
    constexpr int kNumColor  = 4;
    constexpr int kNumGenre  = 9;
    constexpr int kL3Classes = 9; // B1,B2,B3,B4,BS,R1,R2,R3,R4

    auto sigmoid(float x) -> float { return 1.0f / (1.0f + std::exp(-x)); }

    // Index of the maximum element in [begin, begin + count).
    auto argmax(const float* begin, int count) -> int {
        int best       = 0;
        float best_val = begin[0];
        for (int i = 1; i < count; ++i) {
            if (begin[i] > best_val) {
                best_val = begin[i];
                best     = i;
            }
        }
        return best;
    }

    // Clamps a float rect to the image bounds and returns an integer ROI.
    auto clamp_roi(const cv::Rect2f& box, int width, int height) -> cv::Rect {
        int x = std::clamp(static_cast<int>(std::floor(box.x)), 0, width - 1);
        int y = std::clamp(static_cast<int>(std::floor(box.y)), 0, height - 1);
        int w = std::clamp(static_cast<int>(std::ceil(box.width)), 1, width - x);
        int h = std::clamp(static_cast<int>(std::ceil(box.height)), 1, height - y);
        return cv::Rect(x, y, w, h);
    }

    // Letterbox to a square canvas; anchor selects top-left (L2) vs centered (L3).
    // Reports scale and pad so callers can invert the mapping.
    enum class LetterboxAnchor : std::uint8_t { TOP_LEFT, CENTER };

    auto letterbox(const cv::Mat& src, int side, LetterboxAnchor anchor, float& scale, int& pad_x,
        int& pad_y) -> cv::Mat {
        scale     = std::min(static_cast<float>(side) / static_cast<float>(src.cols),
            static_cast<float>(side) / static_cast<float>(src.rows));
        int new_w = std::max(1, static_cast<int>(std::round(src.cols * scale)));
        int new_h = std::max(1, static_cast<int>(std::round(src.rows * scale)));
        cv::Mat resized;
        cv::resize(src, resized, cv::Size(new_w, new_h));
        cv::Mat canvas(side, side, src.type(), cv::Scalar(0, 0, 0));
        if (anchor == LetterboxAnchor::CENTER) {
            pad_x = (side - new_w) / 2;
            pad_y = (side - new_h) / 2;
        } else {
            pad_x = 0;
            pad_y = 0;
        }
        resized.copyTo(canvas(cv::Rect(pad_x, pad_y, new_w, new_h)));
        return canvas;
    }

    // Enqueues an RGB u8 canvas on the given engine: GPU 归一化 + 4x 更少 PCIe。
    // 返回后需立即 wait()（refine 串行语义）。
    auto run_trt_u8(radar_camera::model_inference::TensorRtInference& engine,
        const cv::Mat& canvas)
        -> std::expected<std::reference_wrapper<const std::vector<float>>, std::string> {
        if (canvas.empty() || canvas.type() != CV_8UC3) {
            return std::unexpected("refine canvas is not RGB u8");
        }
        if (auto r = engine.start_u8(canvas.ptr<std::uint8_t>(), canvas.cols, canvas.rows); !r) {
            return std::unexpected(r.error());
        }
        return engine.wait();
    }

    auto iou(const cv::Rect2f& a, const cv::Rect2f& b) -> float {
        const float inter = (a & b).area();
        const float uni   = a.area() + b.area() - inter;
        return uni > 0.0f ? inter / uni : 0.0f;
    }

} // namespace

auto ArmorRefiner::init(const ArmorRefineConfig& armor_config,
    const NumberRefineConfig& number_config) -> std::expected<void, std::string> {
    armor_config_  = armor_config;
    number_config_ = number_config;

    if (auto r = l2_trt_.init(armor_config_.armor_model_path); !r)
        return std::unexpected("ArmorRefiner L2 init: " + r.error());
    if (auto r = l3_trt_.init(number_config_.number_model_path); !r)
        return std::unexpected("ArmorRefiner L3 init: " + r.error());

    initialized_ = true;
    return { };
}

auto ArmorRefiner::refine(const cv::Mat& orig_frame, detection::Detection& det,
    const std::vector<std::int64_t>& drone_class_ids, float scale_x, float scale_y) -> void {
    if (!initialized_ || orig_frame.empty() || det.bbox.area() <= 0.0f) return;

    const bool is_drone =
        std::find(drone_class_ids.begin(), drone_class_ids.end(), static_cast<std::int64_t>(det.id))
        != drone_class_ids.end();
    if (is_drone) return; // drones have no armor plate; keep L1 id

    // Map det.bbox from L1 model-input space to orig_frame space.
    const cv::Rect2f orig_bbox(det.bbox.x * scale_x, det.bbox.y * scale_y, det.bbox.width * scale_x,
        det.bbox.height * scale_y);

    // Priority 1: L3 number classifier on the L1 ROI (in orig_frame).
    if (auto number = run_l3(orig_frame, orig_bbox)) {
        if (auto id = l3_idx_to_class_id(number->first)) {
            det.id = *id;
            return;
        }
    }

    // Priority 2: L2 plate detector (genre + color) on the L1 ROI (in orig_frame).
    if (auto plate = run_l2(orig_frame, orig_bbox)) {
        // Re-run L3 on the tighter plate crop (already in orig_frame space).
        if (auto number = run_l3(orig_frame, plate->bbox)) {
            if (auto id = l3_idx_to_class_id(number->first)) {
                det.id = *id;
                return;
            }
        }
        ArmorColor color = plate->color;
        if (color == ArmorColor::UNKNOWN) color = pixel_color(orig_frame, plate->bbox);
        if (auto id = l2_to_class_id(plate->genre, color)) {
            det.id = *id;
            return;
        }
    }

    // Priority 3: keep the L1 id unchanged.
}

auto ArmorRefiner::run_l2(const cv::Mat& frame, const cv::Rect2f& roi)
    -> std::optional<ArmorPlate> {
    const cv::Rect crop_rect = clamp_roi(roi, frame.cols, frame.rows);
    const cv::Mat crop       = frame(crop_rect);
    if (crop.empty()) return std::nullopt;

    float scale    = 1.0f;
    int pad_x      = 0;
    int pad_y      = 0;
    const int side = armor_config_.model_input;
    cv::Mat input  = letterbox(crop, side, LetterboxAnchor::TOP_LEFT, scale, pad_x, pad_y);

    auto wait_result = run_trt_u8(l2_trt_, input);
    if (!wait_result) return std::nullopt;
    const std::vector<float>& flat = wait_result->get();

    // Output is [1,25200,22] stored row-major: num = output_elements / kL2Stride.
    const int num = static_cast<int>(flat.size()) / kL2Stride;
    if (num <= 0) return std::nullopt;
    const float* data = flat.data();

    struct Candidate {
        cv::Rect2f box;
        int genre;
        ArmorColor color;
        float conf;
    };
    // Post-threshold survivors are typically a handful; reserving all `num`
    // grid cells (~25200) would allocate hundreds of KB on every call.
    std::vector<Candidate> candidates;

    for (int i = 0; i < num; ++i) {
        const float* row = data + static_cast<size_t>(i) * static_cast<size_t>(kL2Stride);
        const float conf = sigmoid(row[8]);
        if (conf < armor_config_.score_threshold) continue;

        // Corner coords (cols 0-7) -> axis-aligned box in letterbox space.
        float min_x = row[0];
        float max_x = row[0];
        float min_y = row[1];
        float max_y = row[1];
        for (int c = 0; c < 4; ++c) {
            min_x = std::min(min_x, row[c * 2 + 0]);
            max_x = std::max(max_x, row[c * 2 + 0]);
            min_y = std::min(min_y, row[c * 2 + 1]);
            max_y = std::max(max_y, row[c * 2 + 1]);
        }
        // Invert letterbox -> crop space -> source frame.
        const float bx =
            (min_x - static_cast<float>(pad_x)) / scale + static_cast<float>(crop_rect.x);
        const float by =
            (min_y - static_cast<float>(pad_y)) / scale + static_cast<float>(crop_rect.y);
        const float bw = (max_x - min_x) / scale;
        const float bh = (max_y - min_y) / scale;
        if (bw < 1.0f || bh < 1.0f) continue;

        const int color_idx = argmax(row + kColorBase, kNumColor);
        const int genre_idx = argmax(row + kGenreBase, kNumGenre);
        ArmorColor color    = ArmorColor::UNKNOWN;
        if (color_idx == 1) color = ArmorColor::RED;
        else if (color_idx == 2) color = ArmorColor::BLUE;

        candidates.push_back({ cv::Rect2f(bx, by, bw, bh), genre_idx, color, conf });
    }

    if (candidates.empty()) return std::nullopt;

    std::sort(candidates.begin(), candidates.end(),
        [](const Candidate& a, const Candidate& b) { return a.conf > b.conf; });

    // Greedy NMS; keep the single best surviving plate.
    std::vector<bool> suppressed(candidates.size(), false);
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (suppressed[i]) continue;
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (suppressed[j]) continue;
            if (iou(candidates[i].box, candidates[j].box) > armor_config_.nms_threshold) {
                suppressed[j] = true;
            }
        }
    }

    for (size_t i = 0; i < candidates.size(); ++i) {
        if (suppressed[i]) continue;
        return ArmorPlate { .bbox = candidates[i].box,
            .genre                = candidates[i].genre,
            .color                = candidates[i].color,
            .confidence           = candidates[i].conf };
    }
    return std::nullopt;
}

auto ArmorRefiner::run_l3(const cv::Mat& frame, const cv::Rect2f& plate)
    -> std::optional<std::pair<int, float>> {
    const cv::Rect crop_rect = clamp_roi(plate, frame.cols, frame.rows);
    const cv::Mat crop       = frame(crop_rect);
    if (crop.empty()) return std::nullopt;

    float scale    = 1.0f;
    int pad_x      = 0;
    int pad_y      = 0;
    const int side = number_config_.model_input;
    cv::Mat input  = letterbox(crop, side, LetterboxAnchor::CENTER, scale, pad_x, pad_y);

    auto wait_result = run_trt_u8(l3_trt_, input);
    if (!wait_result) return std::nullopt;
    const std::vector<float>& flat = wait_result->get();
    if (flat.size() < static_cast<size_t>(kL3Classes)) return std::nullopt;
    const float* logits = flat.data();

    // Model output is already softmax probabilities — use directly.
    const int idx = argmax(logits, kL3Classes);
    const float p = logits[idx];
    if (p < number_config_.conf_threshold) return std::nullopt;
    return std::make_pair(idx, p);
}

auto ArmorRefiner::pixel_color(const cv::Mat& frame, const cv::Rect2f& roi) -> ArmorColor {
    const cv::Rect crop_rect = clamp_roi(roi, frame.cols, frame.rows);
    const cv::Mat crop       = frame(crop_rect);
    if (crop.empty()) return ArmorColor::UNKNOWN;
    // frame is RGB8 (SHM layout): channel 0 = R, channel 2 = B.
    const cv::Scalar mean    = cv::mean(crop);
    const double r           = mean[0];
    const double b           = mean[2];
    constexpr double kMargin = 20.0;
    if (r - b > kMargin) return ArmorColor::RED;
    if (b - r > kMargin) return ArmorColor::BLUE;
    return ArmorColor::UNKNOWN;
}

auto ArmorRefiner::l2_to_class_id(int genre, ArmorColor color) -> std::optional<int> {
    if (color == ArmorColor::UNKNOWN) return std::nullopt;
    // L2 genre remap: 1=hero,2=eng,3=inf3,4=inf4,6=sentry (others unmappable).
    // L1 class ids: RED 0=hero,1=eng,2=inf3,3=inf4,4=sentry; BLUE = RED + 6.
    int red_id = -1;
    switch (genre) {
    case 1:
        red_id = 0;
        break; // hero
    case 2:
        red_id = 1;
        break; // engineer
    case 3:
        red_id = 2;
        break; // infantry3
    case 4:
        red_id = 3;
        break; // infantry4
    case 6:
        red_id = 4;
        break; // sentry
    default:
        return std::nullopt;
    }
    return color == ArmorColor::RED ? red_id : red_id + 6;
}

auto ArmorRefiner::l3_idx_to_class_id(int idx) -> std::optional<int> {
    // L3 classes: 0=B1,1=B2,2=B3,3=B4,4=BS,5=R1,6=R2,7=R3,8=R4.
    // L1 class ids: RED 0=hero..4=sentry; BLUE 6=hero..10=sentry.
    switch (idx) {
    case 0:
        return 6; // B1 blue hero
    case 1:
        return 7; // B2 blue engineer
    case 2:
        return 8; // B3 blue infantry3
    case 3:
        return 9; // B4 blue infantry4
    case 4:
        return 10; // BS blue sentry
    case 5:
        return 0; // R1 red hero
    case 6:
        return 1; // R2 red engineer
    case 7:
        return 2; // R3 red infantry3
    case 8:
        return 3; // R4 red infantry4
    default:
        return std::nullopt;
    }
}

} // namespace radar_camera::armor_refine
