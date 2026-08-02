#include "radar_camera/armor_infer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <memory>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace radar_camera::armor_infer {

const char* kNames[] = { "hero-R", "eng-R", "inf3-R", "inf4-R", "sentry-R", "drone-R",
    "hero-B", "eng-B", "inf3-B", "inf4-B", "sentry-B", "drone-B" };
const char* kNumberNames[] = { "B1", "B2", "B3", "B4", "BS", "R1", "R2", "R3", "R4" };

auto sigmoid(float x) -> float { return 1.0f / (1.0f + std::exp(-x)); }

auto letterbox(const cv::Mat& src, int side, bool center, float& scale, int& px, int& py) -> cv::Mat {
    scale = std::min(static_cast<float>(side) / src.cols, static_cast<float>(side) / src.rows);
    const int w = std::max(1, static_cast<int>(std::round(src.cols * scale)));
    const int h = std::max(1, static_cast<int>(std::round(src.rows * scale)));
    px = center ? (side - w) / 2 : 0;
    py = center ? (side - h) / 2 : 0;
    cv::Mat out(side, side, src.type(), cv::Scalar::all(0));
    cv::Mat resized;
    cv::resize(src, resized, { w, h });
    resized.copyTo(out(cv::Rect(px, py, w, h)));
    return out;
}

auto blob(const cv::Mat& rgb) -> std::vector<float> {
    cv::Mat b = cv::dnn::blobFromImage(rgb, 1.0 / 255.0, {}, {}, false, false);
    std::vector<float> out(b.total());
    std::memcpy(out.data(), b.ptr<float>(), out.size() * sizeof(float));
    return out;
}

auto iou(const cv::Rect2f& a, const cv::Rect2f& b) -> float {
    const auto inter = a & b;
    const float union_area = a.area() + b.area() - inter.area();
    return union_area <= 0.0f ? 0.0f : inter.area() / union_area;
}

auto decode_l1(const std::vector<float>& raw, float scale, float l1_conf) -> std::vector<Det> {
    std::vector<Det> best;
    for (size_t i = 0; i + 5 < raw.size(); i += 6) {
        const float conf = raw[i + 4];
        const int id = static_cast<int>(raw[i + 5]);
        if (conf < l1_conf) continue;
        const float x1 = raw[i] / scale;
        const float y1 = raw[i + 1] / scale;
        const float x2 = raw[i + 2] / scale;
        const float y2 = raw[i + 3] / scale;
        cv::Rect2f box(x1, y1, x2 - x1, y2 - y1);
        if (box.width < 1 || box.height < 1) continue;
        const float ratio = std::max(box.width, box.height) / std::min(box.width, box.height);
        const bool drone = id == 5 || id == 11;
        if (ratio < (drone ? 2.0f : 0.5f) || ratio > (drone ? 10.0f : 3.0f)) continue;
        auto it = std::find_if(best.begin(), best.end(), [id](const Det& d) { return d.id == id; });
        Det candidate { id, conf, box };
        if (it == best.end()) best.push_back(candidate);
        else if (conf > it->conf) *it = candidate;
    }
    return best;
}

auto decode_l2(const std::vector<float>& raw, const cv::Rect2f& roi, float scale, int px, int py,
    float l2_conf, float l2_nms) -> std::optional<Plate> {
    std::vector<Plate> candidates;
    for (size_t i = 0; i + 21 < raw.size(); i += 22) {
        const float confidence = sigmoid(raw[i + 8]);
        if (confidence < l2_conf) continue;
        float min_x = raw[i], max_x = raw[i];
        float min_y = raw[i + 1], max_y = raw[i + 1];
        for (int p = 0; p < 4; ++p) {
            min_x = std::min(min_x, raw[i + p * 2]);
            max_x = std::max(max_x, raw[i + p * 2]);
            min_y = std::min(min_y, raw[i + p * 2 + 1]);
            max_y = std::max(max_y, raw[i + p * 2 + 1]);
        }
        cv::Rect2f box((min_x - px) / scale + roi.x, (min_y - py) / scale + roi.y,
            (max_x - min_x) / scale, (max_y - min_y) / scale);
        if (box.width < 1 || box.height < 1) continue;
        int color = static_cast<int>(std::max_element(raw.begin() + i + 9, raw.begin() + i + 13)
            - (raw.begin() + i + 9));
        int genre = static_cast<int>(std::max_element(raw.begin() + i + 13, raw.begin() + i + 22)
            - (raw.begin() + i + 13));
        std::vector<cv::Point2f> corners;
        for (int p = 0; p < 4; ++p) {
            corners.emplace_back((raw[i + p * 2] - px) / scale + roi.x,
                (raw[i + p * 2 + 1] - py) / scale + roi.y);
        }
        candidates.push_back({ box, corners, genre, color, confidence });
    }
    std::sort(candidates.begin(), candidates.end(),
        [](const Plate& a, const Plate& b) { return a.conf > b.conf; });
    for (size_t i = 0; i < candidates.size(); ++i) {
        bool suppressed = false;
        for (size_t j = 0; j < i; ++j) if (iou(candidates[i].box, candidates[j].box) > l2_nms) suppressed = true;
        if (!suppressed) return candidates[i];
    }
    return std::nullopt;
}

auto l3_id(int index) -> std::optional<int> {
    if (index >= 0 && index <= 4) return index + 6;
    if (index >= 5 && index <= 8) return index - 5;
    return std::nullopt;
}

auto l2_id(int genre, int color) -> std::optional<int> {
    const int ids[] = { -1, 0, 1, 2, 3, -1, 4 };
    if (genre < 0 || genre >= 7 || color == 0 || ids[genre] < 0) return std::nullopt;
    return color == 1 ? ids[genre] : ids[genre] + 6;
}

auto l1_names(int id) -> const char* {
    return (id >= 0 && id < 12) ? kNames[id] : "?";
}

auto l3_names(int index) -> const char* {
    return (index >= 0 && index < 9) ? kNumberNames[index] : "?";
}

namespace {

auto crop(const cv::Mat& frame, cv::Rect2f box) -> cv::Mat {
    int x = std::clamp(static_cast<int>(box.x), 0, frame.cols - 1);
    int y = std::clamp(static_cast<int>(box.y), 0, frame.rows - 1);
    int w = std::clamp(static_cast<int>(box.width), 1, frame.cols - x);
    int h = std::clamp(static_cast<int>(box.height), 1, frame.rows - y);
    return frame(cv::Rect(x, y, w, h));
}

auto run_l3(model_inference::TensorRtInference& engine, const cv::Mat& frame,
    cv::Rect2f box, float l3_conf) -> std::optional<Number> {
    float scale; int px; int py;
    auto input = letterbox(crop(frame, box), kSideL3, true, scale, px, py);
    auto data = blob(input);
    if (!engine.start(data.data(), data.size())) return std::nullopt;
    auto output = engine.wait();
    if (!output) return std::nullopt;
    const auto& raw = output->get();
    if (raw.size() < 9) return std::nullopt;
    const int index = static_cast<int>(std::max_element(raw.begin(), raw.begin() + 9) - raw.begin());
    const float confidence = raw[index];
    if (confidence < l3_conf) return std::nullopt;
    return Number { index, confidence };
}

auto run_l2(model_inference::TensorRtInference& engine, const cv::Mat& frame,
    cv::Rect2f roi, float l2_conf) -> std::optional<Plate> {
    float scale; int px; int py;
    const cv::Rect2f source_roi = roi;
    auto input = letterbox(crop(frame, roi), kSideL2, false, scale, px, py);
    auto data = blob(input);
    if (!engine.start(data.data(), data.size())) return std::nullopt;
    auto output = engine.wait();
    if (!output) return std::nullopt;
    return decode_l2(output->get(), source_roi, scale, px, py, l2_conf);
}

auto infer_l1(model_inference::TensorRtInference& engine, const cv::Mat& rgb) -> std::vector<Det> {
    float scale; int px; int py;
    auto input = letterbox(rgb, kSideL1, false, scale, px, py);
    auto data = blob(input);
    if (!engine.start(data.data(), data.size())) return {};
    auto output = engine.wait();
    if (!output) return {};
    return decode_l1(output->get(), scale);
}
} // namespace

class ArmorInferImpl {
public:
    model_inference::TensorRtInference l1_engine;
    model_inference::TensorRtInference l2_engine;
    model_inference::TensorRtInference l3_engine;
    float l1_conf { kL1Conf };
    float l2_conf { kL2Conf };
    float l3_conf { kL3Conf };
};

ArmorInfer::ArmorInfer(std::unique_ptr<ArmorInferImpl> impl) : impl_(std::move(impl)) {}

auto ArmorInfer::create(const std::string& model_dir,
    float l1_conf, float l2_conf, float l3_conf)
    -> std::expected<std::shared_ptr<ArmorInfer>, std::string> {
    auto impl = std::make_unique<ArmorInferImpl>();
    impl->l1_conf = l1_conf;
    impl->l2_conf = l2_conf;
    impl->l3_conf = l3_conf;
    const auto engine = [&](const char* name) {
        return (std::filesystem::path(model_dir) / name).string();
    };
    if (auto r = impl->l1_engine.init(engine("best_fixed_names_1280_fp16.engine")); !r)
        return std::unexpected("L1 engine init failed: " + r.error());
    if (auto r = impl->l2_engine.init(engine("shenzhen-0708_fp16.engine")); !r)
        return std::unexpected("L2 engine init failed: " + r.error());
    if (auto r = impl->l3_engine.init(engine("armor-number_fp16.engine")); !r)
        return std::unexpected("L3 engine init failed: " + r.error());
    return std::shared_ptr<ArmorInfer>(new ArmorInfer(std::move(impl)));
}

auto ArmorInfer::infer(const cv::Mat& frame_rgb) -> std::vector<ArmorResult> {
    std::vector<ArmorResult> results;
    for (const auto& detection : infer_l1(impl_->l1_engine, frame_rgb)) {
        ArmorResult result { detection.id, detection.conf, detection.box, std::nullopt,
            std::nullopt, detection.id, "L1", "MATCH" };
        if (detection.id != 5 && detection.id != 11) {
            auto plate = run_l2(impl_->l2_engine, frame_rgb, detection.box, impl_->l2_conf);
            if (plate) {
                result.l2 = plate;
                auto number = run_l3(impl_->l3_engine, frame_rgb, plate->box, impl_->l3_conf);
                if (number && l3_id(number->index)) {
                    result.l3 = number;
                    result.final_id = *l3_id(number->index);
                    result.decision = "L3-plate";
                } else if (l2_id(plate->genre, plate->color)) {
                    result.final_id = *l2_id(plate->genre, plate->color);
                    result.decision = "L2";
                }
            }
        }
        const bool l2_match = result.l2.has_value();
        const bool l3_match = result.l3.has_value();
        result.match_state = (l2_match || l3_match || detection.id == 5 || detection.id == 11)
            ? "MATCH" : "MISS";
        results.push_back(std::move(result));
    }
    return results;
}

} // namespace radar_camera::armor_infer
