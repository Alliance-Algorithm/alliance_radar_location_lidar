#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "radar_camera/tensorrt_inference.hpp"

namespace fs = std::filesystem;
using radar_camera::model_inference::TensorRtInference;

namespace {
constexpr int kL1Side = 1280;
constexpr int kL3Side = 224;
constexpr float kL1Confidence = 0.30f;
constexpr float kL3Confidence = 0.80f;
const char* l1_names[] = { "hero-B", "eng-B", "inf3-B", "inf4-B", "sentry-B", "drone-B",
    "hero-R", "eng-R", "inf3-R", "inf4-R", "sentry-R", "drone-R" };
const char* l3_names[] = { "B1", "B2", "B3", "B4", "BS", "R1", "R2", "R3", "R4" };

struct Detection { int id; float confidence; cv::Rect2f box; };
struct Number { int index; float confidence; };

auto crop(const cv::Mat& image, cv::Rect2f box) -> cv::Mat {
    const int x = std::clamp(static_cast<int>(box.x), 0, image.cols - 1);
    const int y = std::clamp(static_cast<int>(box.y), 0, image.rows - 1);
    const int w = std::clamp(static_cast<int>(box.width), 1, image.cols - x);
    const int h = std::clamp(static_cast<int>(box.height), 1, image.rows - y);
    return image(cv::Rect(x, y, w, h));
}

auto infer(TensorRtInference& engine, const cv::Mat& input) -> std::vector<float> {
    cv::Mat blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, {}, {}, false, false);
    std::vector<float> data(blob.total());
    std::memcpy(data.data(), blob.ptr<float>(), data.size() * sizeof(float));
    if (!engine.start(data.data(), data.size())) return {};
    const auto output = engine.wait();
    return output ? output->get() : std::vector<float> {};
}

auto l1_detect(TensorRtInference& engine, const cv::Mat& rgb) -> std::vector<Detection> {
    const float scale = std::min(static_cast<float>(kL1Side) / rgb.cols,
        static_cast<float>(kL1Side) / rgb.rows);
    const int width = std::max(1, static_cast<int>(std::round(rgb.cols * scale)));
    const int height = std::max(1, static_cast<int>(std::round(rgb.rows * scale)));
    cv::Mat input(kL1Side, kL1Side, rgb.type(), cv::Scalar::all(0));
    cv::Mat resized;
    cv::resize(rgb, resized, { width, height });
    resized.copyTo(input(cv::Rect(0, 0, width, height)));
    const auto raw = infer(engine, input);
    std::vector<Detection> best;
    for (size_t i = 0; i + 5 < raw.size(); i += 6) {
        const float confidence = raw[i + 4];
        const int id = static_cast<int>(raw[i + 5]);
        if (confidence < kL1Confidence) continue;
        cv::Rect2f box(raw[i] / scale, raw[i + 1] / scale,
            (raw[i + 2] - raw[i]) / scale, (raw[i + 3] - raw[i + 1]) / scale);
        if (box.width < 1.0f || box.height < 1.0f) continue;
        const float ratio = std::max(box.width, box.height) / std::min(box.width, box.height);
        const bool drone = id == 5 || id == 11;
        if (ratio < (drone ? 2.0f : 0.5f) || ratio > (drone ? 10.0f : 3.0f)) continue;
        const Detection candidate { id, confidence, box };
        auto current = std::find_if(best.begin(), best.end(),
            [id](const Detection& item) { return item.id == id; });
        if (current == best.end()) best.push_back(candidate);
        else if (confidence > current->confidence) *current = candidate;
    }
    return best;
}

auto run_l3(TensorRtInference& engine, const cv::Mat& rgb, cv::Rect2f box) -> std::optional<Number> {
    cv::Mat roi = crop(rgb, box);
    const float scale = std::min(static_cast<float>(kL3Side) / roi.cols,
        static_cast<float>(kL3Side) / roi.rows);
    const int width = std::max(1, static_cast<int>(std::round(roi.cols * scale)));
    const int height = std::max(1, static_cast<int>(std::round(roi.rows * scale)));
    const int px = (kL3Side - width) / 2;
    const int py = (kL3Side - height) / 2;
    cv::Mat input(kL3Side, kL3Side, roi.type(), cv::Scalar::all(0));
    cv::Mat resized;
    cv::resize(roi, resized, { width, height });
    resized.copyTo(input(cv::Rect(px, py, width, height)));
    const auto raw = infer(engine, input);
    if (raw.size() < 9) return std::nullopt;
    // Model output is already softmax probabilities — use directly.
    const int index = static_cast<int>(std::max_element(raw.begin(), raw.begin() + 9) - raw.begin());
    const float confidence = raw[index];
    if (confidence < kL3Confidence) return std::nullopt;
    return Number { index, confidence };
}

void label(cv::Mat& image, const std::string& value, cv::Point point, cv::Scalar color, double scale = 0.9) {
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, cv::Scalar::all(0), 5, cv::LINE_AA);
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, scale, color, 2, cv::LINE_AA);
}

auto main_impl(int argc, char** argv) -> int {
    if (argc < 4) {
        std::cerr << "usage: annotate_l3 <frames> <output> <model_dir>\n";
        return 2;
    }
    const fs::path frames = argv[1], output = argv[2], models = argv[3];
    fs::create_directories(output);
    TensorRtInference l1_engine, l3_engine;
    if (!l1_engine.init((models / "best_fixed_names_1280_fp16.engine").string())) return 1;
    if (!l3_engine.init((models / "armor-number_fp16.engine").string())) return 1;
    std::ofstream csv(output / "results.csv");
    csv << "frame,det_idx,l1_class,l1_confidence,l1_x,l1_y,l1_w,l1_h,l3_index,l3_name,l3_confidence,state\n";
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(frames)) {
        if (entry.path().extension() == ".jpg") files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    const cv::Scalar l1_color(255, 180, 0), l3_color(0, 220, 0), miss_color(0, 0, 255);
    for (const auto& file : files) {
        cv::Mat bgr = cv::imread(file.string());
        if (bgr.empty()) continue;
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        constexpr int panel_width = 900;
        cv::Mat annotated(bgr.rows, bgr.cols + panel_width, CV_8UC3, cv::Scalar(28, 28, 28));
        bgr.copyTo(annotated(cv::Rect(0, 0, bgr.cols, bgr.rows)));
        label(annotated, "L3 number on L1 ROI  conf>=0.80", { 25, 58 }, { 255, 255, 255 }, 1.0);
        const auto detections = l1_detect(l1_engine, rgb);
        int index = 0;
        for (const auto& detection : detections) {
            const bool drone = detection.id == 5 || detection.id == 11;
            const auto number = drone ? std::optional<Number> {} : run_l3(l3_engine, rgb, detection.box);
            const std::string state = number ? "MATCH" : "MISS";
            const auto color = number ? l3_color : miss_color;
            cv::rectangle(annotated, { static_cast<int>(detection.box.x), static_cast<int>(detection.box.y) },
                { static_cast<int>(detection.box.x + detection.box.width), static_cast<int>(detection.box.y + detection.box.height) },
                l1_color, 8);
            label(annotated, "L1 " + std::string(l1_names[detection.id]) + " "
                    + std::to_string(static_cast<int>(detection.confidence * 100.0f)) + "%",
                { static_cast<int>(detection.box.x), std::max(120, static_cast<int>(detection.box.y) - 58) }, l1_color);
            if (number) {
                label(annotated, "L3 " + std::string(l3_names[number->index]) + " "
                        + std::to_string(static_cast<int>(number->confidence * 100.0f)) + "% " + state,
                    { static_cast<int>(detection.box.x), std::max(120, static_cast<int>(detection.box.y) - 18) }, color);
            } else {
                label(annotated, drone ? "L3 SKIP (DRONE)" : "L3 MISS", { static_cast<int>(detection.box.x),
                    std::max(120, static_cast<int>(detection.box.y) - 18) }, color);
            }
            const int card_y = 120 + index * 560;
            if (card_y + 500 < annotated.rows) {
                const int panel_x = bgr.cols + 30;
                label(annotated, "L1 ROI / L3 INPUT", { panel_x, card_y }, { 255, 255, 255 }, 0.8);
                cv::Mat preview;
                cv::resize(crop(bgr, detection.box), preview, { panel_width - 60, 400 });
                preview.copyTo(annotated(cv::Rect(panel_x, card_y + 20, preview.cols, preview.rows)));
                label(annotated, number ? "L3 " + std::string(l3_names[number->index]) : drone ? "L3 SKIP" : "L3 MISS",
                    { panel_x, card_y + 455 }, color, 0.8);
            }
            csv << file.filename().string() << "," << index << "," << detection.id << ","
                << detection.confidence << "," << detection.box.x << "," << detection.box.y << ","
                << detection.box.width << "," << detection.box.height << ","
                << (number ? number->index : -1) << "," << (number ? l3_names[number->index] : "") << ","
                << (number ? number->confidence : 0.0f) << "," << state << "\n";
            ++index;
        }
        cv::imwrite((output / (file.stem().string() + "_L3.jpg")).string(), annotated);
    }
    return 0;
}
} // namespace

auto main(int argc, char** argv) -> int { return main_impl(argc, argv); }
