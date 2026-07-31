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
constexpr float kConfidence = 0.30f;
const char* kNames[] = { "hero-B", "eng-B", "inf3-B", "inf4-B", "sentry-B", "drone-B",
    "hero-R", "eng-R", "inf3-R", "inf4-R", "sentry-R", "drone-R" };

struct Detection { int id; float confidence; cv::Rect2f box; };

auto letterbox(const cv::Mat& src, float& scale) -> cv::Mat {
    constexpr int side = 1280;
    scale = std::min(static_cast<float>(side) / src.cols, static_cast<float>(side) / src.rows);
    const int width = std::max(1, static_cast<int>(std::round(src.cols * scale)));
    const int height = std::max(1, static_cast<int>(std::round(src.rows * scale)));
    cv::Mat out(side, side, src.type(), cv::Scalar::all(0));
    cv::Mat resized;
    cv::resize(src, resized, { width, height });
    resized.copyTo(out(cv::Rect(0, 0, width, height)));
    return out;
}

auto infer(TensorRtInference& engine, const cv::Mat& input) -> std::vector<float> {
    cv::Mat blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, {}, {}, false, false);
    std::vector<float> values(blob.total());
    std::memcpy(values.data(), blob.ptr<float>(), values.size() * sizeof(float));
    if (!engine.start(values.data(), values.size())) return {};
    auto output = engine.wait();
    return output ? output->get() : std::vector<float> {};
}

auto detect(TensorRtInference& engine, const cv::Mat& rgb) -> std::vector<Detection> {
    float scale = 1.0f;
    const auto raw = infer(engine, letterbox(rgb, scale));
    std::vector<Detection> best;
    for (size_t i = 0; i + 5 < raw.size(); i += 6) {
        const float confidence = raw[i + 4];
        const int id = static_cast<int>(raw[i + 5]);
        if (confidence < kConfidence) continue;
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

void label(cv::Mat& image, const std::string& value, cv::Point point, cv::Scalar color) {
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar::all(0), 5, cv::LINE_AA);
    cv::putText(image, value, point, cv::FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv::LINE_AA);
}

auto main_impl(int argc, char** argv) -> int {
    if (argc < 4) {
        std::cerr << "usage: annotate_l1 <frames> <output> <model_dir>\n";
        return 2;
    }
    const fs::path frames = argv[1];
    const fs::path output = argv[2];
    const fs::path models = argv[3];
    fs::create_directories(output);
    TensorRtInference engine;
    if (!engine.init((models / "best_fixed_names_1280_fp16.engine").string())) return 1;
    std::ofstream csv(output / "results.csv");
    csv << "frame,det_idx,class_id,class_name,confidence,x,y,w,h\n";
    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(frames)) {
        if (entry.path().extension() == ".jpg") files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    const cv::Scalar colors[] = { {255, 180, 0}, {255, 180, 0}, {255, 180, 0}, {255, 180, 0},
        {255, 180, 0}, {0, 220, 255}, {0, 140, 255}, {0, 140, 255}, {0, 140, 255},
        {0, 140, 255}, {0, 140, 255}, {0, 220, 255} };
    for (const auto& file : files) {
        cv::Mat bgr = cv::imread(file.string());
        if (bgr.empty()) continue;
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        cv::Mat annotated = bgr.clone();
        label(annotated, "L1 vehicle  conf>=0.30  per-class max", { 25, 58 }, { 255, 255, 255 });
        const auto detections = detect(engine, rgb);
        int index = 0;
        for (const auto& detection : detections) {
            const cv::Scalar color = colors[std::clamp(detection.id, 0, 11)];
            cv::rectangle(annotated, { static_cast<int>(detection.box.x), static_cast<int>(detection.box.y) },
                { static_cast<int>(detection.box.x + detection.box.width), static_cast<int>(detection.box.y + detection.box.height) }, color, 8);
            const std::string value = std::string(kNames[detection.id]) + "  "
                + std::to_string(static_cast<int>(detection.confidence * 100.0f)) + "%";
            label(annotated, value, { static_cast<int>(detection.box.x),
                std::max(120, static_cast<int>(detection.box.y) - 18) }, color);
            csv << file.filename().string() << "," << index++ << "," << detection.id << ","
                << kNames[detection.id] << "," << detection.confidence << "," << detection.box.x << ","
                << detection.box.y << "," << detection.box.width << "," << detection.box.height << "\n";
        }
        cv::imwrite((output / (file.stem().string() + "_L1.jpg")).string(), annotated);
    }
    return 0;
}
} // namespace

auto main(int argc, char** argv) -> int { return main_impl(argc, argv); }
