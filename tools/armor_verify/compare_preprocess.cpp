#include <chrono>
#include <filesystem>
#include <iostream>
#include <print>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "radar_camera/armor_infer.hpp"
#include "radar_camera/tensorrt_inference.hpp"

namespace fs = std::filesystem;
using radar_camera::model_inference::TensorRtInference;
using radar_camera::armor_infer::decode_l1;
using radar_camera::armor_infer::l1_names;

namespace {

constexpr int kSide = 1280;
constexpr float kConf = 0.20f;

auto letterbox(const cv::Mat& src, int side) -> cv::Mat {
    const float scale = std::min(static_cast<float>(side) / src.cols,
        static_cast<float>(side) / src.rows);
    const int w = std::max(1, static_cast<int>(std::lround(src.cols * scale)));
    const int h = std::max(1, static_cast<int>(std::lround(src.rows * scale)));
    const int px = (side - w) / 2;
    const int py = (side - h) / 2;
    cv::Mat out(side, side, CV_8UC3, cv::Scalar::all(0));
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(w, h));
    resized.copyTo(out(cv::Rect(px, py, w, h)));
    return out;
}

auto stretch(const cv::Mat& src, int side) -> cv::Mat {
    cv::Mat out;
    cv::resize(src, out, cv::Size(side, side));
    return out;
}

auto to_blob(const cv::Mat& rgb) -> std::vector<float> {
    cv::Mat blob = cv::dnn::blobFromImage(rgb, 1.0 / 255.0, cv::Size(), cv::Scalar(), false, false);
    std::vector<float> buf(blob.total());
    std::memcpy(buf.data(), blob.ptr<float>(), buf.size() * sizeof(float));
    return buf;
}

struct Stats {
    int frames_with_dets = 0;
    int total_dets = 0;
    float conf_sum = 0.0f;
    int drone5_frames = 0;
    int drone11_frames = 0;
};

void run(TensorRtInference& engine, const cv::Mat& rgb, const std::string& mode, Stats& s,
    int frame_idx) {
    cv::Mat input = (mode == "letterbox") ? letterbox(rgb, kSide) : stretch(rgb, kSide);
    if (frame_idx == 0) {
        std::println("{} input: {}x{} type={} empty={}", mode, input.cols, input.rows,
            input.type(), input.empty());
    }
    auto blob = to_blob(input);
    if (frame_idx == 0) std::println("{} blob: {} elems", mode, blob.size());
    if (blob.empty()) return;
    if (!engine.start(blob.data(), blob.size())) {
        if (frame_idx == 0) std::println("{} start failed", mode);
        return;
    }
    auto out = engine.wait();
    if (!out) {
        if (frame_idx == 0) std::println("{} wait failed", mode);
        return;
    }
    if (frame_idx == 0) std::println("{} raw size: {}", mode, out->get().size());
    auto dets = decode_l1(out->get(), 1.0f, kConf);
    if (!dets.empty()) {
        ++s.frames_with_dets;
        for (const auto& d : dets) {
            ++s.total_dets;
            s.conf_sum += d.conf;
            if (d.id == 5) ++s.drone5_frames;
            if (d.id == 11) ++s.drone11_frames;
        }
    }
}

} // namespace

auto main(int argc, char** argv) -> int {
    if (argc < 3) {
        std::println(std::cerr, "usage: compare_preprocess <frames_dir> <model_dir>");
        return 2;
    }
    const fs::path dir = argv[1];
    TensorRtInference engine;
    auto init = engine.init((fs::path(argv[2]) / "best_fixed_names_1280_fp16.engine").string());
    if (!init) {
        std::println(std::cerr, "engine init failed: {}", init.error());
        return 1;
    }

    std::vector<fs::path> files;
    for (const auto& e : fs::directory_iterator(dir)) {
        if (e.path().extension() == ".jpg") files.push_back(e.path());
    }
    std::sort(files.begin(), files.end());
    std::println("frames: {}", files.size());

    Stats lb, st;
    for (size_t i = 0; i < files.size(); ++i) { const auto& f = files[i];
        cv::Mat bgr = cv::imread(f.string());
        if (bgr.empty()) continue;
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        run(engine, rgb, "letterbox", lb, i);
        run(engine, rgb, "stretch", st, i);
    }

    const auto dump = [](const char* mode, const Stats& s, int nframes) {
        std::println("{}: frames_with_dets={}/{} total_dets={} avg_conf={:.3f} drone_id5_frames={} drone_id11_frames={}",
            mode, s.frames_with_dets, nframes, s.total_dets,
            s.total_dets > 0 ? s.conf_sum / s.total_dets : 0.0f,
            s.drone5_frames, s.drone11_frames);
    };
    dump("letterbox", lb, static_cast<int>(files.size()));
    dump("stretch  ", st, static_cast<int>(files.size()));
    return 0;
}
