#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <expected>
#include <filesystem>
#include <iostream>
#include <print>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "hikcamera/shared_frame_writer.hpp"

namespace {
struct Args {
    std::string video;
    std::string frames_dir;
    std::string shm { "/hikcamera_shm" };
    double speed { 1.0 };
    int max_frames { 0 };  // 0 = unlimited
    double fps { 30.0 };   // 仅 --frames-dir 模式
    bool loop { false };   // video 模式循环播放
};

auto parse_args(int argc, char** argv) -> std::expected<Args, std::string> {
    Args a;
    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto next = [&]() -> std::string {
                if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
                return argv[++i];
            };
            if (arg == "--video") a.video = next();
            else if (arg == "--frames-dir") a.frames_dir = next();
            else if (arg == "--shm") a.shm = next();
            else if (arg == "--speed") a.speed = std::stod(next());
            else if (arg == "--loop") a.loop = true;
            else if (arg == "--fps") a.fps = std::stod(next());
            else if (arg == "--max-frames") a.max_frames = std::stoi(next());
            else return std::unexpected("unknown arg: " + arg);
        }
    } catch (const std::exception& error) {
        return std::unexpected(std::string("bad arg: ") + error.what());
    }
    if (a.video.empty() && a.frames_dir.empty())
        return std::unexpected("--video or --frames-dir required");
    if (a.speed < 0.0) return std::unexpected("--speed must be >= 0 (0 = as fast as possible)");
    return a;
}

auto load_frames(const std::string& dir)
    -> std::expected<std::vector<cv::Mat>, std::string> {
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file()) continue;
        const auto ext = entry.path().extension().string();
        std::string lower;
        for (const char c : ext) lower += static_cast<char>(std::tolower(c));
        if (lower == ".jpg" || lower == ".jpeg" || lower == ".png" || lower == ".bmp") {
            files.push_back(entry.path());
        }
    }
    if (ec) return std::unexpected("scan dir failed: " + ec.message());
    if (files.empty()) return std::unexpected("no image files found in: " + dir);
    std::sort(files.begin(), files.end());
    std::vector<cv::Mat> frames;
    frames.reserve(files.size());
    for (const auto& path : files) {
        cv::Mat img = cv::imread(path.string(), cv::IMREAD_COLOR);
        if (img.empty()) {
            std::println(std::cerr, "warn: failed to read {}", path.string());
            continue;
        }
        frames.push_back(std::move(img));
    }
    if (frames.empty()) return std::unexpected("all images failed to load in: " + dir);
    return frames;
}
} // namespace

auto main(int argc, char** argv) -> int {
    auto args = parse_args(argc, argv);
    if (!args) {
        std::println(std::cerr,
            "usage: mp4_replay --video <mp4> | --frames-dir <dir> [--shm name] [--speed x]"
            " [--fps x] [--max-frames N]");
        std::println(std::cerr, "error: {}", args.error());
        return 2;
    }

    std::vector<cv::Mat> dir_frames;
    cv::VideoCapture cap;
    double fps = 0.0;
    int width = 0, height = 0;
    if (!args->frames_dir.empty()) {
        auto loaded = load_frames(args->frames_dir);
        if (!loaded) {
            std::println(std::cerr, "error: {}", loaded.error());
            return 1;
        }
        dir_frames = std::move(*loaded);
        fps  = args->fps;
        width  = dir_frames.front().cols;
        height = dir_frames.front().rows;
        std::println("[mp4_replay] dir {}: {} frames {}x{} @ {} fps, looping, speed={}",
            args->frames_dir, dir_frames.size(), width, height, fps, args->speed);
    } else {
        cap.open(args->video);
        if (!cap.isOpened()) {
            std::println(std::cerr, "failed to open video: {}", args->video);
            return 1;
        }
        fps    = cap.get(cv::CAP_PROP_FPS);
        width  = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        std::println("[mp4_replay] {} {}x{} @ {} fps, speed={}", args->video, width, height, fps,
            args->speed);
    }

    auto writer = hikcamera::SharedFrameWriter::create(args->shm.c_str());
    if (!writer) {
        std::println(std::cerr, "SHM create failed (exists?): {}", writer.error());
        std::println(std::cerr, "  rm -f /dev/shm{}  or use a different --shm", args->shm);
        return 1;
    }
    std::println("[mp4_replay] SHM {} created", args->shm);

    cv::Mat frame;
    uint64_t seq = 0;
    const auto frame_interval = std::chrono::duration<double>(1.0 / (fps > 0 ? fps : 25.0));

    for (;;) {
        if (!dir_frames.empty()) {
            frame = dir_frames[static_cast<size_t>(seq) % dir_frames.size()];
        } else {
            if (!cap.read(frame)) {
                if (args->loop) {
                    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
                    if (!cap.read(frame)) break;
                } else {
                    break;
                }
            }
        }
        if (frame.empty()) break;
        if (frame.channels() != 3) {
            std::println(std::cerr, "unexpected channels: {}", frame.channels());
            return 1;
        }
        cv::Mat bgr;
        if (frame.type() != CV_8UC3) frame.convertTo(bgr, CV_8UC3);
        else bgr = frame;
        // SHM 规范为 RGB8（hikcamera SDK 真机输出 RGB8，模型按 RGB 训练）；
        // OpenCV 解码为 BGR，写入前统一转 RGB 与真机一致。
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

        ++seq;
        hikcamera::FrameMetadata meta;
        meta.committed_sequence    = seq;
        meta.frame_id              = seq;
        meta.host_monotonic_ns     = static_cast<std::uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
        meta.width                 = static_cast<std::uint32_t>(rgb.cols);
        meta.height                = static_cast<std::uint32_t>(rgb.rows);
        meta.stride_bytes          = static_cast<std::uint32_t>(rgb.cols * 3);
        meta.committed_bytes       = static_cast<std::uint32_t>(rgb.total() * 3);
        meta.pixel_format          = hikcamera::PixelFormat::RGB8;

        auto ret = writer->write(meta, std::span<const unsigned char>(rgb.data, rgb.total() * 3));
        if (!ret) {
            std::println(std::cerr, "SHM write failed at frame {}: {}", seq, ret.error());
            return 1;
        }
        if (seq % 30 == 0) {
            std::println("[mp4_replay] {} frames", seq);
        }
        if (args->speed > 0.0) {
            std::this_thread::sleep_for(frame_interval / args->speed);
        }
        if (args->max_frames > 0 && seq >= static_cast<std::uint64_t>(args->max_frames)) break;
    }

    std::println("[mp4_replay] done: {} frames", seq);
    return 0;
}
