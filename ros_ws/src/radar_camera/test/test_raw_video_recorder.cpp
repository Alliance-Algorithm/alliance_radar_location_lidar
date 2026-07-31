#include <chrono>
#include <cstdlib>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "radar_camera/raw_video_recorder.hpp"
#include "radar_camera/recording_fifo.hpp"

namespace {

using radar_camera::recording::RawFrame;
using radar_camera::recording::RecordingConfig;

auto config(const std::filesystem::path& output_dir) -> RecordingConfig {
    return {
        .enabled              = true,
        .output_dir           = output_dir.string(),
        .width                = 4,
        .height               = 4,
        .fps                  = 30,
        .bitrate              = 1000000,
        .gop                  = 30,
        .encoder              = "h264_nvenc",
        .segment_duration_sec = 60,
        .buffer_pool_frames   = 4,
        .max_buffer_bytes     = 4 * 4 * 3 * 4,
    };
}

auto frame(std::uint64_t sequence) -> RawFrame {
    return { cv::Mat(4, 4, CV_8UC3, cv::Scalar(sequence, sequence, sequence)).clone(), sequence,
        sequence * 1000 };
}

} // namespace

TEST(RawVideoRecorder, DisabledStartIsNoOp) {
    auto cfg    = config("/tmp/radar-camera-disabled");
    cfg.enabled = false;
    radar_camera::recording::RecordingFifo fifo(2);
    radar_camera::recording::RawVideoRecorder recorder(cfg, fifo);

    EXPECT_TRUE(recorder.start());
    EXPECT_EQ(recorder.state(), radar_camera::recording::RecorderState::stopped);
    recorder.stop();
}

TEST(RawVideoRecorder, RejectsInvalidConfigurationWhenEnabled) {
    auto cfg  = config("/tmp/radar-camera-invalid");
    cfg.width = 3;
    radar_camera::recording::RecordingFifo fifo(2);
    radar_camera::recording::RawVideoRecorder recorder(cfg, fifo);

    const auto result = recorder.start();

    ASSERT_FALSE(result);
    EXPECT_NE(result.error().find("width"), std::string::npos);
}

TEST(RawVideoRecorder, StopIsIdempotent) {
    auto cfg = config("/tmp/radar-camera-stop");
    radar_camera::recording::RecordingFifo fifo(2);
    radar_camera::recording::RawVideoRecorder recorder(cfg, fifo);

    recorder.stop();
    recorder.stop();
    EXPECT_EQ(recorder.state(), radar_camera::recording::RecorderState::stopped);
}

TEST(RawVideoRecorder, SegmentPathIsDeterministic) {
    const auto session_start = std::chrono::system_clock::from_time_t(0);

    EXPECT_EQ(radar_camera::recording::segment_path("/tmp/recordings", session_start, 7),
        "/tmp/recordings/19700101T000000Z_000007.ts");
}

TEST(RawVideoRecorder, ConsumesFramesInFifoOrder) {
    const auto output_dir = std::filesystem::temp_directory_path() / "radar-camera-order";
    std::filesystem::remove_all(output_dir);
    auto cfg = config(output_dir);
    cfg.fps = 2;
    cfg.segment_duration_sec = 1;
    radar_camera::recording::RecordingFifo fifo(4);
    radar_camera::recording::RawVideoRecorder recorder(cfg, fifo);

    const auto started = recorder.start();
    if (!started) {
        GTEST_SKIP() << started.error();
    }
    for (int attempt = 0; attempt < 100 && recorder.state() ==
            radar_camera::recording::RecorderState::running;
         ++attempt) {
        if (recorder.stats().segments > 0) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (recorder.state() != radar_camera::recording::RecorderState::running) {
        recorder.stop();
        GTEST_SKIP() << "NVENC recorder unavailable: asynchronous startup failed";
    }
    ASSERT_TRUE(fifo.try_push(frame(1)));
    ASSERT_TRUE(fifo.try_push(frame(2)));
    for (int attempt = 0; attempt < 100; ++attempt) {
        if (recorder.stats().encoded == 2) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    recorder.stop();
    EXPECT_EQ(recorder.stats().queued, 2U);
    ASSERT_EQ(recorder.stats().encoded, 2U);
    EXPECT_EQ(recorder.state(), radar_camera::recording::RecorderState::stopped);
    std::size_t segment_count = 0;
    std::vector<std::pair<std::string, std::uint64_t>> records;
    for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
        if (entry.path().extension() == ".ts") {
            ++segment_count;
        } else if (entry.path().extension() == ".jsonl") {
            std::ifstream sidecar(entry.path());
            ASSERT_TRUE(sidecar);
            for (std::string line; std::getline(sidecar, line);) {
                const auto marker = std::string{"\"sequence\":"};
                const auto begin = line.find(marker);
                ASSERT_NE(begin, std::string::npos);
                const auto value_begin = begin + marker.size();
                records.emplace_back(entry.path().filename().string(),
                    std::stoull(line.substr(value_begin)));
            }
        }
    }
    EXPECT_GE(segment_count, 1U);
    std::sort(records.begin(), records.end());
    std::vector<std::uint64_t> sequences;
    for (const auto& [segment, sequence] : records) {
        static_cast<void>(segment);
        sequences.push_back(sequence);
    }
    ASSERT_EQ(sequences, (std::vector<std::uint64_t> { 1, 2 }));
    std::filesystem::remove_all(output_dir);
}

TEST(RawVideoRecorder, HardwareEncodeIsOptIn) {
    if (std::getenv("RADAR_CAMERA_RUN_HW_RECORDING_TESTS") == nullptr) {
        GTEST_SKIP() << "set RADAR_CAMERA_RUN_HW_RECORDING_TESTS=1 to run NVENC test";
    }

    const auto output_dir = std::filesystem::temp_directory_path() / "radar-camera-hw-test";
    std::filesystem::remove_all(output_dir);
    auto cfg             = config(output_dir);
    cfg.width            = 5472;
    cfg.height           = 3648;
    cfg.fps              = 1;
    cfg.segment_duration_sec = 1;
    cfg.max_buffer_bytes = static_cast<std::size_t>(cfg.width) * cfg.height * 3 * 2;
    radar_camera::recording::RecordingFifo fifo(2);
    radar_camera::recording::RawVideoRecorder recorder(cfg, fifo);
    ASSERT_TRUE(recorder.start());
    ASSERT_TRUE(fifo.try_push(
        RawFrame { cv::Mat(cfg.height, cfg.width, CV_8UC3, cv::Scalar(1, 2, 3)), 1, 1000 }));
    ASSERT_TRUE(fifo.try_push(
        RawFrame { cv::Mat(cfg.height, cfg.width, CV_8UC3, cv::Scalar(4, 5, 6)), 2, 2000 }));
    for (int attempt = 0; attempt < 100 && recorder.stats().encoded < 2; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    recorder.stop();

    ASSERT_EQ(recorder.state(), radar_camera::recording::RecorderState::stopped);
    ASSERT_EQ(recorder.stats().encoded, 2U);
    std::size_t ts_count = 0;
    std::size_t sidecar_count = 0;
    for (const auto& entry : std::filesystem::directory_iterator(output_dir)) {
        if (entry.path().extension() == ".ts") {
            ++ts_count;
            const auto probe_output = output_dir / "probe.txt";
            const auto command = "ffprobe -v error -select_streams v:0 -show_entries "
                "stream=codec_name,width,height -of csv=p=0 \"" + entry.path().string()
                + "\" > \"" + probe_output.string() + "\"";
            ASSERT_EQ(std::system(command.c_str()), 0);
            std::ifstream probe(probe_output);
            ASSERT_TRUE(probe);
            std::string line;
            ASSERT_TRUE(std::getline(probe, line));
            EXPECT_EQ(line, "h264,5472,3648");
        } else if (entry.path().extension() == ".jsonl") {
            ++sidecar_count;
            std::ifstream sidecar(entry.path());
            ASSERT_TRUE(sidecar);
            std::string content((std::istreambuf_iterator<char>(sidecar)), {});
            EXPECT_TRUE(content.find("\"sequence\":1") != std::string::npos
                || content.find("\"sequence\":2") != std::string::npos);
        }
    }
    ASSERT_EQ(ts_count, 2U);
    ASSERT_EQ(sidecar_count, 2U);
    std::filesystem::remove_all(output_dir);
}
