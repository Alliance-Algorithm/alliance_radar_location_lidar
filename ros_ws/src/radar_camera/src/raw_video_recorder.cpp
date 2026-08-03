#include "radar_camera/raw_video_recorder.hpp"

#include <chrono>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sys/resource.h>
#include <sstream>
#include <thread>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace radar_camera::recording {
namespace {

    auto ffmpeg_error(int error) -> std::string {
        char buffer[AV_ERROR_MAX_STRING_SIZE] = { };
        av_strerror(error, buffer, sizeof(buffer));
        return buffer;
    }

    auto time_string(std::chrono::system_clock::time_point time) -> std::string {
        const auto seconds = std::chrono::system_clock::to_time_t(time);
        std::tm tm { };
        gmtime_r(&seconds, &tm);
        std::ostringstream stream;
        stream << std::put_time(&tm, "%Y%m%dT%H%M%SZ");
        return stream.str();
    }

    struct Segment {
        AVFormatContext* format = nullptr;
        AVCodecContext* codec   = nullptr;
        AVStream* stream        = nullptr;
        SwsContext* scaler      = nullptr;
        int scaler_in_w         = 0;  // scaler 输入尺寸（=帧尺寸，惰性创建）
        int scaler_in_h         = 0;
        AVFrame* frame          = nullptr;
        std::ofstream sidecar;
        bool finalized = false;

        ~Segment() {
            if (format != nullptr && format->oformat != nullptr
                && !(format->oformat->flags & AVFMT_NOFILE)) {
                // Destruction cannot report an I/O error; normal shutdown uses close_output().
                avio_closep(&format->pb);
            }
            av_frame_free(&frame);
            sws_freeContext(scaler);
            avcodec_free_context(&codec);
            avformat_free_context(format);
        }

        auto close_output() -> std::expected<void, std::string> {
            if (format == nullptr || format->oformat == nullptr
                || (format->oformat->flags & AVFMT_NOFILE) || format->pb == nullptr) {
                return { };
            }
            const auto result = avio_closep(&format->pb);
            if (result < 0) {
                return std::unexpected("MPEG-TS output close failed: " + ffmpeg_error(result));
            }
            return { };
        }
    };

    enum class DrainResult { packet_drained, no_packets, flushed };

    auto write_packet(Segment& segment, AVPacket* packet) -> std::expected<void, std::string> {
        av_packet_rescale_ts(packet, segment.codec->time_base, segment.stream->time_base);
        const auto result = av_interleaved_write_frame(segment.format, packet);
        av_packet_unref(packet);
        if (result < 0) {
            return std::unexpected("MPEG-TS packet write failed: " + ffmpeg_error(result));
        }
        return { };
    }

    auto drain_packets(Segment& segment, AVPacket* packet, bool flushing)
        -> std::expected<DrainResult, std::string> {
        bool packet_drained = false;
        for (;;) {
            const auto result = avcodec_receive_packet(segment.codec, packet);
            if (result == AVERROR(EAGAIN)) {
                return packet_drained ? DrainResult::packet_drained : DrainResult::no_packets;
            }
            if (result == AVERROR_EOF) {
                if (flushing) {
                    return DrainResult::flushed;
                }
                return std::unexpected("NVENC packet receive reached EOF before flush");
            }
            if (result < 0) {
                return std::unexpected("NVENC packet receive failed: " + ffmpeg_error(result));
            }
            if (const auto written = write_packet(segment, packet); !written) {
                return std::unexpected(written.error());
            }
            packet_drained = true;
            if (!flushing) {
                continue;
            }
        }
    }

    auto finalize_segment(Segment& segment) -> std::expected<void, std::string> {
        if (segment.finalized) {
            return { };
        }
        if (segment.codec != nullptr && segment.format != nullptr) {
            if (segment.stream == nullptr || segment.format->oformat == nullptr) {
                return std::unexpected("incomplete MPEG-TS segment during finalization");
            }
            auto result = avcodec_send_frame(segment.codec, nullptr);
            if (result < 0 && result != AVERROR_EOF) {
                return std::unexpected("NVENC flush send failed: " + ffmpeg_error(result));
            }
            AVPacket* packet = av_packet_alloc();
            if (packet == nullptr) {
                return std::unexpected("could not allocate flush packet");
            }
            for (;;) {
                const auto drained = drain_packets(segment, packet, true);
                if (!drained) {
                    av_packet_free(&packet);
                    return std::unexpected(drained.error());
                }
                if (*drained == DrainResult::flushed) {
                    break;
                }
                if (*drained == DrainResult::no_packets) {
                    av_packet_free(&packet);
                    return std::unexpected("NVENC flush returned EAGAIN");
                }
            }
            av_packet_free(&packet);
            result = av_write_trailer(segment.format);
            if (result < 0) {
                return std::unexpected("MPEG-TS trailer write failed: " + ffmpeg_error(result));
            }
            if (const auto closed = segment.close_output(); !closed) {
                return std::unexpected(closed.error());
            }
        }
        if (segment.sidecar.is_open()) {
            segment.sidecar.flush();
            if (!segment.sidecar) {
                return std::unexpected("sidecar flush failed");
            }
            segment.sidecar.close();
            if (segment.sidecar.fail()) {
                return std::unexpected("sidecar close failed");
            }
        }
        segment.finalized = true;
        return { };
    }

    auto open_segment(const RecordingConfig& config, const std::filesystem::path& path)
        -> std::expected<std::unique_ptr<Segment>, std::string> {
        auto segment        = std::make_unique<Segment>();
        const auto* encoder = avcodec_find_encoder_by_name(config.encoder.c_str());
        if (encoder == nullptr) {
            return std::unexpected("NVENC encoder not found: " + config.encoder);
        }
        auto result =
            avformat_alloc_output_context2(&segment->format, nullptr, "mpegts", path.c_str());
        if (result < 0 || segment->format == nullptr) {
            return std::unexpected("could not allocate MPEG-TS context: " + ffmpeg_error(result));
        }
        segment->stream = avformat_new_stream(segment->format, nullptr);
        segment->codec  = avcodec_alloc_context3(encoder);
        if (segment->stream == nullptr || segment->codec == nullptr) {
            return std::unexpected("could not allocate FFmpeg video stream");
        }
        segment->codec->codec_id     = config.encoder == "hevc_nvenc" ? AV_CODEC_ID_HEVC : AV_CODEC_ID_H264;
        segment->codec->codec_type   = AVMEDIA_TYPE_VIDEO;
        segment->codec->width        = config.width;
        segment->codec->height       = config.height;
        segment->codec->pix_fmt      = AV_PIX_FMT_YUV420P;
        segment->codec->time_base    = AVRational { 1, config.fps };
        segment->codec->framerate    = AVRational { config.fps, 1 };
        segment->codec->gop_size     = config.gop;
        segment->codec->bit_rate     = config.bitrate;
        segment->codec->max_b_frames = 0;
        if (config.encoder != "libx264") {  // nvenc (h264/hevc) 通用私参
            av_opt_set(segment->codec->priv_data, "preset", "p1", 0);
            av_opt_set(segment->codec->priv_data, "tune", "ll", 0);
            av_opt_set(segment->codec->priv_data, "bf", "0", 0);
        } else if (config.encoder == "libx264") {
            // 软编码（无 NVENC 的开发/测试环境）：libx264 不支持 nvenc 的
            // preset p1/tune ll 私参，改用 ultrafast + zerolatency 低延迟配置。
            av_opt_set(segment->codec->priv_data, "preset", "ultrafast", 0);
            av_opt_set(segment->codec->priv_data, "tune", "zerolatency", 0);
        }
        result = avcodec_open2(segment->codec, encoder, nullptr);
        if (result < 0) {
            return std::unexpected("could not open " + config.encoder + ": "
                + ffmpeg_error(result));
        }
        result = avcodec_parameters_from_context(segment->stream->codecpar, segment->codec);
        if (result < 0) {
            return std::unexpected("could not copy codec parameters: " + ffmpeg_error(result));
        }
        segment->stream->time_base = segment->codec->time_base;
        if (segment->format->oformat == nullptr) {
            return std::unexpected("MPEG-TS format has no output format");
        }
        if (!(segment->format->oformat->flags & AVFMT_NOFILE)) {
            result = avio_open(&segment->format->pb, path.c_str(), AVIO_FLAG_WRITE);
            if (result < 0) {
                return std::unexpected("could not open segment: " + ffmpeg_error(result));
            }
        }
        result = avformat_write_header(segment->format, nullptr);
        if (result < 0) {
            return std::unexpected("could not write MPEG-TS header: " + ffmpeg_error(result));
        }
        // scaler 惰性创建：输入尺寸=实际帧尺寸（相机 5472x3648），输出=config 编码尺寸。
        // 5472x3648 超出 NVENC H.264 上限 (4096x4096)，录制按 config 缩放后编码。
        segment->frame = av_frame_alloc();
        if (segment->frame == nullptr) {
            return std::unexpected("could not allocate RGB-to-YUV conversion resources");
        }
        segment->frame->format = AV_PIX_FMT_YUV420P;
        segment->frame->width  = config.width;
        segment->frame->height = config.height;
        result                 = av_frame_get_buffer(segment->frame, 32);
        if (result < 0) {
            return std::unexpected("could not allocate YUV frame: " + ffmpeg_error(result));
        }
        segment->sidecar.open(path.string() + ".jsonl", std::ios::out | std::ios::trunc);
        if (!segment->sidecar) {
            return std::unexpected("could not open sidecar file");
        }
        return segment;
    }

    auto probe_encoder(const RecordingConfig& config) -> std::expected<void, std::string> {
        const auto* encoder = avcodec_find_encoder_by_name(config.encoder.c_str());
        if (encoder == nullptr) {
            return std::unexpected("NVENC encoder not found: " + config.encoder);
        }
        auto* codec = avcodec_alloc_context3(encoder);
        if (codec == nullptr) {
            return std::unexpected("could not allocate h264_nvenc probe context");
        }
        codec->codec_id     = config.encoder == "hevc_nvenc" ? AV_CODEC_ID_HEVC : AV_CODEC_ID_H264;
        codec->codec_type   = AVMEDIA_TYPE_VIDEO;
        codec->width        = config.width;
        codec->height       = config.height;
        codec->pix_fmt      = AV_PIX_FMT_YUV420P;
        codec->time_base    = AVRational { 1, config.fps };
        codec->framerate    = AVRational { config.fps, 1 };
        codec->gop_size     = config.gop;
        codec->bit_rate     = config.bitrate;
        codec->max_b_frames = 0;
        if (config.encoder != "libx264") {  // nvenc (h264/hevc) 通用私参
            av_opt_set(codec->priv_data, "preset", "p1", 0);
            av_opt_set(codec->priv_data, "tune", "ll", 0);
            av_opt_set(codec->priv_data, "bf", "0", 0);
        } else if (config.encoder == "libx264") {
            av_opt_set(codec->priv_data, "preset", "ultrafast", 0);
            av_opt_set(codec->priv_data, "tune", "zerolatency", 0);
        }
        const auto result = avcodec_open2(codec, encoder, nullptr);
        avcodec_free_context(&codec);
        if (result < 0) {
            return std::unexpected("could not open " + config.encoder + ": "
                + ffmpeg_error(result));
        }
        return { };
    }

} // namespace

auto segment_path(const std::filesystem::path& output_dir,
    std::chrono::system_clock::time_point session_start, std::size_t segment_index)
    -> std::filesystem::path {
    std::ostringstream index;
    index << std::setfill('0') << std::setw(6) << segment_index;
    return output_dir / (time_string(session_start) + "_" + index.str() + ".ts");
}

RawVideoRecorder::RawVideoRecorder(RecordingConfig config, RecordingFifo& fifo)
    : config_(std::move(config))
    , fifo_(fifo) { }

RawVideoRecorder::~RawVideoRecorder() { stop(); }

auto RawVideoRecorder::start() -> std::expected<void, std::string> {
    std::lock_guard lifecycle_lock(lifecycle_mutex_);
    if (!config_.enabled) {
        return { };
    }
    if (const auto valid = validate_config(config_); !valid) {
        return std::unexpected(valid.error());
    }
    try {
        std::filesystem::create_directories(config_.output_dir);
        if (!std::filesystem::is_directory(config_.output_dir)) {
            return std::unexpected("recording output path is not a directory");
        }
    } catch (const std::exception& error) {
        return std::unexpected(
            std::string("could not prepare recording output directory: ") + error.what());
    }
    if (const auto encoder = probe_encoder(config_); !encoder) {
        return std::unexpected(encoder.error());
    }
    if (running_.exchange(true, std::memory_order_acq_rel)) {
        return std::unexpected("recorder is already running");
    }
    try {
        if (thread_.joinable()) {
            thread_.join();
        }
        fifo_.reset();
        stop_requested_.store(false, std::memory_order_release);
        {
            std::lock_guard lock(mutex_);
            state_ = RecorderState::running;
            stats_ = { };
            failure_reason_.clear();
        }
        thread_ = std::thread(&RawVideoRecorder::loop, this);
        // 录制线程低优先级：CPU 紧张时优先保障推理线程（nice +10）。
        if (thread_.joinable()) {
            setpriority(PRIO_PROCESS, static_cast<id_t>(thread_.native_handle()), 10);
        }
    } catch (const std::exception& error) {
        running_.store(false, std::memory_order_release);
        fifo_.request_overrun(std::string("could not start recorder: ") + error.what());
        std::lock_guard lock(mutex_);
        ++stats_.errors;
        state_          = RecorderState::failed;
        failure_reason_ = std::string("could not start recorder: ") + error.what();
        return std::unexpected(std::string("could not start recorder: ") + error.what());
    }
    return { };
}

void RawVideoRecorder::stop() {
    std::lock_guard lifecycle_lock(lifecycle_mutex_);
    if (thread_.joinable() && running_.load(std::memory_order_acquire)) {
        // Closing the input prevents new frames while the worker drains accepted frames.
        fifo_.close();
        stop_requested_.store(true, std::memory_order_release);
    } else {
        running_.store(false, std::memory_order_release);
    }
    if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(false, std::memory_order_release);
    std::lock_guard lock(mutex_);
    if (state_ == RecorderState::running) {
        state_ = RecorderState::stopped;
    }
}

void RawVideoRecorder::fail(std::string reason, bool overrun) {
    fifo_.request_overrun(reason);
    stop_requested_.store(false, std::memory_order_release);
    std::lock_guard lock(mutex_);
    ++stats_.errors;
    if (overrun) {
        ++stats_.overruns;
    }
    state_          = overrun ? RecorderState::overrun : RecorderState::failed;
    failure_reason_ = std::move(reason);
    running_.store(false, std::memory_order_release);
}

auto RawVideoRecorder::state() const -> RecorderState {
    std::lock_guard lock(mutex_);
    return state_;
}

auto RawVideoRecorder::stats() const -> RecorderStats {
    std::lock_guard lock(mutex_);
    return stats_;
}

auto RawVideoRecorder::failure_reason() const -> std::string {
    std::lock_guard lock(mutex_);
    return failure_reason_;
}

void RawVideoRecorder::loop() {
    const auto session_start  = std::chrono::system_clock::now();
    std::size_t segment_index = 0;
    std::uint64_t frame_index = 0;
    const auto segment_frames = static_cast<std::uint64_t>(config_.fps)
        * static_cast<std::uint64_t>(config_.segment_duration_sec);
    auto segment =
        open_segment(config_, segment_path(config_.output_dir, session_start, segment_index));
    if (!segment) {
        fail(segment.error(), false);
        return;
    }
    {
        std::lock_guard lock(mutex_);
        ++stats_.segments;
    }
    while (running_.load(std::memory_order_acquire)
        || stop_requested_.load(std::memory_order_acquire)) {
        if (fifo_.overrun()) {
            fail("recording FIFO overrun", true);
            break;
        }
        auto frame = fifo_.pop();
        if (!frame) {
            if (stop_requested_.load(std::memory_order_acquire)) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        // 主动丢帧：相机 20fps 输入，按 config.fps 节流编码（如 5472 HEVC 只能 ~13fps，
        // 录 10fps）。不丢帧则 fifo 填满 OVERRUN 停掉整条推理链路。
        const auto now = std::chrono::steady_clock::now();
        const auto frame_interval =
            std::chrono::duration<double>(1.0 / static_cast<double>(config_.fps));
        if (now - last_encoded_ < frame_interval) {
            {
                std::lock_guard lock(mutex_);
                ++stats_.dropped;
            }
            continue;
        }
        last_encoded_ = now;
        {
            std::lock_guard lock(mutex_);
            ++stats_.queued;
        }
        if (frame->rgb.type() != CV_8UC3) {
            fail("raw frame format does not match recorder", true);
            break;
        }
        if ((*segment)->scaler == nullptr || (*segment)->scaler_in_w != frame->rgb.cols
            || (*segment)->scaler_in_h != frame->rgb.rows) {
            sws_freeContext((*segment)->scaler);
            (*segment)->scaler = sws_getContext(frame->rgb.cols, frame->rgb.rows,
                AV_PIX_FMT_RGB24, config_.width, config_.height, AV_PIX_FMT_YUV420P,
                SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
            if ((*segment)->scaler == nullptr) {
                fail("could not allocate RGB-to-YUV scaler", false);
                break;
            }
            (*segment)->scaler_in_w = frame->rgb.cols;
            (*segment)->scaler_in_h = frame->rgb.rows;
        }
        auto result = av_frame_make_writable((*segment)->frame);
        if (result < 0) {
            fail("could not make YUV frame writable: " + ffmpeg_error(result), false);
            break;
        }
        const std::uint8_t* source[] = { frame->rgb.ptr<std::uint8_t>() };
        const int source_stride[]    = { static_cast<int>(frame->rgb.step) };
        if (sws_scale((*segment)->scaler, source, source_stride, 0, frame->rgb.rows,
                (*segment)->frame->data, (*segment)->frame->linesize)
            <= 0) {
            fail("RGB-to-YUV conversion failed", false);
            break;
        }
        AVPacket* packet = av_packet_alloc();
        if (packet == nullptr) {
            fail("could not allocate encoded packet", false);
            break;
        }
        if (frame_index >= static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())) {
            av_packet_free(&packet);
            fail("frame index exceeds FFmpeg timestamp range", false);
            break;
        }
        (*segment)->frame->pts = static_cast<std::int64_t>(frame_index);
        for (;;) {
            result = avcodec_send_frame((*segment)->codec, (*segment)->frame);
            if (result != AVERROR(EAGAIN)) {
                break;
            }
            const auto drained = drain_packets(**segment, packet, false);
            if (!drained) {
                av_packet_free(&packet);
                fail(drained.error(), false);
                break;
            }
            if (*drained == DrainResult::no_packets) {
                av_packet_free(&packet);
                fail("NVENC send returned EAGAIN without a packet to drain", false);
                break;
            }
        }
        if (!running_.load(std::memory_order_acquire)
            && !stop_requested_.load(std::memory_order_acquire)) {
            av_packet_free(&packet);
            break;
        }
        if (result < 0) {
            av_packet_free(&packet);
            fail("NVENC frame send failed: " + ffmpeg_error(result), false);
            break;
        }
        const auto drained = drain_packets(**segment, packet, false);
        av_packet_free(&packet);
        if (!drained) {
            fail(drained.error(), false);
            break;
        }
        ++frame_index;
        (*segment)->sidecar
            << "{\"sequence\":" << frame->sequence
            << ",\"source_monotonic_ns\":" << frame->host_monotonic_ns << ",\"segment\":\""
            << segment_path(config_.output_dir, session_start, segment_index).filename().string()
            << "\",\"overruns\":" << fifo_.overrun() << ",\"errors\":" << stats().errors << "}\n";
        if (!(*segment)->sidecar) {
            fail("sidecar write failed", false);
            break;
        }
        (*segment)->sidecar.flush();
        if (!(*segment)->sidecar) {
            fail("sidecar write or flush failed", false);
            break;
        }
        {
            std::lock_guard lock(mutex_);
            ++stats_.encoded;
        }
        if (frame_index % segment_frames == 0) {
            if (const auto finalized = finalize_segment(**segment); !finalized) {
                fail(finalized.error(), false);
                break;
            }
            segment->reset();
            if (segment_index == std::numeric_limits<std::size_t>::max()) {
                fail("segment index overflow", false);
                break;
            }
            ++segment_index;
            segment = open_segment(
                config_, segment_path(config_.output_dir, session_start, segment_index));
            if (!segment) {
                fail(segment.error(), false);
                break;
            }
            std::lock_guard lock(mutex_);
            ++stats_.segments;
        }
    }
    if (segment) {
        if (const auto finalized = finalize_segment(**segment); !finalized) {
            fail(finalized.error(), false);
        }
    }
}

} // namespace radar_camera::recording
