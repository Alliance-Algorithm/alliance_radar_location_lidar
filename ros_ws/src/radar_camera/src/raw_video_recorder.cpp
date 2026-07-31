#include "radar_camera/raw_video_recorder.hpp"

#include <chrono>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>

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
        AVFrame* frame          = nullptr;
        std::ofstream sidecar;

        ~Segment() {
            if (format != nullptr) {
                if (codec != nullptr) {
                    avcodec_send_frame(codec, nullptr);
                    AVPacket* packet = av_packet_alloc();
                    while (packet != nullptr && avcodec_receive_packet(codec, packet) >= 0) {
                        av_packet_rescale_ts(packet, codec->time_base, stream->time_base);
                        av_interleaved_write_frame(format, packet);
                        av_packet_unref(packet);
                    }
                    av_packet_free(&packet);
                }
                av_write_trailer(format);
            }
            if (format != nullptr && !(format->oformat->flags & AVFMT_NOFILE)) {
                avio_closep(&format->pb);
            }
            av_frame_free(&frame);
            sws_freeContext(scaler);
            avcodec_free_context(&codec);
            avformat_free_context(format);
        }
    };

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
        segment->codec->codec_id     = AV_CODEC_ID_H264;
        segment->codec->codec_type   = AVMEDIA_TYPE_VIDEO;
        segment->codec->width        = config.width;
        segment->codec->height       = config.height;
        segment->codec->pix_fmt      = AV_PIX_FMT_YUV420P;
        segment->codec->time_base    = AVRational { 1, config.fps };
        segment->codec->framerate    = AVRational { config.fps, 1 };
        segment->codec->gop_size     = config.gop;
        segment->codec->bit_rate     = config.bitrate;
        segment->codec->max_b_frames = 0;
        segment->codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        av_opt_set(segment->codec->priv_data, "preset", "p1", 0);
        av_opt_set(segment->codec->priv_data, "tune", "ll", 0);
        av_opt_set(segment->codec->priv_data, "bf", "0", 0);
        result = avcodec_open2(segment->codec, encoder, nullptr);
        if (result < 0) {
            return std::unexpected("could not open h264_nvenc: " + ffmpeg_error(result));
        }
        result = avcodec_parameters_from_context(segment->stream->codecpar, segment->codec);
        if (result < 0) {
            return std::unexpected("could not copy codec parameters: " + ffmpeg_error(result));
        }
        segment->stream->time_base = segment->codec->time_base;
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
        segment->scaler =
            sws_getContext(config.width, config.height, AV_PIX_FMT_RGB24, config.width,
                config.height, AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
        segment->frame = av_frame_alloc();
        if (segment->scaler == nullptr || segment->frame == nullptr) {
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
    if (running_.exchange(true, std::memory_order_acq_rel)) {
        return std::unexpected("recorder is already running");
    }
    try {
        std::filesystem::create_directories(config_.output_dir);
        {
            std::lock_guard lock(mutex_);
            state_ = RecorderState::running;
            stats_ = { };
        }
        thread_ = std::thread(&RawVideoRecorder::loop, this);
    } catch (const std::exception& error) {
        running_.store(false, std::memory_order_release);
        std::lock_guard lock(mutex_);
        state_ = RecorderState::failed;
        return std::unexpected(std::string("could not start recorder: ") + error.what());
    }
    return { };
}

void RawVideoRecorder::stop() {
    std::lock_guard lifecycle_lock(lifecycle_mutex_);
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
    std::lock_guard lock(mutex_);
    if (state_ == RecorderState::running) {
        state_ = RecorderState::stopped;
    }
}

auto RawVideoRecorder::state() const -> RecorderState {
    std::lock_guard lock(mutex_);
    return state_;
}

auto RawVideoRecorder::stats() const -> RecorderStats {
    std::lock_guard lock(mutex_);
    return stats_;
}

void RawVideoRecorder::loop() {
    const auto session_start  = std::chrono::system_clock::now();
    std::size_t segment_index = 0;
    std::uint64_t frame_index = 0;
    auto segment =
        open_segment(config_, segment_path(config_.output_dir, session_start, segment_index));
    if (!segment) {
        fifo_.request_overrun(segment.error());
        std::lock_guard lock(mutex_);
        ++stats_.errors;
        ++stats_.overruns;
        state_ = RecorderState::failed;
        running_.store(false, std::memory_order_release);
        return;
    }
    {
        std::lock_guard lock(mutex_);
        ++stats_.segments;
    }
    while (running_.load(std::memory_order_acquire)) {
        if (fifo_.overrun()) {
            std::lock_guard lock(mutex_);
            ++stats_.overruns;
            state_ = RecorderState::failed;
            break;
        }
        auto frame = fifo_.pop();
        if (!frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (frame->rgb.cols != config_.width || frame->rgb.rows != config_.height
            || frame->rgb.type() != CV_8UC3) {
            fifo_.request_overrun("raw frame dimensions or format do not match recorder");
            std::lock_guard lock(mutex_);
            ++stats_.errors;
            ++stats_.overruns;
            state_ = RecorderState::failed;
            break;
        }
        auto result = av_frame_make_writable(segment->frame);
        if (result < 0) {
            fifo_.request_overrun("could not make YUV frame writable: " + ffmpeg_error(result));
            std::lock_guard lock(mutex_);
            ++stats_.errors;
            ++stats_.overruns;
            state_ = RecorderState::failed;
            break;
        }
        const auto* source[]      = { frame->rgb.ptr<std::uint8_t>() };
        const int source_stride[] = { static_cast<int>(frame->rgb.step) };
        if (sws_scale(segment->scaler, source, source_stride, 0, config_.height,
                segment->frame->data, segment->frame->linesize)
            <= 0) {
            fifo_.request_overrun("RGB-to-YUV conversion failed");
            std::lock_guard lock(mutex_);
            ++stats_.errors;
            ++stats_.overruns;
            state_ = RecorderState::failed;
            break;
        }
        segment->frame->pts = static_cast<std::int64_t>(frame_index++);
        result              = avcodec_send_frame(segment->codec, segment->frame);
        AVPacket* packet    = av_packet_alloc();
        if (packet == nullptr) {
            result = AVERROR(ENOMEM);
        }
        while (result >= 0 && packet != nullptr
            && (result = avcodec_receive_packet(segment->codec, packet)) >= 0) {
            av_packet_rescale_ts(packet, segment->codec->time_base, segment->stream->time_base);
            result = av_interleaved_write_frame(segment->format, packet);
            av_packet_unref(packet);
        }
        av_packet_free(&packet);
        if (result < 0 && result != AVERROR(EAGAIN)) {
            fifo_.request_overrun("NVENC encode or MPEG-TS write failed: " + ffmpeg_error(result));
            std::lock_guard lock(mutex_);
            ++stats_.errors;
            ++stats_.overruns;
            state_ = RecorderState::failed;
            break;
        }
        segment->sidecar
            << "{\"sequence\":" << frame->sequence
            << ",\"source_monotonic_ns\":" << frame->host_monotonic_ns << ",\"segment\":\""
            << segment_path(config_.output_dir, session_start, segment_index).filename().string()
            << "\",\"overruns\":" << fifo_.overrun() << ",\"errors\":" << stats().errors << "}\n";
        {
            std::lock_guard lock(mutex_);
            ++stats_.queued;
            ++stats_.encoded;
        }
        const auto segment_frames = static_cast<std::uint64_t>(config_.fps)
            * static_cast<std::uint64_t>(config_.segment_duration_sec);
        if (frame_index % segment_frames == 0) {
            segment.reset();
            ++segment_index;
            segment = open_segment(
                config_, segment_path(config_.output_dir, session_start, segment_index));
            if (!segment) {
                fifo_.request_overrun(segment.error());
                std::lock_guard lock(mutex_);
                ++stats_.errors;
                ++stats_.overruns;
                state_ = RecorderState::failed;
                break;
            }
            std::lock_guard lock(mutex_);
            ++stats_.segments;
        }
    }
}

} // namespace radar_camera::recording
