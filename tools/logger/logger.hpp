#pragma once

#include "logger_config.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <print>
#include <string>

namespace Radar::Logger {

// header-only 实现: 用函数内 static 存放 sink 状态, 避免额外的 .cpp/链接目标。
class Logger final {
public:
    static void Init(const std::string& log_root) {
        auto& state    = State();
        state.log_root = log_root;
        std::error_code ec;
        std::filesystem::create_directories(log_root, ec);
        std::lock_guard lock(state.mutex);
        state.file.open(log_root + "/radar.log", std::ios::app);
    }

    static void Shutdown() {
        auto& state = State();
        std::lock_guard lock(state.mutex);
        if (state.file.is_open()) {
            state.file.close();
        }
    }

    template <LogConfig::LogLevel level>
    static void LOG(const std::string& module_name, const std::string& message) {
        const auto log_data =
            LogConfig::LogData { .level = level, .module_name = module_name, .message = message };
        const auto line = std::format("[{}] [{}] [{}] {}", log_data.timestamp,
            LevelName(level), log_data.module_name, log_data.message);

        auto& state = State();
        std::lock_guard lock(state.mutex);
        std::println(level == LogConfig::LogLevel::ERROR ? stderr : stdout, "{}", line);
        if (state.file.is_open()) {
            state.file << line << '\n';
            state.file.flush();
        }
    }

private:
    Logger() = delete;

    struct SinkState {
        std::string log_root;
        std::ofstream file;
        std::mutex mutex;
    };

    static auto State() -> SinkState& {
        static SinkState state;
        return state;
    }

    static constexpr auto LevelName(LogConfig::LogLevel level) -> const char* {
        switch (level) {
            case LogConfig::LogLevel::DEBUG:   return "DEBUG";
            case LogConfig::LogLevel::INFO:    return "INFO";
            case LogConfig::LogLevel::WARNING: return "WARN";
            case LogConfig::LogLevel::ERROR:   return "ERROR";
        }
        return "UNKNOWN";
    }
};

} // namespace Radar::Logger

#define LOG_INFO(module, msg)                                                                      \
    Radar::Logger::Logger::LOG<Radar::Logger::LogConfig::LogLevel::INFO>(module, msg)
#define LOG_WARN(module, msg)                                                                      \
    Radar::Logger::Logger::LOG<Radar::Logger::LogConfig::LogLevel::WARNING>(module, msg)
#define LOG_ERROR(module, msg)                                                                     \
    Radar::Logger::Logger::LOG<Radar::Logger::LogConfig::LogLevel::ERROR>(module, msg)

#ifdef NDEBUG
#define LOG_DEBUG(module, msg) ((void)0)
#else
#define LOG_DEBUG(module, msg)                                                                     \
    Radar::Logger::Logger::LOG<Radar::Logger::LogConfig::LogLevel::DEBUG>(module, msg)
#endif
