#pragma once

#include "logger_config.hpp"

#include <string>

namespace Radar::Logger {

class Logger final {
public:
    static void Init(const std::string& log_root);
    static void Shutdown();

    template <LogConfig::LogLevel level>
    static void LOG(const std::string& module_name, const std::string& message) {
        auto log_data =
            LogConfig::LogData { .level = level, .module_name = module_name, .message = message };
        (void)log_data;
    }

private:
    Logger() = delete;
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
