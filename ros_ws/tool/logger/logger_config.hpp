#include <chrono>
#include <string>
using system_clock = std::chrono::system_clock;
using milliseconds = std::chrono::milliseconds;
namespace Radar::Logger::LogConfig {
enum class LogLevel { INFO, WARNING, ERROR };
struct LogData {
    std::string timestamp = []() {
        auto now        = std::chrono::time_point_cast<milliseconds>(system_clock::now());
        auto time_t_now = system_clock::to_time_t(now);
        auto local      = std::chrono::zoned_time { std::chrono::current_zone(), now };
        return std::format("{:%H:%M:%S}", local);
    }();
    LogLevel level = LogLevel::INFO;
    std::string module_name;
    std::string message;
};
}