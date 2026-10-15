#ifndef AEROLAB_CORE_LOG_H_
#define AEROLAB_CORE_LOG_H_

#include <format>
#include <functional>
#include <string_view>
#include <utility>

namespace aerolab {

enum class LogLevel { kDebug, kInfo, kWarning, kError };

std::string_view LogLevelName(LogLevel level);

// Receives every message after it is written to stderr. The editor installs a
// sink to mirror messages into its event log. A sink must not log; messages
// logged from inside a sink reach stderr only.
using LogSink = std::function<void(LogLevel level, std::string_view message)>;

// Replaces the active sink. Pass an empty function to remove it. Thread-safe.
void SetLogSink(LogSink sink);

void LogMessage(LogLevel level, std::string_view message);

template <typename... Args>
void LogDebug(std::format_string<Args...> format, Args&&... args) {
  LogMessage(LogLevel::kDebug,
             std::format(format, std::forward<Args>(args)...));
}

template <typename... Args>
void LogInfo(std::format_string<Args...> format, Args&&... args) {
  LogMessage(LogLevel::kInfo, std::format(format, std::forward<Args>(args)...));
}

template <typename... Args>
void LogWarning(std::format_string<Args...> format, Args&&... args) {
  LogMessage(LogLevel::kWarning,
             std::format(format, std::forward<Args>(args)...));
}

template <typename... Args>
void LogError(std::format_string<Args...> format, Args&&... args) {
  LogMessage(LogLevel::kError,
             std::format(format, std::forward<Args>(args)...));
}

}  // namespace aerolab

#endif  // AEROLAB_CORE_LOG_H_
