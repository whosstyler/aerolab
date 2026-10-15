#include "app/options.h"

#include <charconv>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace aerolab {
namespace {

constexpr std::string_view kUsage =
    "Usage: aerolab [options]\n"
    "  --frames N         Exit after N frames (smoke testing).\n"
    "  --screenshot PATH  Save the last frame as PNG (requires --frames).\n"
    "  --no-vsync         Disable vertical sync.\n"
    "  --hidden           Do not show the window (automated runs).\n"
    "  --select NAME      Select and follow the named object at startup.\n"
    "  --help             Show this message.\n";

std::optional<int> ParsePositiveInt(std::string_view text) {
  int value = 0;
  const auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc() || end != text.data() + text.size() || value <= 0) {
    return std::nullopt;
  }
  return value;
}

ParseResult Error(std::string message) {
  ParseResult result;
  result.status = ParseStatus::kError;
  result.error = std::move(message);
  return result;
}

}  // namespace

ParseResult ParseCommandLine(std::span<const std::string_view> args) {
  ParseResult result;
  AppOptions& options = result.options;
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string_view arg = args[i];
    const bool has_value = i + 1 < args.size();
    if (arg == "--help" || arg == "-h") {
      result.status = ParseStatus::kHelp;
      return result;
    } else if (arg == "--frames") {
      const std::optional<int> frames =
          has_value ? ParsePositiveInt(args[++i]) : std::nullopt;
      if (!frames.has_value()) {
        return Error("--frames expects a positive integer");
      }
      options.max_frames = *frames;
    } else if (arg == "--screenshot") {
      if (!has_value || args[i + 1].empty()) {
        return Error("--screenshot expects a file path");
      }
      options.screenshot_path = std::string(args[++i]);
    } else if (arg == "--select") {
      if (!has_value || args[i + 1].empty()) {
        return Error("--select expects an object name");
      }
      options.select_name = std::string(args[++i]);
    } else if (arg == "--no-vsync") {
      options.vsync = false;
    } else if (arg == "--hidden") {
      options.hidden_window = true;
    } else {
      return Error("unknown option '" + std::string(arg) + "'");
    }
  }
  if (!options.screenshot_path.empty() && options.max_frames == 0) {
    return Error("--screenshot requires --frames");
  }
  return result;
}

std::string_view UsageText() { return kUsage; }

}  // namespace aerolab
