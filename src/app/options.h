#ifndef AEROLAB_APP_OPTIONS_H_
#define AEROLAB_APP_OPTIONS_H_

#include <span>
#include <string>
#include <string_view>

namespace aerolab {

struct AppOptions {
  // Number of frames to run before exiting; zero runs until the window closes.
  int max_frames = 0;
  // When non-empty, the final frame is saved to this PNG path before exit.
  std::string screenshot_path;
  bool vsync = true;
  bool hidden_window = false;
};

enum class ParseStatus { kRun, kHelp, kError };

struct ParseResult {
  ParseStatus status = ParseStatus::kRun;
  AppOptions options;
  // Explanation for kError; empty otherwise.
  std::string error;
};

// Parses command-line arguments, excluding the program name.
ParseResult ParseCommandLine(std::span<const std::string_view> args);

std::string_view UsageText();

}  // namespace aerolab

#endif  // AEROLAB_APP_OPTIONS_H_
