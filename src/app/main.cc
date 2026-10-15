// AeroLab entry point: parses command-line options and runs the application.

#include <SDL3/SDL_main.h>

#include <cstdio>
#include <memory>
#include <string_view>
#include <vector>

#include "app/application.h"
#include "app/options.h"

int main(int argc, char** argv) {
  const std::vector<std::string_view> args(argv + 1, argv + argc);
  const aerolab::ParseResult parsed = aerolab::ParseCommandLine(args);
  const std::string_view usage = aerolab::UsageText();
  switch (parsed.status) {
    case aerolab::ParseStatus::kHelp:
      std::fwrite(usage.data(), 1, usage.size(), stdout);
      return 0;
    case aerolab::ParseStatus::kError:
      std::fprintf(stderr, "aerolab: %s\n", parsed.error.c_str());
      std::fwrite(usage.data(), 1, usage.size(), stderr);
      return 2;
    case aerolab::ParseStatus::kRun:
      break;
  }

  std::unique_ptr<aerolab::Application> app =
      aerolab::Application::Create(parsed.options);
  if (app == nullptr) return 1;
  return app->Run();
}
