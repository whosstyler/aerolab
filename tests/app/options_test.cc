#include "app/options.h"

#include <gtest/gtest.h>

#include <string_view>
#include <vector>

namespace aerolab {
namespace {

ParseResult Parse(std::vector<std::string_view> args) {
  return ParseCommandLine(args);
}

TEST(OptionsTest, DefaultsWithNoArguments) {
  const ParseResult result = Parse({});
  EXPECT_EQ(result.status, ParseStatus::kRun);
  EXPECT_EQ(result.options.max_frames, 0);
  EXPECT_TRUE(result.options.vsync);
  EXPECT_FALSE(result.options.hidden_window);
  EXPECT_TRUE(result.options.screenshot_path.empty());
}

TEST(OptionsTest, ParsesAllFlags) {
  const ParseResult result = Parse(
      {"--frames", "12", "--screenshot", "out.png", "--no-vsync", "--hidden"});
  ASSERT_EQ(result.status, ParseStatus::kRun);
  EXPECT_EQ(result.options.max_frames, 12);
  EXPECT_EQ(result.options.screenshot_path, "out.png");
  EXPECT_FALSE(result.options.vsync);
  EXPECT_TRUE(result.options.hidden_window);
}

TEST(OptionsTest, HelpIsNotAnError) {
  EXPECT_EQ(Parse({"--help"}).status, ParseStatus::kHelp);
  EXPECT_EQ(Parse({"--frames", "3", "-h"}).status, ParseStatus::kHelp);
  EXPECT_FALSE(UsageText().empty());
}

TEST(OptionsTest, RejectsInvalidFrameCounts) {
  for (std::string_view bad : {"0", "-4", "abc", "12x", "", "99999999999"}) {
    EXPECT_EQ(Parse({"--frames", bad}).status, ParseStatus::kError) << bad;
  }
  EXPECT_EQ(Parse({"--frames"}).status, ParseStatus::kError);
}

TEST(OptionsTest, ScreenshotNeedsPathAndFrameLimit) {
  EXPECT_EQ(Parse({"--screenshot"}).status, ParseStatus::kError);
  EXPECT_EQ(Parse({"--frames", "2", "--screenshot", ""}).status,
            ParseStatus::kError);
  const ParseResult no_frames = Parse({"--screenshot", "a.png"});
  EXPECT_EQ(no_frames.status, ParseStatus::kError);
  EXPECT_FALSE(no_frames.error.empty());
}

TEST(OptionsTest, RejectsUnknownOptions) {
  const ParseResult result = Parse({"--fly-fast"});
  EXPECT_EQ(result.status, ParseStatus::kError);
  EXPECT_NE(result.error.find("--fly-fast"), std::string::npos);
}

}  // namespace
}  // namespace aerolab
