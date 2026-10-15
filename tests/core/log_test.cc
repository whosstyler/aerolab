#include "core/log.h"

#include <gtest/gtest.h>

#include <string>
#include <string_view>
#include <vector>

namespace aerolab {
namespace {

struct Captured {
  LogLevel level;
  std::string message;
};

class LogTest : public ::testing::Test {
 protected:
  void SetUp() override {
    SetLogSink([this](LogLevel level, std::string_view message) {
      captured_.push_back({level, std::string(message)});
    });
  }
  void TearDown() override { SetLogSink(nullptr); }

  std::vector<Captured> captured_;
};

TEST_F(LogTest, SinkReceivesFormattedMessagesWithLevels) {
  LogInfo("value={} name={}", 42, "dart");
  LogWarning("careful");
  LogError("broken {}", 1.5);
  LogDebug("trace");
  ASSERT_EQ(captured_.size(), 4u);
  EXPECT_EQ(captured_[0].level, LogLevel::kInfo);
  EXPECT_EQ(captured_[0].message, "value=42 name=dart");
  EXPECT_EQ(captured_[1].level, LogLevel::kWarning);
  EXPECT_EQ(captured_[2].message, "broken 1.5");
  EXPECT_EQ(captured_[3].level, LogLevel::kDebug);
}

TEST_F(LogTest, RemovingSinkStopsDelivery) {
  SetLogSink(nullptr);
  LogInfo("not captured");
  EXPECT_TRUE(captured_.empty());
}

TEST_F(LogTest, SinkThatLogsDoesNotRecurse) {
  int calls = 0;
  SetLogSink([&calls](LogLevel, std::string_view) {
    ++calls;
    LogInfo("from inside the sink");
  });
  LogInfo("outer");
  EXPECT_EQ(calls, 1);
}

TEST(LogLevelTest, Names) {
  EXPECT_EQ(LogLevelName(LogLevel::kDebug), "debug");
  EXPECT_EQ(LogLevelName(LogLevel::kInfo), "info");
  EXPECT_EQ(LogLevelName(LogLevel::kWarning), "warning");
  EXPECT_EQ(LogLevelName(LogLevel::kError), "error");
}

}  // namespace
}  // namespace aerolab
