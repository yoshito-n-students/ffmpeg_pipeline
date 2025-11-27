#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <cstring>
#include <string>

extern "C" {
#include <libavformat/avformat.h>
}

#include "internal.hpp"
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace {

// This suite is intended to surface leaks while running under sanitizers or memcheck.
// CI environments without sanitizers can target the "asan" or "valgrind" CTest labels
// (e.g., `ctest -L valgrind --overwrite MemoryCheckCommandOptions="--leak-check=full"`
// or `ctest -L asan` with `-fsanitize=address`) to exercise this test more thoroughly.
class MemoryLeakTest : public ::testing::Test {
protected:
  void SetUp() override {
#if defined(__GLIBC__)
    // Prefer glib-friendly allocation strategies so leak detectors have full visibility.
    setenv("G_SLICE", "always-malloc", 0);
    setenv("G_DEBUG", "gc-friendly", 0);
#endif
    const char *lsan_env = std::getenv("LSAN_OPTIONS");
    if (lsan_env == nullptr || std::strstr(lsan_env, "detect_leaks") == nullptr) {
      setenv("LSAN_OPTIONS", "detect_leaks=1", 0);
    }
  }
};

} // namespace

TEST_F(MemoryLeakTest, DictionaryCreationDoesNotThrow) {
  EXPECT_NO_THROW({
    for (int i = 0; i < 32; ++i) {
      const auto dictionary = ffmpeg_cpp::Dictionary::create("foo: bar");
      ASSERT_TRUE(dictionary);
    }
  });
}

TEST_F(MemoryLeakTest, PacketAndFrameCreationDoesNotThrow) {
  EXPECT_NO_THROW({
    const std::array<uint8_t, 14> payload = {'p', 'a', 'c', 'k', 'e', 't', '_',
                                             'p', 'a', 'y', 'l', 'o', 'a', 'd'};
    for (int i = 0; i < 32; ++i) {
      const auto packet = ffmpeg_cpp::Packet::create(payload.data(), payload.size());
      ASSERT_TRUE(packet);
      const auto frame = ffmpeg_cpp::Frame::create(payload.data(), payload.size());
      ASSERT_TRUE(frame);
    }
  });
}

TEST_F(MemoryLeakTest, InputReadDoesNotThrow) {
  EXPECT_NO_THROW({
    for (int i = 0; i < 12; ++i) {
      auto input = ffmpeg_cpp::Input::create("testsrc=size=8x8:rate=1:duration=1", "lavfi",
                                             ffmpeg_cpp::Dictionary::null(), "video");
      ASSERT_TRUE(input);
      // Drain a few packets to exercise alloc/free paths repeatedly.
      for (int attempt = 0; attempt < 5; ++attempt) {
        (void)input.read_frame();
      }
    }
  });
}
