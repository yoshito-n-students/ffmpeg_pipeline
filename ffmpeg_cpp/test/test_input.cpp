#include <gtest/gtest.h>

#include <string>

extern "C" {
#include <libavformat/avformat.h>
}

#include "internal.hpp"
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

TEST(InputTest, NullInputIsFalsy) {
  const auto input = ffmpeg_cpp::Input::null();
  EXPECT_FALSE(input);
}

TEST(InputTest, CreateLavfiSetsNonBlockingFlag) {
  const auto input = ffmpeg_cpp::Input::create("testsrc=size=16x16:rate=1:duration=1", "lavfi",
                                               ffmpeg_cpp::Dictionary::null(), "video");
  ASSERT_TRUE(input);
  EXPECT_NE(0, input->flags & AVFMT_FLAG_NONBLOCK);
}

TEST(InputTest, UnknownFormatThrows) {
  EXPECT_THROW(static_cast<void>(ffmpeg_cpp::Input::create("testsrc=duration=1", "no_such_format")),
               ffmpeg_cpp::Error);
}

TEST(InputTest, UnknownMediaTypeThrows) {
  EXPECT_THROW(static_cast<void>(ffmpeg_cpp::Input::create(
                   "testsrc=duration=1", "lavfi", ffmpeg_cpp::Dictionary::null(), "not_a_type")),
               ffmpeg_cpp::Error);
}

TEST(InputTest, MediaTypeSelectsVideoStreamParameters) {
  auto input = ffmpeg_cpp::Input::create("testsrc=size=16x16:rate=1:duration=1", "lavfi",
                                         ffmpeg_cpp::Dictionary::null(), "video");

  ASSERT_TRUE(input);
  const auto params = input.codec_parameters();
  ASSERT_TRUE(params);
  EXPECT_EQ(AVMEDIA_TYPE_VIDEO, params->codec_type);
  EXPECT_EQ(16, params->width);
  EXPECT_EQ(16, params->height);
}

TEST(InputTest, SurplusOptionsAreRejected) {
  EXPECT_THROW(static_cast<void>(ffmpeg_cpp::Input::create(
                   "testsrc=duration=1", "lavfi",
                   ffmpeg_cpp::Dictionary::create("invalid_option: value"), "video")),
               ffmpeg_cpp::Error);
}

TEST(InputTest, CodecParametersCopiedFromStream) {
  const auto input = ffmpeg_cpp::Input::create("testsrc=size=32x24:rate=1:duration=1", "lavfi",
                                               ffmpeg_cpp::Dictionary::null(), "video");
  const auto params = input.codec_parameters();

  ASSERT_TRUE(params);
  EXPECT_EQ(AVMEDIA_TYPE_VIDEO, params->codec_type);
  EXPECT_EQ(32, params->width);
  EXPECT_EQ(24, params->height);
}

TEST(InputTest, ReadFrameReturnsPacketWithoutBlocking) {
  auto input = ffmpeg_cpp::Input::create("testsrc=size=8x8:rate=1:duration=1", "lavfi",
                                         ffmpeg_cpp::Dictionary::null(), "video");

  bool received_packet = false;
  for (int attempt = 0; attempt < 5 && !received_packet; ++attempt) {
    try {
      auto packet = input.read_frame();
      if (!packet) {
        continue;
      }
      received_packet = true;
      EXPECT_FALSE(packet.empty());
      EXPECT_EQ(input->streams[packet->stream_index]->index, packet->stream_index);
    } catch (const ffmpeg_cpp::Error &error) {
      FAIL() << "Unexpected exception while reading frame: " << error.what();
    }
  }

  EXPECT_TRUE(received_packet);
}
