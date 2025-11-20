#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/channel_layout.h>
#include <libavutil/samplefmt.h>
}

#include "internal.hpp"
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace {

ffmpeg_cpp::CodecParameters make_audio_parameters(const int sample_rate) {
  auto params = ffmpeg_cpp::CodecParameters::create();
  params->codec_type = AVMEDIA_TYPE_AUDIO;
  params->codec_id = AV_CODEC_ID_PCM_S16LE;
  params->format = AV_SAMPLE_FMT_S16;
  params->sample_rate = sample_rate;
  av_channel_layout_default(&params->ch_layout, 1);
  return params;
}

ffmpeg_cpp::Output make_output(const ffmpeg_cpp::CodecParameters &params, const std::string &format,
                               const std::string &path) {
  return ffmpeg_cpp::Output::create(format, path, params);
}

ffmpeg_cpp::Packet make_packet(const int stream_index, const std::uint8_t fill_value,
                               const std::size_t size) {
  std::vector<std::uint8_t> payload(size, fill_value);
  auto packet = ffmpeg_cpp::Packet::create(payload.data(), payload.size());
  packet->stream_index = stream_index;
  return packet;
}

} // namespace

TEST(OutputTest, NullOutputIsFalsy) {
  const auto output = ffmpeg_cpp::Output::null();
  EXPECT_FALSE(output);
}

TEST(OutputTest, CreateSetsNonBlockingFlagAndTimeBase) {
  constexpr int sample_rate = 48'000;
  const auto params = make_audio_parameters(sample_rate);
  const auto output = make_output(params, "null", "/dev/null");

  ASSERT_TRUE(output);
  EXPECT_NE(0, output->flags & AVFMT_FLAG_NONBLOCK);
  ASSERT_NE(nullptr, output->streams[0]);
  EXPECT_EQ(sample_rate, output->streams[0]->time_base.den);
}

TEST(OutputTest, SurplusOptionsAreRejected) {
  const auto params = make_audio_parameters(48'000);
  EXPECT_THROW(
      static_cast<void>(ffmpeg_cpp::Output::create(
          "null", "/dev/null", params, ffmpeg_cpp::Dictionary::create("unexpected: value"))),
      ffmpeg_cpp::Error);
}

TEST(OutputTest, WriteFrameSetsStreamIndexAndAcceptsSequentialWrites) {
  const auto params = make_audio_parameters(48'000);
  auto output = make_output(params, "null", "/dev/null");
  ASSERT_TRUE(output);
  ASSERT_NE(nullptr, output->streams[0]);

  const int stream_index = output->streams[0]->index;

  auto packet1 = make_packet(stream_index + 5, 0xAA, 4);

  bool first_written = false;
  ASSERT_NO_THROW({ first_written = output.write_frame(packet1); });
  EXPECT_TRUE(first_written);

  auto packet2 = make_packet(stream_index + 7, 0xBB, 4);

  bool second_written = false;
  ASSERT_NO_THROW({ second_written = output.write_frame(packet2); });
  EXPECT_TRUE(second_written);
}
