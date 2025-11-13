#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <vector>

extern "C" {
#include <libavutil/channel_layout.h>
#include <libavutil/imgutils.h>
#include <libavutil/samplefmt.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include "internal.hpp"

namespace {

std::vector<std::uint8_t> make_pattern(const std::size_t size) {
  std::vector<std::uint8_t> data(size);
  std::iota(data.begin(), data.end(), std::uint8_t{0});
  return data;
}

ffmpeg_pipeline_msgs::msg::Frame make_video_message(const int width, const int height,
                                                     const AVPixelFormat pixel_format) {
  ffmpeg_pipeline_msgs::msg::Frame msg;
  msg.pts = 42;
  msg.pkt_dts = 41;
  msg.time_base.num = 1;
  msg.time_base.den = 90'000;
  msg.width = width;
  msg.height = height;
  msg.format = ffmpeg_cpp::to_string(pixel_format);

  const int linesize = av_image_get_linesize(pixel_format, width, 0);
  EXPECT_GT(linesize, 0);
  msg.data = make_pattern(static_cast<std::size_t>(linesize) * height);

  return msg;
}

ffmpeg_pipeline_msgs::msg::Frame make_audio_message(const int nb_samples, const int sample_rate,
                                                     const AVSampleFormat sample_format,
                                                     const std::string &layout_str) {
  ffmpeg_pipeline_msgs::msg::Frame msg;
  msg.pts = 7;
  msg.pkt_dts = 5;
  msg.time_base.num = 1;
  msg.time_base.den = sample_rate;
  msg.nb_samples = nb_samples;
  msg.sample_rate = sample_rate;
  msg.ch_layout = layout_str;
  msg.format = ffmpeg_cpp::to_string(sample_format);

  AVChannelLayout layout{};
  const int layout_result = av_channel_layout_from_string(&layout, layout_str.c_str());
  EXPECT_GE(layout_result, 0);
  const int channels = layout.nb_channels;
  av_channel_layout_uninit(&layout);

  const int data_size = av_samples_get_buffer_size(nullptr, channels, nb_samples, sample_format, 1);
  EXPECT_GT(data_size, 0);
  msg.data = make_pattern(static_cast<std::size_t>(data_size));

  return msg;
}

sensor_msgs::msg::Image make_image_message(const int width, const int height,
                                            const std::string &encoding) {
  sensor_msgs::msg::Image msg;
  msg.width = width;
  msg.height = height;
  msg.encoding = encoding;
  msg.step = width * 3; // rgb8-like encoding
  msg.data = make_pattern(static_cast<std::size_t>(msg.step) * height);
  return msg;
}

} // namespace

TEST(FrameTest, NullFrameIsEmpty) {
  const auto frame = ffmpeg_cpp::Frame::null();
  EXPECT_FALSE(frame);
  EXPECT_TRUE(frame.empty());
}

TEST(FrameTest, CreateFrameDefaultsToEmpty) {
  const auto frame = ffmpeg_cpp::Frame::create();
  ASSERT_TRUE(frame);
  EXPECT_TRUE(frame.empty());
  EXPECT_EQ(nullptr, frame->data[0]);
}

TEST(FrameTest, CreateFromRawBufferCopiesData) {
  const auto source = make_pattern(16);
  const auto frame = ffmpeg_cpp::Frame::create(source.data(), source.size());

  ASSERT_TRUE(frame);
  ASSERT_NE(nullptr, frame->data[0]);
  EXPECT_NE(static_cast<const void *>(source.data()), static_cast<const void *>(frame->data[0]));
  EXPECT_EQ(source.size(), frame->buf[0]->size);
  EXPECT_EQ(0, std::memcmp(source.data(), frame->data[0], source.size()));
  EXPECT_TRUE(frame.empty());
}

TEST(FrameTest, CreateFromVideoMessagePopulatesVideoFields) {
  const int width = 4;
  const int height = 3;
  const AVPixelFormat pixel_format = AV_PIX_FMT_RGB24;
  const auto msg = make_video_message(width, height, pixel_format);

  const auto frame = ffmpeg_cpp::Frame::create(msg);
  ASSERT_TRUE(frame);

  EXPECT_FALSE(frame.empty());
  EXPECT_EQ(width, frame->width);
  EXPECT_EQ(height, frame->height);
  EXPECT_EQ(pixel_format, static_cast<AVPixelFormat>(frame->format));
  const int expected_linesize = av_image_get_linesize(pixel_format, width, 0);
  EXPECT_EQ(expected_linesize, frame->linesize[0]);
  EXPECT_EQ(msg.data.size(), static_cast<std::size_t>(frame->linesize[0] * height));
  EXPECT_EQ("rgb24", frame.format_name());
  EXPECT_TRUE(frame.ch_layout_str().empty());
  ASSERT_NE(nullptr, frame->data[0]);
  EXPECT_NE(static_cast<const void *>(msg.data.data()), static_cast<const void *>(frame->data[0]));
  EXPECT_EQ(0, std::memcmp(msg.data.data(), frame->data[0], msg.data.size()));
}

TEST(FrameTest, CreateFromAudioMessagePopulatesAudioFields) {
  const int nb_samples = 6;
  const int sample_rate = 48'000;
  const AVSampleFormat sample_format = AV_SAMPLE_FMT_S16;
  const std::string layout = "stereo";
  const auto msg = make_audio_message(nb_samples, sample_rate, sample_format, layout);

  const auto frame = ffmpeg_cpp::Frame::create(msg);
  ASSERT_TRUE(frame);

  EXPECT_FALSE(frame.empty());
  EXPECT_EQ(0, frame->width);
  EXPECT_EQ(0, frame->height);
  EXPECT_EQ(nb_samples, frame->nb_samples);
  EXPECT_EQ(sample_rate, frame->sample_rate);
  EXPECT_EQ(sample_format, static_cast<AVSampleFormat>(frame->format));
  EXPECT_EQ(layout, frame.ch_layout_str());
  EXPECT_EQ("s16", frame.format_name());
  const int expected_linesize = av_samples_get_buffer_size(nullptr, frame->ch_layout.nb_channels,
                                                          nb_samples, sample_format, 1);
  EXPECT_EQ(expected_linesize, frame->linesize[0]);
  ASSERT_NE(nullptr, frame->data[0]);
  EXPECT_NE(static_cast<const void *>(msg.data.data()), static_cast<const void *>(frame->data[0]));
  EXPECT_EQ(0, std::memcmp(msg.data.data(), frame->data[0], msg.data.size()));
}

TEST(FrameTest, CreateFromImageMessageUsesEncodingProperties) {
  const int width = 3;
  const int height = 2;
  const auto image = make_image_message(width, height, "rgb8");

  const auto frame = ffmpeg_cpp::Frame::create(image);
  ASSERT_TRUE(frame);

  EXPECT_FALSE(frame.empty());
  EXPECT_EQ(width, frame->width);
  EXPECT_EQ(height, frame->height);
  EXPECT_EQ(image.step, frame->linesize[0]);
  EXPECT_NE(AV_PIX_FMT_NONE, static_cast<AVPixelFormat>(frame->format));
  ASSERT_NE(nullptr, frame->data[0]);
  EXPECT_NE(static_cast<const void *>(image.data.data()), static_cast<const void *>(frame->data[0]));
  EXPECT_EQ(0, std::memcmp(image.data.data(), frame->data[0], image.data.size()));
}

TEST(FrameTest, CopyConstructorClonesUnderlyingFrame) {
  const auto data = make_pattern(8);
  auto original = ffmpeg_cpp::Frame::create(data.data(), data.size());
  ASSERT_TRUE(original);

  const ffmpeg_cpp::Frame copy(original);
  ASSERT_TRUE(copy);

  EXPECT_NE(original.get(), copy.get());
  EXPECT_EQ(0, std::memcmp(original->data[0], copy->data[0], 8));
}

TEST(FrameTest, MoveOperationsTransferOwnershipAndClearSource) {
  const auto original_data = make_pattern(4);
  auto original = ffmpeg_cpp::Frame::create(original_data.data(), original_data.size());
  ASSERT_TRUE(original);

  auto moved = ffmpeg_cpp::Frame(std::move(original));
  EXPECT_TRUE(moved);
  EXPECT_FALSE(original);

  const auto second_data = make_pattern(4);
  auto second = ffmpeg_cpp::Frame::create(second_data.data(), second_data.size());
  ASSERT_TRUE(second);

  moved = std::move(second);
  EXPECT_TRUE(moved);
  EXPECT_FALSE(second);

  moved = std::move(moved); // self-move should be harmless
  EXPECT_TRUE(moved);
}

TEST(FrameTest, ToFrameMsgRoundTripsVideoMetadataAndPayload) {
  const int width = 5;
  const int height = 4;
  const auto msg = make_video_message(width, height, AV_PIX_FMT_RGB24);
  auto frame = ffmpeg_cpp::Frame::create(msg);
  ASSERT_TRUE(frame);

  const rclcpp::Time stamp(1, 2);
  const auto roundtrip = frame.to_frame_msg(stamp);

  EXPECT_EQ(stamp.nanoseconds(), rclcpp::Time(roundtrip.header.stamp).nanoseconds());
  EXPECT_EQ(msg.pts, roundtrip.pts);
  EXPECT_EQ(msg.pkt_dts, roundtrip.pkt_dts);
  EXPECT_EQ(msg.time_base.num, roundtrip.time_base.num);
  EXPECT_EQ(msg.time_base.den, roundtrip.time_base.den);
  EXPECT_EQ(width, roundtrip.width);
  EXPECT_EQ(height, roundtrip.height);
  EXPECT_EQ("rgb24", roundtrip.format);
  EXPECT_TRUE(roundtrip.ch_layout.empty());
  EXPECT_EQ(msg.data, roundtrip.data);
}

TEST(FrameTest, ToFrameMsgRoundTripsAudioMetadataAndPayload) {
  const auto msg = make_audio_message(8, 44'100, AV_SAMPLE_FMT_FLTP, "mono");
  auto frame = ffmpeg_cpp::Frame::create(msg);
  ASSERT_TRUE(frame);

  const rclcpp::Time stamp(3, 4);
  const auto roundtrip = frame.to_frame_msg(stamp);

  EXPECT_EQ(stamp.nanoseconds(), rclcpp::Time(roundtrip.header.stamp).nanoseconds());
  EXPECT_EQ(msg.nb_samples, roundtrip.nb_samples);
  EXPECT_EQ(msg.sample_rate, roundtrip.sample_rate);
  EXPECT_EQ("fltp", roundtrip.format);
  EXPECT_EQ("mono", roundtrip.ch_layout);
  EXPECT_EQ(msg.data, roundtrip.data);
}

TEST(FrameTest, ToImageMsgExportsDimensionsAndPayload) {
  const auto msg = make_video_message(6, 2, AV_PIX_FMT_RGB24);
  auto frame = ffmpeg_cpp::Frame::create(msg);
  ASSERT_TRUE(frame);

  const rclcpp::Time stamp(9, 10);
  const auto image = frame.to_image_msg(stamp, "rgb8");

  EXPECT_EQ(stamp.nanoseconds(), rclcpp::Time(image.header.stamp).nanoseconds());
  EXPECT_EQ(2u, image.height);
  EXPECT_EQ(6u, image.width);
  EXPECT_EQ("rgb8", image.encoding);
  EXPECT_EQ(static_cast<std::size_t>(image.step) * image.height, image.data.size());
  EXPECT_EQ(msg.data, image.data);
}

