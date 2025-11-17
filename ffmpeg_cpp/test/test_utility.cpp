#include <gtest/gtest.h>

#include <array>
#include <cerrno>
#include <string>

extern "C" {
#include <libavutil/error.h>
#include <libavutil/log.h>
#include <libavutil/pixdesc.h>
}

#include <sensor_msgs/image_encodings.hpp>

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace {

struct LogLevelParam {
  const char *name;
  int level;
};

constexpr std::array<LogLevelParam, 9> kLogLevelParams = {{{"quiet", AV_LOG_QUIET},
                                                           {"panic", AV_LOG_PANIC},
                                                           {"fatal", AV_LOG_FATAL},
                                                           {"error", AV_LOG_ERROR},
                                                           {"warning", AV_LOG_WARNING},
                                                           {"info", AV_LOG_INFO},
                                                           {"verbose", AV_LOG_VERBOSE},
                                                           {"debug", AV_LOG_DEBUG},
                                                           {"trace", AV_LOG_TRACE}}};

class SetLogLevelTest : public ::testing::TestWithParam<LogLevelParam> {
protected:
  void SetUp() override { original_level_ = av_log_get_level(); }
  void TearDown() override { av_log_set_level(original_level_); }

private:
  int original_level_{};
};

} // namespace

TEST_P(SetLogLevelTest, AppliesRequestedLevel) {
  const auto param = GetParam();

  ffmpeg_cpp::set_log_level(param.name);

  EXPECT_EQ(param.level, av_log_get_level());
}

INSTANTIATE_TEST_SUITE_P(AllLogLevels, SetLogLevelTest, ::testing::ValuesIn(kLogLevelParams));

class SetLogLevelInvalidTest : public ::testing::Test {
protected:
  void SetUp() override { original_level_ = av_log_get_level(); }
  void TearDown() override { av_log_set_level(original_level_); }

private:
  int original_level_{};
};

TEST_F(SetLogLevelInvalidTest, ThrowsOnUnknownLevel) {
  const int original_level = av_log_get_level();
  EXPECT_THROW(ffmpeg_cpp::set_log_level("not-a-level"), ffmpeg_cpp::Error);
  EXPECT_EQ(original_level, av_log_get_level());
}

TEST(Err2StrTest, ReturnsExpectedMessageForKnownErrors) {
  struct {
    int errnum;
  } constexpr error_messages[] = {{AVERROR(EINVAL)}, {AVERROR(EIO)}};

  for (const auto &param : error_messages) {
    EXPECT_FALSE(ffmpeg_cpp::err2str(param.errnum).empty());
  }
}

TEST(Err2StrTest, HandlesSuccessCode) {
  const auto result = ffmpeg_cpp::err2str(0);

  EXPECT_FALSE(result.empty());
}

TEST(Err2StrTest, HandlesUnknownErrorCode) {
  constexpr int kUnknownErrnum = -123456;
  const auto result = ffmpeg_cpp::err2str(kUnknownErrnum);

  EXPECT_FALSE(result.empty());
}

namespace {

struct FormatMappingParam {
  const char *ffmpeg_name;
  const char *ros_encoding;
};

constexpr std::array<FormatMappingParam, 18> kFormatMappings = {
    {{"rgb24", sensor_msgs::image_encodings::RGB8},
     {"rgba", sensor_msgs::image_encodings::RGBA8},
     {"rgb48", sensor_msgs::image_encodings::RGB16},
     {"rgba64", sensor_msgs::image_encodings::RGBA16},
     {"bgr24", sensor_msgs::image_encodings::BGR8},
     {"bgra", sensor_msgs::image_encodings::BGRA8},
     {"bgr48", sensor_msgs::image_encodings::BGR16},
     {"bgra64", sensor_msgs::image_encodings::BGRA16},
     {"gray8", sensor_msgs::image_encodings::MONO8},
     {"gray16", sensor_msgs::image_encodings::MONO16},
     {"bayer_rggb8", sensor_msgs::image_encodings::BAYER_RGGB8},
     {"bayer_rggb16", sensor_msgs::image_encodings::BAYER_RGGB16},
     {"bayer_bggr8", sensor_msgs::image_encodings::BAYER_BGGR8},
     {"bayer_bggr16", sensor_msgs::image_encodings::BAYER_BGGR16},
     {"bayer_grbg8", sensor_msgs::image_encodings::BAYER_GRBG8},
     {"bayer_grbg16", sensor_msgs::image_encodings::BAYER_GRBG16},
     {"bayer_gbrg8", sensor_msgs::image_encodings::BAYER_GBRG8},
     {"bayer_gbrg16", sensor_msgs::image_encodings::BAYER_GBRG16}}};

class ToRosEncodingTest : public ::testing::TestWithParam<FormatMappingParam> {};

} // namespace

TEST_P(ToRosEncodingTest, ConvertsKnownFormats) {
  const auto param = GetParam();

  EXPECT_EQ(param.ros_encoding, ffmpeg_cpp::to_ros_image_encoding(param.ffmpeg_name));
}

INSTANTIATE_TEST_SUITE_P(KnownMappings, ToRosEncodingTest, ::testing::ValuesIn(kFormatMappings));

TEST(ToRosEncodingTest, ReturnsEmptyStringForUnknownFormat) {
  EXPECT_TRUE(ffmpeg_cpp::to_ros_image_encoding("unknown_format").empty());
}

class ToFfmpegFormatNameTest : public ::testing::TestWithParam<FormatMappingParam> {};

TEST_P(ToFfmpegFormatNameTest, ConvertsKnownEncodings) {
  const auto param = GetParam();

  const auto format_name = ffmpeg_cpp::to_ffmpeg_format_name(param.ros_encoding);
  const auto expected_pix_fmt = av_get_pix_fmt(param.ffmpeg_name);
  ASSERT_NE(AV_PIX_FMT_NONE, expected_pix_fmt);

  ASSERT_FALSE(format_name.empty());
  EXPECT_EQ(expected_pix_fmt, av_get_pix_fmt(format_name.c_str()));
}

INSTANTIATE_TEST_SUITE_P(KnownMappings, ToFfmpegFormatNameTest,
                         ::testing::ValuesIn(kFormatMappings));

TEST(ToFfmpegFormatNameTest, ReturnsEmptyStringForUnknownEncoding) {
  EXPECT_TRUE(ffmpeg_cpp::to_ffmpeg_format_name("not/real").empty());
}

TEST(FormatMappingTest, RosEncodingRoundTripsThroughFfmpegName) {
  for (const auto &mapping : kFormatMappings) {
    const auto ffmpeg_name = ffmpeg_cpp::to_ffmpeg_format_name(mapping.ros_encoding);
    ASSERT_FALSE(ffmpeg_name.empty());
    EXPECT_EQ(mapping.ros_encoding, ffmpeg_cpp::to_ros_image_encoding(ffmpeg_name));
  }
}

