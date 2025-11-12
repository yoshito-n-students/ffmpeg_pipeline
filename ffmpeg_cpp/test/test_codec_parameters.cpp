#include <gtest/gtest.h>

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

// Ensures the null wrapper surfaces empty codec metadata.
TEST(CodecParametersTest, NullCodecParametersExposeEmptyMetadata) {
  const auto params = ffmpeg_cpp::CodecParameters::null();
  EXPECT_FALSE(params);
  EXPECT_EQ("", params.codec_type_name());
  EXPECT_EQ("", params.codec_name());
  EXPECT_EQ("", params.format_name());
  EXPECT_EQ("", params.ch_layout_str());
}

// Ensures create() yields an allocated wrapper with default metadata.
TEST(CodecParametersTest, CreateReturnsAllocatedWrapper) {
  const auto params = ffmpeg_cpp::CodecParameters::create();
  EXPECT_TRUE(params);
  EXPECT_EQ("none", params.codec_name());
}

// Ensures video-centric YAML populates the expected video fields.
TEST(CodecParametersTest, VideoYamlPopulatesVideoFields) {
  const std::string yaml = R"(codec: h264
format: yuv420p
width: 1920
height: 1080
framerate: [30000, 1001]
)";

  const auto params = ffmpeg_cpp::CodecParameters::create(yaml);
  ASSERT_TRUE(params);
  EXPECT_EQ("video", params.codec_type_name());
  EXPECT_EQ("h264", params.codec_name());
  EXPECT_EQ("yuv420p", params.format_name());
}

// Ensures audio-centric YAML populates the expected audio fields.
TEST(CodecParametersTest, AudioYamlPopulatesAudioFields) {
  const std::string yaml = R"(codec: pcm_s16le
format: s16
ch_layout: mono
sample_rate: 44100
)";

  const auto params = ffmpeg_cpp::CodecParameters::create(yaml);
  ASSERT_TRUE(params);
  EXPECT_EQ("audio", params.codec_type_name());
  EXPECT_EQ("pcm_s16le", params.codec_name());
  EXPECT_EQ("s16", params.format_name());
  EXPECT_EQ("mono", params.ch_layout_str());
}

// Ensures subtitle codecs ignore format metadata that should not apply.
TEST(CodecParametersTest, SubtitleFormatIsIgnored) {
  const std::string yaml = R"(codec: webvtt
format: text
)";

  const auto params = ffmpeg_cpp::CodecParameters::create(yaml);
  ASSERT_TRUE(params);
  EXPECT_EQ("subtitle", params.codec_type_name());
  EXPECT_EQ("webvtt", params.codec_name());
  EXPECT_EQ("", params.format_name());
}

// Ensures flow-style YAML input is accepted and populates dimensions.
TEST(CodecParametersTest, FlowStyleYamlIsSupported) {
  const std::string yaml = R"({codec: h264, format: yuv420p, width: 640, height: 360})";

  const auto params = ffmpeg_cpp::CodecParameters::create(yaml);
  ASSERT_TRUE(params);
  EXPECT_EQ("video", params.codec_type_name());
  EXPECT_EQ("h264", params.codec_name());
  EXPECT_EQ("yuv420p", params.format_name());
  EXPECT_EQ(640, params->width);
  EXPECT_EQ(360, params->height);
}

// Ensures the copy constructor creates an independent clone of the parameters.
TEST(CodecParametersTest, CopyConstructorCreatesIndependentClone) {
  auto original = ffmpeg_cpp::CodecParameters::create(R"(codec: pcm_s16le
ch_layout: mono
)");
  ASSERT_TRUE(original);
  EXPECT_EQ("pcm_s16le", original.codec_name());

  const ffmpeg_cpp::CodecParameters copy(original);
  ASSERT_TRUE(copy);
  EXPECT_EQ(original.codec_name(), copy.codec_name());
  EXPECT_EQ(original.ch_layout_str(), copy.ch_layout_str());

  original = ffmpeg_cpp::CodecParameters::null();
  EXPECT_FALSE(original);
  EXPECT_EQ("pcm_s16le", copy.codec_name());
  EXPECT_EQ("mono", copy.ch_layout_str());
}

// Ensures the move constructor transfers ownership and clears the source wrapper.
TEST(CodecParametersTest, MoveConstructorTransfersOwnership) {
  auto original = ffmpeg_cpp::CodecParameters::create(R"(codec: pcm_s16le
ch_layout: mono
)");
  ASSERT_TRUE(original);

  const auto moved = ffmpeg_cpp::CodecParameters(std::move(original));
  EXPECT_TRUE(moved);
  EXPECT_FALSE(original);
  EXPECT_EQ("pcm_s16le", moved.codec_name());
  EXPECT_EQ("mono", moved.ch_layout_str());
  EXPECT_EQ("", original.codec_name());
  EXPECT_EQ("", original.ch_layout_str());
}

// Ensures move assignment transfers metadata, resets the source, and survives self-move.
TEST(CodecParametersTest, MoveAssignmentTransfersAndClearsSource) {
  auto source = ffmpeg_cpp::CodecParameters::create(R"(codec: pcm_s16le
ch_layout: mono
)");
  auto destination = ffmpeg_cpp::CodecParameters::create(R"(codec: h264
format: yuv420p
)");

  destination = std::move(source);
  EXPECT_TRUE(destination);
  EXPECT_FALSE(source);
  EXPECT_EQ("pcm_s16le", destination.codec_name());
  EXPECT_EQ("mono", destination.ch_layout_str());
  EXPECT_EQ("", source.codec_name());

  auto self_move = ffmpeg_cpp::CodecParameters::create(R"(codec: h264
format: yuv420p
)");
  const std::string expected_codec = self_move.codec_name();
  const std::string expected_format = self_move.format_name();
  self_move = std::move(self_move);
  EXPECT_TRUE(self_move);
  EXPECT_EQ(expected_codec, self_move.codec_name());
  EXPECT_EQ(expected_format, self_move.format_name());
}

// Ensures invalid codec names trigger ffmpeg_cpp::Error exceptions.
TEST(CodecParametersTest, CreateWithInvalidCodecThrows) {
  EXPECT_THROW(ffmpeg_cpp::CodecParameters::create(R"(codec: definitely_not_real
)"),
               ffmpeg_cpp::Error);
}

// Ensures invalid channel layouts trigger ffmpeg_cpp::Error exceptions.
TEST(CodecParametersTest, CreateWithInvalidChannelLayoutThrows) {
  const std::string yaml = R"(codec: pcm_s16le
ch_layout: not_a_layout
)";
  EXPECT_THROW(ffmpeg_cpp::CodecParameters::create(yaml), ffmpeg_cpp::Error);
}
