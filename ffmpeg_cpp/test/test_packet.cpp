#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/packet.h>
}

#include "internal.hpp"
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace {

std::vector<std::uint8_t> make_pattern(const std::size_t size) {
  std::vector<std::uint8_t> data(size);
  for (std::size_t i = 0; i < size; ++i) {
    data[i] = static_cast<std::uint8_t>(i & 0xFF);
  }
  return data;
}

ffmpeg_pipeline_msgs::msg::Packet make_packet_message() {
  ffmpeg_pipeline_msgs::msg::Packet msg;
  msg.pts = 123;
  msg.dts = 45;
  msg.duration = 67;
  msg.time_base.num = 1;
  msg.time_base.den = 90000;
  msg.codec = "h264";
  msg.data = make_pattern(16);
  return msg;
}

sensor_msgs::msg::CompressedImage make_compressed_image(const std::string &format) {
  sensor_msgs::msg::CompressedImage msg;
  msg.format = format;
  msg.data = make_pattern(32);
  return msg;
}

} // namespace

TEST(PacketTest, NullPacketIsEmpty) {
  const auto packet = ffmpeg_cpp::Packet::null();
  EXPECT_FALSE(packet);
  EXPECT_TRUE(packet.empty());
}

TEST(PacketTest, CreatePacketDefaultsToEmpty) {
  const auto packet = ffmpeg_cpp::Packet::create();
  ASSERT_TRUE(packet);
  EXPECT_TRUE(packet.empty());
  EXPECT_EQ(nullptr, packet->data);
  EXPECT_EQ(0, packet->size);
}

TEST(PacketTest, CreateFromRawBufferCopiesData) {
  const auto source = make_pattern(24);
  const auto packet = ffmpeg_cpp::Packet::create(source.data(), source.size());

  ASSERT_TRUE(packet);
  ASSERT_NE(nullptr, packet->data);
  EXPECT_NE(static_cast<const void *>(source.data()), static_cast<const void *>(packet->data));
  EXPECT_EQ(static_cast<int>(source.size()), packet->size);
  EXPECT_EQ(0, std::memcmp(source.data(), packet->data, source.size()));
  EXPECT_FALSE(packet.empty());
}

TEST(PacketTest, CreateFromMessageCopiesMetadataAndPayload) {
  const auto msg = make_packet_message();
  const auto packet = ffmpeg_cpp::Packet::create(msg);

  ASSERT_TRUE(packet);
  EXPECT_FALSE(packet.empty());
  EXPECT_EQ(msg.pts, packet->pts);
  EXPECT_EQ(msg.dts, packet->dts);
  EXPECT_EQ(msg.duration, packet->duration);
  EXPECT_EQ(msg.time_base.num, packet->time_base.num);
  EXPECT_EQ(msg.time_base.den, packet->time_base.den);
  ASSERT_NE(nullptr, packet->data);
  EXPECT_NE(static_cast<const void *>(msg.data.data()), static_cast<const void *>(packet->data));
  EXPECT_EQ(static_cast<int>(msg.data.size()), packet->size);
  EXPECT_EQ(0, std::memcmp(msg.data.data(), packet->data, msg.data.size()));
}

TEST(PacketTest, CreateFromCompressedImageCopiesPayload) {
  const auto compressed = make_compressed_image("jpeg");
  const auto packet = ffmpeg_cpp::Packet::create(compressed);

  ASSERT_TRUE(packet);
  EXPECT_FALSE(packet.empty());
  ASSERT_NE(nullptr, packet->data);
  EXPECT_NE(static_cast<const void *>(compressed.data.data()),
            static_cast<const void *>(packet->data));
  EXPECT_EQ(static_cast<int>(compressed.data.size()), packet->size);
  EXPECT_EQ(0, std::memcmp(compressed.data.data(), packet->data, compressed.data.size()));
}

TEST(PacketTest, CopyConstructorClonesUnderlyingPacket) {
  const auto source = make_pattern(12);
  auto original = ffmpeg_cpp::Packet::create(source.data(), source.size());
  ASSERT_TRUE(original);

  const ffmpeg_cpp::Packet copy(original);
  ASSERT_TRUE(copy);

  EXPECT_NE(original.get(), copy.get());
  ASSERT_NE(nullptr, copy->data);
  // av_packet_clone shares the underlying refcounted buffer, so the data pointers are allowed
  // to match while still providing value semantics.
  EXPECT_EQ(static_cast<const void *>(original->data), static_cast<const void *>(copy->data));
  EXPECT_EQ(0, std::memcmp(original->data, copy->data, source.size()));
}

TEST(PacketTest, MoveOperationsTransferOwnershipAndClearSource) {
  const auto source = make_pattern(10);
  auto original = ffmpeg_cpp::Packet::create(source.data(), source.size());
  ASSERT_TRUE(original);

  auto moved = ffmpeg_cpp::Packet(std::move(original));
  EXPECT_TRUE(moved);
  EXPECT_FALSE(original);

  const auto other = make_pattern(8);
  auto second = ffmpeg_cpp::Packet::create(other.data(), other.size());
  ASSERT_TRUE(second);

  moved = std::move(second);
  EXPECT_TRUE(moved);
  EXPECT_FALSE(second);

  moved = std::move(moved);
  EXPECT_TRUE(moved);
}

TEST(PacketTest, EmptyReflectsPayloadPresence) {
  auto packet = ffmpeg_cpp::Packet::create();
  ASSERT_TRUE(packet);
  EXPECT_TRUE(packet.empty());

  const auto payload = make_pattern(4);
  packet = ffmpeg_cpp::Packet::create(payload.data(), payload.size());
  EXPECT_FALSE(packet.empty());
}

TEST(PacketTest, ToPacketMsgRoundTripsMetadataAndPayload) {
  const auto msg = make_packet_message();
  const auto packet = ffmpeg_cpp::Packet::create(msg);
  ASSERT_TRUE(packet);

  const rclcpp::Time stamp(5, 6);
  const auto roundtrip = packet.to_packet_msg(stamp, msg.codec);

  EXPECT_EQ(stamp.nanoseconds(), rclcpp::Time(roundtrip.header.stamp).nanoseconds());
  EXPECT_EQ(msg.codec, roundtrip.codec);
  EXPECT_EQ(msg.pts, roundtrip.pts);
  EXPECT_EQ(msg.dts, roundtrip.dts);
  EXPECT_EQ(msg.duration, roundtrip.duration);
  EXPECT_EQ(msg.time_base.num, roundtrip.time_base.num);
  EXPECT_EQ(msg.time_base.den, roundtrip.time_base.den);
  EXPECT_EQ(msg.data, roundtrip.data);
}

TEST(PacketTest, ToCompressedImageMsgRoundTripsPayload) {
  const auto compressed = make_compressed_image("png");
  const auto packet = ffmpeg_cpp::Packet::create(compressed);
  ASSERT_TRUE(packet);

  const rclcpp::Time stamp(7, 8);
  const auto image = packet.to_compressed_image_msg(stamp, compressed.format);

  EXPECT_EQ(stamp.nanoseconds(), rclcpp::Time(image.header.stamp).nanoseconds());
  EXPECT_EQ(compressed.format, image.format);
  EXPECT_EQ(compressed.data, image.data);
}
