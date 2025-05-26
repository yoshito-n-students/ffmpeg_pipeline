#include <cstdint> // required before including <libavutil/avutil.h>
#include <cstring> // for std::memcpy()

extern "C" {
#include <libavutil/avutil.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/packet.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "internal.hpp"

namespace ffmpeg_cpp {

// ==================================
// Packet - RAII wrapper for AVPacket
// ==================================

Packet Packet::null() { return Packet(nullptr); }

Packet Packet::create() {
  Packet packet(av_packet_alloc());
  if (!packet) {
    throw Error("Packet::create(): Failed to allocate AVPacket");
  }
  return packet;
}

Packet Packet::create(const std::uint8_t *const data, const std::size_t size) {
  // Take the copy of the given data
  std::uint8_t *const data_copy = static_cast<std::uint8_t *>(av_malloc(size));
  if (!data_copy) {
    throw Error("Packet::create(): Failed to allocate memory for packet data");
  }
  std::memcpy(data_copy, data, size);

  // Create the packet from the copied data
  Packet packet = Packet::create();
  if (const int ret = av_packet_from_data(packet.get(), data_copy, size); ret < 0) {
    av_free(data_copy);
    throw Error("Packet::create(): Failed to create a packet from data", ret);
  }
  return packet;
}

Packet Packet::create(const ffmpeg_pipeline_msgs::msg::Packet &msg) {
  Packet packet = Packet::create(msg.data.data(), msg.data.size());
  packet->pts = msg.pts;
  packet->dts = msg.dts;
  packet->duration = msg.duration;
  packet->time_base.num = msg.time_base.num;
  packet->time_base.den = msg.time_base.den;
  return packet;
}

Packet Packet::create(const sensor_msgs::msg::CompressedImage &msg) {
  return Packet::create(msg.data.data(), msg.data.size());
}

Packet::Packet(const Packet &other) : UniquePtr<AVPacket>() {
  if (other) {
    reset(av_packet_clone(other.get()));
    if (!get()) {
      throw Error("Packet::Packet(): Failed to clone packet");
    }
  } else {
    *this = null();
  }
}

ffmpeg_pipeline_msgs::msg::Packet Packet::to_packet_msg(const rclcpp::Time &stamp,
                                                        const std::string &codec_name) const {
  ffmpeg_pipeline_msgs::msg::Packet msg;
  msg.header.stamp = stamp;
  msg.codec = codec_name;
  msg.pts = get()->pts;
  msg.dts = get()->dts;
  msg.data.assign(get()->data, get()->data + get()->size);
  msg.duration = get()->duration;
  msg.time_base.num = get()->time_base.num;
  msg.time_base.den = get()->time_base.den;
  return msg;
}

sensor_msgs::msg::CompressedImage Packet::to_compressed_image_msg(const rclcpp::Time &stamp,
                                                                  const std::string &format) const {
  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp = stamp;
  msg.format = format;
  msg.data.assign(get()->data, get()->data + get()->size);
  return msg;
}
} // namespace ffmpeg_cpp