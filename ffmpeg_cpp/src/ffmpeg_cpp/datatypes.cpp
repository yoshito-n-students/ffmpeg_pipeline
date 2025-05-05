#include <algorithm>
#include <cstring> // for std::memset()
#include <iostream>
#include <iterator> // for std::begin(), std::end()
#include <tuple>    // for std::tie()
#include <utility>  // for std::pair

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

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

Packet::Packet(const Packet &other) : std::unique_ptr<AVPacket, Deleter<AVPacket>>() {
  *this = Packet::create();
  if (const int ret = av_packet_ref(get(), other.get()); ret < 0) {
    throw Error("Packet::Packet(): Failed to create a reference to packet", ret);
  }
}

ffmpeg_pipeline_msgs::msg::Packet Packet::to_msg(const rclcpp::Time &stamp,
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

// ================================
// Frame - RAII wrapper for AVFrame
// ================================

Frame Frame::null() { return Frame(nullptr); }

Frame Frame::create() {
  Frame frame(av_frame_alloc());
  if (!frame) {
    throw Error("Frame::create(): Failed to allocate AVFrame");
  }
  return frame;
}

Frame::Frame(const Frame &other) : std::unique_ptr<AVFrame, Deleter<AVFrame>>() {
  *this = Frame::create();
  if (const int ret = av_frame_ref(get(), other.get()); ret < 0) {
    throw Error("Frame::Frame(): Failed to create a reference to frame", ret);
  }
}

Frame Frame::transfer_data() const {
  Frame dst = Frame::create();
  if (const int ret = av_hwframe_transfer_data(dst.get(), get(), 0); ret < 0) {
    throw Error("Frame::transfer_data(): Error transferring data", ret);
  }
  return dst;
}

std::string Frame::format_name() const {
  // There is no field in AVFrame to indicate the data type, so we use heuristics to determine it
  if (get()->width > 0 && get()->height > 0) {
    return av_get_pix_fmt_name(static_cast<AVPixelFormat>(get()->format));
  } else if (get()->nb_samples > 0) {
    return av_get_sample_fmt_name(static_cast<AVSampleFormat>(get()->format));
  }
  return "";
}

std::string Frame::ch_layout_str() const { return to_string(get()->ch_layout); }

} // namespace ffmpeg_cpp