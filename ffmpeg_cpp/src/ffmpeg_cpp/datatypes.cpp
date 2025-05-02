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

Packet::Packet() : std::unique_ptr<AVPacket, Deleter<AVPacket>>(av_packet_alloc()) {
  if (!get()) {
    throw Error("Packet::Packet(): Failed to allocate AVPacket");
  }
}

Packet::Packet(const std::uint8_t *const data, const std::size_t size) : Packet() {
  // Take the copy of the given data
  std::uint8_t *const data_copy = static_cast<std::uint8_t *>(av_malloc(size));
  if (!data_copy) {
    throw Error("Packet::Packet(): Failed to allocate memory for packet data");
  }
  std::memcpy(data_copy, data, size);

  // Create the packet from the copied data
  if (const int ret = av_packet_from_data(get(), data_copy, size); ret < 0) {
    av_free(data_copy);
    throw Error("Packet::Packet(): Failed to create a packet from data", ret);
  }
}

Packet::Packet(const ffmpeg_pipeline_msgs::msg::Packet &msg)
    : Packet(msg.data.data(), msg.data.size()) {
  get()->pts = msg.pts;
  get()->dts = msg.dts;
  get()->duration = msg.duration;
  get()->time_base.num = msg.time_base.num;
  get()->time_base.den = msg.time_base.den;
}

Packet::Packet(const Packet &other) : Packet() {
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

template <> void Deleter<AVPacket>::operator()(AVPacket *packet) const { av_packet_free(&packet); }

// ================================
// Frame - RAII wrapper for AVFrame
// ================================

Frame::Frame() : std::unique_ptr<AVFrame, Deleter<AVFrame>>(av_frame_alloc()) {
  if (!get()) {
    throw Error("Frame::Frame(): Failed to allocate AVFrame");
  }
}

Frame::Frame(const Frame &other) : Frame() {
  if (const int ret = av_frame_ref(get(), other.get()); ret < 0) {
    throw Error("Frame::Frame(): Failed to create a reference to frame", ret);
  }
}

Frame Frame::transfer_data() const {
  Frame dst;
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

template <> void Deleter<AVFrame>::operator()(AVFrame *frame) const { av_frame_free(&frame); }

} // namespace ffmpeg_cpp