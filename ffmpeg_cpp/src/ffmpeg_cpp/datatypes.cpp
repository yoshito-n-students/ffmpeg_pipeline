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

namespace ffmpeg_cpp {

// ========================================
// BufferRef - RAII wrapper for AVBufferRef
// ========================================

BufferRef::BufferRef(const std::uint8_t *const data, const std::size_t unpadded_size)
    : buf_(av_buffer_alloc(unpadded_size + AV_INPUT_BUFFER_PADDING_SIZE), &unref_buffer) {
  if (!buf_) {
    throw Error("BufferRef::BufferRef(): Failed to allocate AVBufferRef");
  }
  // Copy the given data to the buffer and zero the padding
  std::copy(data, data + unpadded_size, buf_->data);
  std::memset(buf_->data + unpadded_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
}

BufferRef::BufferRef(const BufferRef &other) : buf_(av_buffer_ref(other.get()), &unref_buffer) {
  if (!buf_) {
    throw Error("BufferRef::BufferRef(): Failed to create a reference to buffer");
  }
}

BufferRef &BufferRef::operator=(const BufferRef &other) {
  buf_.reset(av_buffer_ref(other.get()));
  if (!buf_) {
    throw Error("BufferRef::operator=(): Failed to create a reference to buffer");
  }
  return *this;
}

void BufferRef::unref_buffer(AVBufferRef *buf) { av_buffer_unref(&buf); }

// ==================================
// Packet - RAII wrapper for AVPacket
// ==================================

Packet::Packet() : packet_(av_packet_alloc(), free_packet) {
  if (!packet_) {
    throw Error("Packet::Packet(): Failed to allocate AVPacket");
  }
}

Packet::Packet(const BufferRef &buf) : Packet() {
  packet_->buf = av_buffer_ref(buf.get());
  if (!packet_->buf) {
    throw Error("Packet::Packet(): Failed to create a reference to buffer");
  }
  packet_->data = buf->data;
  packet_->size = buf.unpadded_size();
}

Packet::Packet(const ffmpeg_pipeline_msgs::msg::Packet &msg)
    : Packet(BufferRef(msg.data.data(), msg.data.size())) {
  // TODO: copy timestamps and other fields??
}

Packet::Packet(const Packet &other) : Packet() {
  if (const int ret = av_packet_ref(packet_.get(), other.get()); ret < 0) {
    throw Error("Packet::Packet(): Failed to create a reference to packet", ret);
  }
}

Packet &Packet::operator=(const Packet &other) {
  if (const int ret = av_packet_ref(packet_.get(), other.get()); ret < 0) {
    throw Error("Packet::operator=(): Failed to create a reference to packet", ret);
  }
  return *this;
}

ffmpeg_pipeline_msgs::msg::Packet::UniquePtr Packet::to_msg() const {
  auto msg = std::make_unique<ffmpeg_pipeline_msgs::msg::Packet>();
  msg->header.stamp.sec = packet_->dts / 1'000'000;
  msg->header.stamp.nanosec = (packet_->dts % 1'000'000) * 1'000;
  // msg->codec = codec_name;
  msg->data.assign(packet_->data, packet_->data + packet_->size);
  return msg;
}

void Packet::free_packet(AVPacket *packet) { av_packet_free(&packet); }

// ================================
// Frame - RAII wrapper for AVFrame
// ================================

Frame::Frame() : frame_(av_frame_alloc(), free_frame) {
  if (!frame_) {
    throw Error("Frame::Frame(): Failed to allocate AVFrame");
  }
}

Frame::Frame(const Frame &other) : Frame() {
  if (const int ret = av_frame_ref(frame_.get(), other.get()); ret < 0) {
    throw Error("Frame::Frame(): Failed to create a reference to frame", ret);
  }
}

Frame &Frame::operator=(const Frame &other) {
  if (const int ret = av_frame_ref(frame_.get(), other.get()); ret < 0) {
    throw Error("Frame::operator=(): Failed to create a reference to frame", ret);
  }
  return *this;
}

Frame Frame::transfer_data() const {
  Frame dst;
  if (const int ret = av_hwframe_transfer_data(dst.get(), frame_.get(), 0); ret < 0) {
    throw Error("Frame::transfer_data(): Error transferring data", ret);
  }
  return dst;
}

std::string Frame::format_name() const {
  // There is no field in AVFrame to indicate the data type, so we use heuristics to determine it
  if (frame_->width > 0 && frame_->height > 0) {
    return av_get_pix_fmt_name(static_cast<AVPixelFormat>(frame_->format));
  } else if (frame_->nb_samples > 0) {
    return av_get_sample_fmt_name(static_cast<AVSampleFormat>(frame_->format));
  }
  return "";
}

void Frame::free_frame(AVFrame *frame) { av_frame_free(&frame); }

} // namespace ffmpeg_cpp