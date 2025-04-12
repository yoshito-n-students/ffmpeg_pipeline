#include <algorithm>
#include <cstring>  // for std::memset()
#include <iterator> // for std::begin(), std::end()

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

BufferRef::BufferRef(const BufferRef &buf) : buf_(av_buffer_ref(buf.get()), &unref_buffer) {
  if (!buf_) {
    throw Error("BufferRef::BufferRef(): Failed to create a reference to buffer");
  }
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

bool Packet::empty() const { return !packet_ || !packet_->data || packet_->size == 0; }

void Packet::free_packet(AVPacket *packet) { av_packet_free(&packet); }

// ================================
// Frame - RAII wrapper for AVFrame
// ================================

Frame::Frame() : frame_(av_frame_alloc(), free_frame) {
  if (!frame_) {
    throw Error("Frame::Frame(): Failed to allocate AVFrame");
  }
}

bool Frame::empty() const { return !frame_ || !frame_->data[0]; }

Frame Frame::transfer_data() const {
  Frame dst;
  if (const int ret = av_hwframe_transfer_data(dst.get(), frame_.get(), 0); ret < 0) {
    throw Error("Frame::transfer_data(): Error transferring data", ret);
  }
  return dst;
}

std::string Frame::format_name() const {
  return av_get_pix_fmt_name(static_cast<AVPixelFormat>(frame_->format));
}

void Frame::free_frame(AVFrame *frame) { av_frame_free(&frame); }

} // namespace ffmpeg_cpp