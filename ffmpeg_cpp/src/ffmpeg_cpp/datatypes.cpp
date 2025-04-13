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

void Packet::free_packet(AVPacket *packet) { av_packet_free(&packet); }

// ================================
// Frame - RAII wrapper for AVFrame
// ================================

Frame::Frame() : frame_(av_frame_alloc(), free_frame) {
  if (!frame_) {
    throw Error("Frame::Frame(): Failed to allocate AVFrame");
  }
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

// ====================================================
// CodecParameters - RAII wrapper for AVCodecParameters
// ====================================================

CodecParameters::CodecParameters() : params_(avcodec_parameters_alloc(), free_parameters) {
  if (!params_) {
    throw Error("CodecParameters::CodecParameters(): Failed to allocate AVCodecParameters");
  }
}

std::string CodecParameters::codec_type_name() const {
  return av_get_media_type_string(params_->codec_type);
}

std::string CodecParameters::codec_name() const { return avcodec_get_name(params_->codec_id); }

std::string CodecParameters::format_name() const {
  switch (params_->codec_type) {
  case AVMEDIA_TYPE_VIDEO:
    return av_get_pix_fmt_name(static_cast<AVPixelFormat>(params_->format));
  case AVMEDIA_TYPE_AUDIO:
    return av_get_sample_fmt_name(static_cast<AVSampleFormat>(params_->format));
  default:
    return "";
  }
}

void CodecParameters::free_parameters(AVCodecParameters *params) {
  avcodec_parameters_free(&params);
}

} // namespace ffmpeg_cpp