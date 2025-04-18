#include <algorithm>
#include <iterator> // for std::begin(), std::end()

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_cpp {

// =========================================
// Decoder - RAII wrapper for AVCodecContext
// =========================================

static void set_options(AVCodecContext *const encoder_ctx) {
  // Create a hardware acceleration context supported by the encoder.
  // If multiple hardware devices are supported, the first one is used.
  if (!encoder_ctx->hw_device_ctx) {
    for (int i = 0;; ++i) {
      const AVCodecHWConfig *const hw_config = avcodec_get_hw_config(encoder_ctx->codec, i);
      if (hw_config                                                         // HW config exists
          && (hw_config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) // HW context supported
          && av_hwdevice_ctx_create(&encoder_ctx->hw_device_ctx, hw_config->device_type, nullptr,
                                    nullptr, 0) == 0 // HW context created
      ) {
        break; // exit the loop if the HW context is created successfully
      } else if (!hw_config) {
        break; // exit the loop if no more HW config is available
      }
    }
  }
}

Encoder::Encoder(const std::string &codec_name) : encoder_ctx_(nullptr, &free_context) {
  // Find the encoder by name
  const AVCodec *const codec = avcodec_find_encoder_by_name(codec_name.c_str());
  if (!codec) {
    throw Error("Encoder::Encoder(): " + codec_name + " was not recognized as a encoder name");
  }

  // Allocate the encoder context and set some options
  encoder_ctx_.reset(avcodec_alloc_context3(codec));
  if (!encoder_ctx_) {
    throw Error("Encoder::Encoder(): Failed to allocate codec context");
  }
  set_options(encoder_ctx_.get());

  // Open the encoder
  if (const int ret = avcodec_open2(encoder_ctx_.get(), codec, nullptr); ret < 0) {
    throw Error("Encoder::Encoder(): Failed to open codec", ret);
  }
}

Encoder::Encoder(const CodecParameters &params) : encoder_ctx_(nullptr, &free_context) {
  // Find the encoder by the given id
  const AVCodec *const codec = avcodec_find_encoder(params->codec_id);
  if (!codec) {
    throw Error("Encoder::Encoder(): Faild to find encoder");
  }

  // Allocate the encoder context and set some options
  encoder_ctx_.reset(avcodec_alloc_context3(codec));
  if (!encoder_ctx_) {
    throw Error("Encoder::Encoder(): Failed to allocate codec context");
  }
  set_options(encoder_ctx_.get());

  // Import the codec parameters to the encoder context
  avcodec_parameters_to_context(encoder_ctx_.get(), params.get());
  encoder_ctx_->time_base = av_inv_q(params->framerate);

  // Open the encoder
  if (const int ret = avcodec_open2(encoder_ctx_.get(), codec, nullptr); ret < 0) {
    throw Error("Encoder::Encoder(): Failed to open codec", ret);
  }
}

void Encoder::send_frame(const Frame &frame) {
  if (const int ret = avcodec_send_frame(encoder_ctx_.get(), frame.get()); ret < 0) {
    throw Error("Encoder::send_frame(): Error sending packet for encoding", ret);
  }
}

Packet Encoder::receive_packet() {
  // If the return value of avcodec_receive_packet() is one of the following, return packet
  // - 0: The packet was successfully encoded
  // - AVERROR(EAGAIN): No packet available due to insufficient input frames
  // - AVERROR_EOF: No packet available because the encoder has finished successfully
  // TODO: Notify the reason for the empty packet to the caller
  Packet packet;
  if (const int ret = avcodec_receive_packet(encoder_ctx_.get(), packet.get());
      ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
    throw Error("Encoder::receive_packet(): Error during encoding", ret);
  }
  return packet;
}

std::string Encoder::codec_name() const { return avcodec_get_name(encoder_ctx_->codec_id); }

std::string Encoder::hw_type_name() const {
  // av_hwdevice_get_type_name(AV_HWDEVICE_TYPE_NONE) returns nullptr,
  // so std::string CANNOT be constructed and std::logic_error is thrown.
  // To avoid this, return "none" in the case of no hardware.
  return encoder_ctx_->hw_device_ctx
             ? av_hwdevice_get_type_name(
                   reinterpret_cast<AVHWDeviceContext *>(encoder_ctx_->hw_device_ctx->data)->type)
             : "none";
}

void Encoder::free_context(AVCodecContext *encoder_ctx) { avcodec_free_context(&encoder_ctx); }

} // namespace ffmpeg_cpp