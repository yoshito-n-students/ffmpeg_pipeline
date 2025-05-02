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

Encoder::Encoder() {}

static void set_context_options(AVCodecContext *const encoder_ctx) {
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

Encoder::Encoder(const std::string &codec_name, Dictionary *const codec_options) : Encoder() {
  // Find the encoder by name
  const AVCodec *const codec = avcodec_find_encoder_by_name(codec_name.c_str());
  if (!codec) {
    throw Error("Encoder::Encoder(): " + codec_name + " was not recognized as a encoder name");
  }

  // Allocate the encoder context and set some options
  reset(avcodec_alloc_context3(codec));
  if (!get()) {
    throw Error("Encoder::Encoder(): Failed to allocate codec context");
  }
  set_context_options(get());

  // Open the encoder. avcodec_open2() may free the options,
  // so we release the ownership of it from unique_ptr during calling the function.
  {
    AVDictionary *codec_options_ptr = codec_options->release();
    if (const int ret = avcodec_open2(get(), codec, &codec_options_ptr); ret < 0) {
      throw Error("Encoder::Encoder(): Failed to open codec", ret);
    }
    codec_options->reset(codec_options_ptr);
  }

  // Check if the encoder accepts all the options
  if (!codec_options->empty()) {
    throw Error("Encoder::Encoder(): Encoder does not accept option " +
                codec_options->to_flow_style_yaml());
  }
}

Encoder::Encoder(const CodecParameters &codec_params, Dictionary *const codec_options) : Encoder() {
  // Find the encoder by the given id
  const AVCodec *const codec = avcodec_find_encoder(codec_params->codec_id);
  if (!codec) {
    throw Error("Encoder::Encoder(): Faild to find encoder");
  }

  // Allocate the encoder context and set some options
  reset(avcodec_alloc_context3(codec));
  if (!get()) {
    throw Error("Encoder::Encoder(): Failed to allocate codec context");
  }
  set_context_options(get());

  // Import the codec parameters to the encoder context
  avcodec_parameters_to_context(get(), codec_params.get());
  get()->time_base = av_inv_q(codec_params->framerate);

  // Open the encoder. avcodec_open2() may free the options,
  // so we release the ownership of it from unique_ptr during calling the function.
  {
    AVDictionary *codec_options_ptr = codec_options->release();
    if (const int ret = avcodec_open2(get(), codec, &codec_options_ptr); ret < 0) {
      throw Error("Encoder::Encoder(): Failed to open codec", ret);
    }
    codec_options->reset(codec_options_ptr);
  }

  // Check if the encoder accepts all the options
  if (!codec_options->empty()) {
    throw Error("Encoder::Encoder(): Encoder does not accept option " +
                codec_options->to_flow_style_yaml());
  }
}

void Encoder::send_frame(const Frame &frame) {
  if (const int ret = avcodec_send_frame(get(), frame.get()); ret < 0) {
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
  if (const int ret = avcodec_receive_packet(get(), packet.get());
      ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
    throw Error("Encoder::receive_packet(): Error during encoding", ret);
  }
  return packet;
}

std::string Encoder::codec_name() const { return avcodec_get_name(get()->codec_id); }

std::string Encoder::hw_type_name() const {
  // av_hwdevice_get_type_name(AV_HWDEVICE_TYPE_NONE) returns nullptr,
  // so std::string CANNOT be constructed and std::logic_error is thrown.
  // To avoid this, return "none" in the case of no hardware.
  return get()->hw_device_ctx
             ? av_hwdevice_get_type_name(
                   reinterpret_cast<AVHWDeviceContext *>(get()->hw_device_ctx->data)->type)
             : "none";
}

} // namespace ffmpeg_cpp