#include <algorithm>
#include <iterator> // for std::begin(), std::end()

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

#include "internal.hpp"

namespace ffmpeg_cpp {

// =========================================
// Decoder - RAII wrapper for AVCodecContext
// =========================================

static void set_options(AVCodecContext *const decoder_ctx) {
  // Set the options to enable error concealment and format preference.
  // Some options are for video decoders, but they suppose no problem for other decoders.
  decoder_ctx->workaround_bugs = FF_BUG_AUTODETECT;
  decoder_ctx->err_recognition = AV_EF_CRCCHECK;
  decoder_ctx->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
  decoder_ctx->get_format = [](AVCodecContext *decoder_ctx, const AVPixelFormat *formats) {
    // Prefer the first pixel formats compatible with ROS image encodings
    // to avoid unnecessary conversions after decoding
    for (const AVPixelFormat *format = formats; *format != AV_PIX_FMT_NONE; ++format) {
      if (std::any_of(std::begin(format_encoding_pairs), std::end(format_encoding_pairs),
                      [format](const auto &pair) { return pair.first == *format; })) {
        return *format;
      }
    }
    // If no compatible pixel format is found, defer to the default behavior
    return avcodec_default_get_format(decoder_ctx, formats);
  };

  // Create a hardware acceleration context supported by the decoder.
  // If multiple hardware devices are supported, the first one is used.
  if (!decoder_ctx->hw_device_ctx) {
    for (int i = 0;; ++i) {
      const AVCodecHWConfig *const hw_config = avcodec_get_hw_config(decoder_ctx->codec, i);
      if (hw_config                                                         // HW config exists
          && (hw_config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) // HW context supported
          && av_hwdevice_ctx_create(&decoder_ctx->hw_device_ctx, hw_config->device_type, nullptr,
                                    nullptr, 0) == 0 // HW context created
      ) {
        break; // exit the loop if the HW context is created successfully
      } else if (!hw_config) {
        break; // exit the loop if no more HW config is available
      }
    }
  }
}

Decoder::Decoder(const std::string &codec_name, Dictionary *const options)
    : decoder_ctx_(nullptr, &free_context) {
  // Find the decoder by name
  const AVCodec *const codec = avcodec_find_decoder_by_name(codec_name.c_str());
  if (!codec) {
    throw Error("Decoder::Decoder(): " + codec_name + " was not recognized as a decoder name");
  }

  // Allocate the decoder context and set some options
  decoder_ctx_.reset(avcodec_alloc_context3(codec));
  if (!decoder_ctx_) {
    throw Error("Decoder::Decoder(): Failed to allocate codec context");
  }
  set_options(decoder_ctx_.get());

  // Open the decoder. avcodec_open2() may free the options,
  // so we release the ownership of it from unique_ptr during calling the function.
  {
    AVDictionary *options_ptr = options->release();
    if (const int ret = avcodec_open2(decoder_ctx_.get(), codec, &options_ptr); ret < 0) {
      throw Error("Decoder::Decoder(): Failed to open codec", ret);
    }
    *options = Dictionary(options_ptr);
  }

  // Check if the decoder accepts all the options
  if (!options->empty()) {
    throw Error("Decoder::Decoder(): Decoder does not accept option [" +
                options->to_flow_style_yaml() + "]");
  }
}

Decoder::Decoder(const CodecParameters &params, Dictionary *const options)
    : decoder_ctx_(nullptr, &free_context) {
  // Find the decoder by the given id
  const AVCodec *const codec = avcodec_find_decoder(params->codec_id);
  if (!codec) {
    throw Error("Decoder::Decoder(): Faild to find decoder");
  }

  // Allocate the decoder context and set some options
  decoder_ctx_.reset(avcodec_alloc_context3(codec));
  if (!decoder_ctx_) {
    throw Error("Decoder::Decoder(): Failed to allocate codec context");
  }
  set_options(decoder_ctx_.get());

  // Import the codec parameters to the decoder context
  avcodec_parameters_to_context(decoder_ctx_.get(), params.get());

  // Open the decoder. avcodec_open2() may free the options,
  // so we release the ownership of it from unique_ptr during calling the function.
  {
    AVDictionary *options_ptr = options->release();
    if (const int ret = avcodec_open2(decoder_ctx_.get(), codec, &options_ptr); ret < 0) {
      throw Error("Decoder::Decoder(): Failed to open codec", ret);
    }
    *options = Dictionary(options_ptr);
  }

  // Check if the decoder accepts all the options
  if (!options->empty()) {
    throw Error("Decoder::Decoder(): Decoder does not accept option [" +
                options->to_flow_style_yaml() + "]");
  }
}

void Decoder::send_packet(const Packet &packet) {
  if (const int ret = avcodec_send_packet(decoder_ctx_.get(), packet.get()); ret < 0) {
    throw Error("Decoder::send_packet(): Error sending packet for decoding", ret);
  }
}

Frame Decoder::receive_frame() {
  // If the return value of avcodec_receive_frame() is one of the following, return frame
  // - 0: The frame was successfully decoded
  // - AVERROR(EAGAIN): No frame available due to insufficient packets
  // - AVERROR_EOF: No frame available because the decoder has finished successfully
  // TODO: Notify the reason for the empty frame to the caller
  Frame frame;
  if (const int ret = avcodec_receive_frame(decoder_ctx_.get(), frame.get());
      ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
    throw Error("Decoder::receive_frame(): Error during decoding", ret);
  }
  return frame;
}

std::string Decoder::codec_name() const { return avcodec_get_name(decoder_ctx_->codec_id); }

std::string Decoder::hw_type_name() const {
  // av_hwdevice_get_type_name(AV_HWDEVICE_TYPE_NONE) returns nullptr,
  // so std::string CANNOT be constructed and std::logic_error is thrown.
  // To avoid this, return "none" in the case of no hardware.
  return decoder_ctx_->hw_device_ctx
             ? av_hwdevice_get_type_name(
                   reinterpret_cast<AVHWDeviceContext *>(decoder_ctx_->hw_device_ctx->data)->type)
             : "none";
}

void Decoder::free_context(AVCodecContext *decoder_ctx) { avcodec_free_context(&decoder_ctx); }

} // namespace ffmpeg_cpp