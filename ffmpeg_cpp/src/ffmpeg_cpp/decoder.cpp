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

Decoder Decoder::null() { return Decoder(nullptr); }

static void set_context_options(AVCodecContext *const decoder_ctx) {
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

Decoder Decoder::create(const std::string &decoder_name, const CodecParameters &codec_params,
                        const Dictionary &decoder_options) {
  // Find the decoder by the given name or codec id
  const AVCodec *codec = nullptr;
  if (!decoder_name.empty()) {
    codec = avcodec_find_decoder_by_name(decoder_name.c_str());
  } else if (codec_params) {
    codec = avcodec_find_decoder(codec_params->codec_id);
  }
  if (!codec) {
    throw Error("Decoder::create(): Failed to find the decoder");
  }

  // Allocate the decoder context and set some options
  Decoder decoder(avcodec_alloc_context3(codec));
  if (!decoder) {
    throw Error("Decoder::create(): Failed to allocate the decoder context");
  }
  set_context_options(decoder.get());

  // Import the codec parameters to the decoder context except for codec_{type, id}
  // because they suppose to be already set by avcodec_alloc_context3()
  // and are ommitted from the codec parameters.
  if (codec_params) {
    const AVMediaType codec_type = decoder->codec_type;
    const AVCodecID codec_id = decoder->codec_id;
    if (const int ret = avcodec_parameters_to_context(decoder.get(), codec_params.get()); ret < 0) {
      throw Error("Decoder::create(): Failed to import codec parameters", ret);
    }
    decoder->codec_type = codec_type;
    decoder->codec_id = codec_id;
  }

  // Open the decoder. We copy the given options and release the ownership of it
  // during calling avcodec_open2() because the funtion modify the options.
  // The following code should work even if the given options are nullptr.
  {
    Dictionary writable_options = decoder_options;
    AVDictionary *writable_options_ptr = writable_options.release();
    const int ret = avcodec_open2(decoder.get(), codec, &writable_options_ptr);
    writable_options.reset(writable_options_ptr);
    if (ret < 0) {
      throw Error("Decoder::create(): Failed to open the decoder", ret);
    }
    if (writable_options) {
      throw Error("Decoder::create(): Options " + writable_options.to_flow_style_yaml() +
                  " were not accepted by the decoder");
    }
  }

  return decoder;
}

void Decoder::send_packet(const Packet &packet) {
  if (const int ret = avcodec_send_packet(get(), packet.get()); ret < 0) {
    throw Error("Decoder::send_packet(): Error sending packet for decoding", ret);
  }
}

Frame Decoder::receive_frame() {
  // If the return value of avcodec_receive_frame() is one of the following, return frame
  // - 0: The frame was successfully decoded
  // - AVERROR(EAGAIN): No frame available due to insufficient packets
  // - AVERROR_EOF: No frame available because the decoder has finished successfully
  // TODO: Notify the reason for the empty frame to the caller
  Frame frame = Frame::create();
  if (const int ret = avcodec_receive_frame(get(), frame.get());
      ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
    throw Error("Decoder::receive_frame(): Error during decoding", ret);
  }
  return frame;
}

std::string Decoder::codec_name() const { return avcodec_get_name(get()->codec_id); }

std::string Decoder::hw_type_name() const {
  // av_hwdevice_get_type_name(AV_HWDEVICE_TYPE_NONE) returns nullptr,
  // so std::string CANNOT be constructed and std::logic_error is thrown.
  // To avoid this, return "none" in the case of no hardware.
  return get()->hw_device_ctx
             ? av_hwdevice_get_type_name(
                   reinterpret_cast<AVHWDeviceContext *>(get()->hw_device_ctx->data)->type)
             : "none";
}

} // namespace ffmpeg_cpp