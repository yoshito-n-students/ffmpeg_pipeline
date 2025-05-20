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

Decoder Decoder::create(const std::string &decoder_name, const CodecParameters &codec_params,
                        const std::string &hw_type_name, const std::string &request_format_name,
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

  // Set the options to enable error concealment and format preference.
  // Some options are for video decoders, but they suppose no problem for other decoders.
  decoder->workaround_bugs = FF_BUG_AUTODETECT;
  decoder->err_recognition = AV_EF_CRCCHECK;
  decoder->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;

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

  // Enable a hardware acceleration
  if (!hw_type_name.empty()) {
    if (hw_type_name == "auto") {
      // Use the first available hardware device type
      for (int i = 0;; ++i) {
        const AVCodecHWConfig *const hw_config = avcodec_get_hw_config(decoder->codec, i);
        if (hw_config // HW config exists
            &&
            (hw_config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) // HW context supported
            && av_hwdevice_ctx_create(&decoder->hw_device_ctx, hw_config->device_type, nullptr,
                                      nullptr, 0) == 0 // HW context created
        ) {
          break; // exit the loop if the HW context is created successfully
        } else if (!hw_config) {
          break; // exit the loop if no more HW config is available
        }
      }
    } else {
      // USe the specified hardware device type
      const AVHWDeviceType hw_type = av_hwdevice_find_type_by_name(hw_type_name.c_str());
      if (hw_type == AV_HWDEVICE_TYPE_NONE) {
        throw Error("Decoder::create(): " + hw_type_name + " is not a valid hardware type name");
      }
      if (const int ret =
              av_hwdevice_ctx_create(&decoder->hw_device_ctx, hw_type, nullptr, nullptr, 0);
          ret < 0) {
        throw Error("Decoder::create(): Failed to create the hardware device context", ret);
      }
    }
  }

  // Set pixel or sample format preference
  if (!request_format_name.empty()) {
    switch (decoder->codec_type) {
    case AVMEDIA_TYPE_VIDEO:
      if (const AVPixelFormat request_pixel_fmt = av_get_pix_fmt(request_format_name.c_str());
          request_pixel_fmt != AV_PIX_FMT_NONE) {
        // Utilize request_sample_fmt field to record the desired pixel format
        decoder->request_sample_fmt = static_cast<AVSampleFormat>(request_pixel_fmt);
        // get_format is called when the decoder decides the destination pixel format.
        // We select the requested pixel format if it is supported by the decoder,
        // otherwise defer to the default behavior.
        decoder->get_format = [](AVCodecContext *decoder_ctx, const AVPixelFormat *formats) {
          for (const AVPixelFormat *format = formats; *format != AV_PIX_FMT_NONE; ++format) {
            if (*format == static_cast<AVPixelFormat>(decoder_ctx->request_sample_fmt)) {
              return *format;
            }
          }
          return avcodec_default_get_format(decoder_ctx, formats);
        };
      } else {
        throw Error("Decoder::create(): " + request_format_name +
                    " is not a valid pixel format name");
      }
      break;
    case AVMEDIA_TYPE_AUDIO:
      if (const AVSampleFormat request_sample_fmt = av_get_sample_fmt(request_format_name.c_str());
          request_sample_fmt != AV_SAMPLE_FMT_NONE) {
        // Utilize request_sample_fmt field as usual
        decoder->request_sample_fmt = request_sample_fmt;
      } else {
        throw Error("Decoder::create(): " + request_format_name +
                    " is not a valid sample format name");
      }
      break;
    default:
      // Warn that the request format is not supported?
      break;
    }
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
  Frame frame = Frame::create();
  if (const int ret = avcodec_receive_frame(get(), frame.get());
      ret >= 0 /* The frame was successfully decoded */) {
    return frame;
  } else if (
      ret == AVERROR(EAGAIN) /* No frame available due to insufficient packets */ ||
      ret == AVERROR_EOF /* No frame available because the decoder has finished successfully */) {
    return Frame::null();
  } else { /* ret < 0; Error during decoding */
    throw Error("Decoder::receive_frame(): Error during decoding", ret);
  }
}

std::string Decoder::codec_name() const { return get() ? avcodec_get_name(get()->codec_id) : ""; }

std::string Decoder::hw_type_name() const {
  return (get() && get()->hw_device_ctx)
             ? to_string(reinterpret_cast<AVHWDeviceContext *>(get()->hw_device_ctx->data)->type)
             : "";
}

} // namespace ffmpeg_cpp