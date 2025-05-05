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

Encoder Encoder::null() { return Encoder(nullptr); }

Encoder Encoder::create(const std::string &encoder_name, const CodecParameters &codec_params,
                        const Dictionary &encoder_options) {
  // Find the encoder by the given name or codec id
  const AVCodec *codec = nullptr;
  if (!encoder_name.empty()) {
    codec = avcodec_find_encoder_by_name(encoder_name.c_str());
  } else if (codec_params) {
    codec = avcodec_find_encoder(codec_params->codec_id);
  }
  if (!codec) {
    throw Error("Encoder::create(): Faild to find encoder");
  }

  // Allocate the encoder context
  Encoder encoder(avcodec_alloc_context3(codec));
  if (!encoder) {
    throw Error("Encoder::create(): Failed to allocate codec context");
  }

  // Create a hardware acceleration context supported by the encoder.
  // If multiple hardware devices are supported, the first one is used.
  if (!encoder->hw_device_ctx) {
    for (int i = 0;; ++i) {
      if (const AVCodecHWConfig *const hw_config = avcodec_get_hw_config(encoder->codec, i);
          hw_config                                                         // HW config exists
          && (hw_config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) // HW context supported
          && av_hwdevice_ctx_create(&encoder->hw_device_ctx, hw_config->device_type, nullptr,
                                    nullptr, 0) == 0 // HW context created
      ) {
        break; // exit the loop if the HW context is created successfully
      } else if (!hw_config) {
        break; // exit the loop if no more HW config is available
      }
    }
  }

  // Import the codec parameters to the encoder context except for codec_{type, id}
  // because they suppose to be already set by avcodec_alloc_context3()
  // and are ommitted from the codec parameters.
  if (codec_params) {
    const AVMediaType codec_type = encoder->codec_type;
    const AVCodecID codec_id = encoder->codec_id;
    if (const int ret = avcodec_parameters_to_context(encoder.get(), codec_params.get()); ret < 0) {
      throw Error("Encoder::create(): Failed to import codec parameters", ret);
    }
    encoder->codec_type = codec_type;
    encoder->codec_id = codec_id;
    encoder->time_base = av_inv_q(codec_params->framerate);
  }

  // Open the encoder
  if (encoder_options) {
    // With the encoder options. We copy the given options and release the ownership of it
    // during calling avcodec_open2() because the funtion modify the options.
    Dictionary writable_options = encoder_options;
    AVDictionary *writable_options_ptr = writable_options.release();
    const int ret = avcodec_open2(encoder.get(), codec, &writable_options_ptr);
    writable_options.reset(writable_options_ptr);
    if (ret < 0) {
      throw Error("Encoder::create(): Failed to open the encoder", ret);
    }
    if (writable_options) {
      throw Error("Encoder::create(): Options " + writable_options.to_flow_style_yaml() +
                  " were not accepted by the encoder");
    }
  } else {
    // Without the encoder options
    if (const int ret = avcodec_open2(encoder.get(), codec, nullptr); ret < 0) {
      throw Error("Encoder::create(): Failed to open the decoder", ret);
    }
  }

  return encoder;
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
  Packet packet = Packet::create();
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