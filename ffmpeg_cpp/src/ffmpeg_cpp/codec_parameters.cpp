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

#include <yaml-cpp/yaml.h>

#include "internal.hpp"

namespace ffmpeg_cpp {

// ====================================================
// CodecParameters - RAII wrapper for AVCodecParameters
// ====================================================

CodecParameters::CodecParameters() : params_(avcodec_parameters_alloc(), free_parameters) {
  if (!params_) {
    throw Error("CodecParameters::CodecParameters(): Failed to allocate AVCodecParameters");
  }
}

CodecParameters::CodecParameters(const std::string &yaml) : CodecParameters() {
  if (!YAML::convert<CodecParameters>::decode(YAML::Load(yaml), *this)) {
    throw Error("CodecParameters::CodecParameters(): Failed to parse codec parameters from yaml");
  }
}

CodecParameters::CodecParameters(const CodecParameters &other) : CodecParameters() {
  if (const int ret = avcodec_parameters_copy(params_.get(), other.get()); ret < 0) {
    throw Error("CodecParameters::CodecParameters(): Failed to copy codec parameters", ret);
  }
}

CodecParameters &CodecParameters::operator=(const CodecParameters &other) {
  if (const int ret = avcodec_parameters_copy(params_.get(), other.get()); ret < 0) {
    throw Error("CodecParameters::operator=(): Failed to copy codec parameters", ret);
  }
  return *this;
}

std::string CodecParameters::codec_type_name() const { return to_string(params_->codec_type); }

std::string CodecParameters::codec_name() const { return avcodec_get_name(params_->codec_id); }

std::string CodecParameters::format_name() const {
  switch (params_->codec_type) {
  case AVMEDIA_TYPE_VIDEO:
    return to_string(static_cast<AVPixelFormat>(params_->format));
  case AVMEDIA_TYPE_AUDIO:
    return to_string(static_cast<AVSampleFormat>(params_->format));
  default:
    return "";
  }
}

void CodecParameters::free_parameters(AVCodecParameters *params) {
  avcodec_parameters_free(&params);
}

} // namespace ffmpeg_cpp

namespace YAML {

// ===========================================
// Conversion between CodecParameters and yaml
// ===========================================

template <typename T>
static void decode_if_exists(const Node &yaml, const std::string &key, T *const value) {
  if (const Node elem = yaml[key]; elem) {
    *value = elem.as<T>();
  }
}

static void decode_if_exists(const Node &yaml, const std::string &key, AVRational *const value) {
  if (const Node elem = yaml[key]; elem) {
    std::tie(value->num, value->den) = elem.as<std::pair<int, int>>();
  }
}

bool convert<ffmpeg_cpp::CodecParameters>::decode(const Node &yaml,
                                                  ffmpeg_cpp::CodecParameters &params) {
  try {
    // === For any codec type ===
    if (const Node elem = yaml["codec"]; elem) {
      const std::string codec_name = elem.as<std::string>();
      if (const AVCodec *const decoder = avcodec_find_decoder_by_name(codec_name.c_str());
          decoder) {
        params->codec_type = avcodec_get_type(decoder->id);
        params->codec_id = decoder->id;
      } else if (const AVCodec *const encoder = avcodec_find_encoder_by_name(codec_name.c_str());
                 encoder) {
        params->codec_type = avcodec_get_type(encoder->id);
        params->codec_id = encoder->id;
      } else {
        throw std::runtime_error(codec_name + " is not a valid codec name");
      }
    }
    decode_if_exists(yaml, "bit_rate", &params->bit_rate);
    decode_if_exists(yaml, "bits_per_coded_sample", &params->bits_per_coded_sample);
    decode_if_exists(yaml, "bits_per_raw_sample", &params->bits_per_raw_sample);
    decode_if_exists(yaml, "profile", &params->profile);
    decode_if_exists(yaml, "level", &params->level);
    // === for video and audio ===
    if (const Node elem = yaml["format"]; elem) {
      switch (params->codec_type) {
      case AVMEDIA_TYPE_VIDEO:
        params->format = av_get_pix_fmt(elem.as<std::string>().c_str());
        break;
      case AVMEDIA_TYPE_AUDIO:
        params->format = av_get_sample_fmt(elem.as<std::string>().c_str());
        break;
      default:
        std::cerr << "format field is ignored as codec type is not video or audio" << std::endl;
        break;
      }
    }
    // === for video and subtitle ===
    decode_if_exists(yaml, "width", &params->width);
    decode_if_exists(yaml, "height", &params->height);
    // === for video ===
    // TODO: field_order, color_range, color_primaries, color_trc, color_space, chroma_location
    decode_if_exists(yaml, "sample_aspect_ratio", &params->sample_aspect_ratio);
    decode_if_exists(yaml, "video_delay", &params->video_delay);
    decode_if_exists(yaml, "framerate", &params->framerate);
    // === for audio ===
    if (const Node elem = yaml["ch_layout"]; elem) {
      const std::string ch_layout_str = elem.as<std::string>();
      if (const int ret = av_channel_layout_from_string(&params->ch_layout, ch_layout_str.c_str());
          ret < 0) {
        throw std::runtime_error(ch_layout_str + " is not a valid channel layout");
      }
    }
    decode_if_exists(yaml, "sample_rate", &params->sample_rate);
    decode_if_exists(yaml, "block_align", &params->block_align);
    decode_if_exists(yaml, "frame_size", &params->frame_size);
    decode_if_exists(yaml, "initial_padding", &params->initial_padding);
    decode_if_exists(yaml, "trailing_padding", &params->trailing_padding);
    decode_if_exists(yaml, "seek_preroll", &params->seek_preroll);
    return true;
  } catch (const std::runtime_error &error) {
    std::cerr << "Failed to decode CodecParameters from yaml: " << error.what() << std::endl;
    return false;
  }
}

} // namespace YAML