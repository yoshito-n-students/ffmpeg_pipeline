#ifndef FFMEPG_CPP_INTERNAL_HPP
#define FFMEPG_CPP_INTERNAL_HPP

#include <cstdint>
#include <string>
#include <utility> // for std::pair<>

extern "C" {
#include <libavutil/avutil.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libavutil/pixfmt.h>
#include <libavutil/samplefmt.h>
}

#include <sensor_msgs/image_encodings.hpp>

namespace ffmpeg_cpp {

// ==============
// to std::string
// ==============

static inline std::string to_string(const AVMediaType type) {
  if (const char *name = av_get_media_type_string(type); name) {
    return name;
  } else {
    return "";
  }
}

static inline std::string to_string(const AVPixelFormat format) {
  if (const char *name = av_get_pix_fmt_name(format); name) {
    return name;
  } else {
    return "";
  }
}

static inline std::string to_string(const AVSampleFormat format) {
  if (const char *name = av_get_sample_fmt_name(format); name) {
    return name;
  } else {
    return "";
  }
}

static inline std::string to_string(const AVChannelLayout &ch_layout) {
  char buf[256];
  if (const int len = av_channel_layout_describe(&ch_layout, buf, sizeof(buf)); len >= 0) {
    return std::string(buf, len);
  } else {
    return "";
  }
}

// ================
// from std::string
// ================

static inline AVMediaType to_media_type(const std::string &str) {
  if (str == av_get_media_type_string(AVMEDIA_TYPE_VIDEO)) {
    return AVMEDIA_TYPE_VIDEO;
  } else if (str == av_get_media_type_string(AVMEDIA_TYPE_AUDIO)) {
    return AVMEDIA_TYPE_AUDIO;
  } else if (str == av_get_media_type_string(AVMEDIA_TYPE_DATA)) {
    return AVMEDIA_TYPE_DATA;
  } else if (str == av_get_media_type_string(AVMEDIA_TYPE_SUBTITLE)) {
    return AVMEDIA_TYPE_SUBTITLE;
  } else if (str == av_get_media_type_string(AVMEDIA_TYPE_ATTACHMENT)) {
    return AVMEDIA_TYPE_ATTACHMENT;
  } else if (str == av_get_media_type_string(AVMEDIA_TYPE_NB)) {
    return AVMEDIA_TYPE_NB;
  } else {
    return AVMEDIA_TYPE_UNKNOWN;
  }
}

static inline AVChannelLayout to_channel_layout(const std::string &str) {
  AVChannelLayout ch_layout;
  if (const int ret = av_channel_layout_from_string(&ch_layout, str.c_str()); ret < 0) {
    ch_layout.nb_channels = 0; // make it invalid if parsing fails
  }
  return ch_layout;
}

static inline const AVCodec *find_codec(const std::string &name) {
  void *iterate_ctx = nullptr;
  while (true) {
    if (const AVCodec *const codec = av_codec_iterate(&iterate_ctx);
        codec && avcodec_get_name(codec->id) == name) {
      return codec;
    } else if (!codec) {
      return nullptr;
    }
  }
}

// ===========
// get options
// ===========

static inline AVPixelFormat get_pixel_format(const void *obj, const std::string &key) {
  AVPixelFormat format;
  if (const int ret = av_opt_get_pixel_fmt(const_cast<void *>(obj), key.c_str(), 0, &format);
      ret >= 0) {
    return format;
  } else {
    return AV_PIX_FMT_NONE;
  }
}

static inline AVSampleFormat get_sample_format(const void *obj, const std::string &key) {
  AVSampleFormat format;
  if (const int ret = av_opt_get_sample_fmt(const_cast<void *>(obj), key.c_str(), 0, &format);
      ret >= 0) {
    return format;
  } else {
    return AV_SAMPLE_FMT_NONE;
  }
}

static inline AVChannelLayout get_channel_layout(const void *obj, const std::string &key) {
  AVChannelLayout ch_layout;
  if (const int ret = av_opt_get_chlayout(const_cast<void *>(obj), key.c_str(), 0, &ch_layout);
      ret < 0) {
    ch_layout.nb_channels = 0; // make it invalid if parsing fails
  }
  return ch_layout;
}

static inline std::int64_t get_int64(const void *obj, const std::string &key,
                                     const std::int64_t fallback_value) {
  std::int64_t value;
  if (const int ret = av_opt_get_int(const_cast<void *>(obj), key.c_str(), 0, &value); ret >= 0) {
    return value;
  } else {
    return fallback_value;
  }
}

} // namespace ffmpeg_cpp

#endif