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

// ==============================================
// AVPixelFormat <-> sensor_msgs::image_encodings
// ==============================================

static constexpr std::pair<AVPixelFormat, const char *> format_encoding_pairs[] = {
    // RGB formats
    {AV_PIX_FMT_RGB24, sensor_msgs::image_encodings::RGB8},
    {AV_PIX_FMT_RGBA, sensor_msgs::image_encodings::RGBA8},
    {AV_PIX_FMT_RGB48, sensor_msgs::image_encodings::RGB16},
    {AV_PIX_FMT_RGBA64, sensor_msgs::image_encodings::RGBA16},
    // BGR formats
    {AV_PIX_FMT_BGR24, sensor_msgs::image_encodings::BGR8},
    {AV_PIX_FMT_BGRA, sensor_msgs::image_encodings::BGRA8},
    {AV_PIX_FMT_BGR48, sensor_msgs::image_encodings::BGR16},
    {AV_PIX_FMT_BGRA64, sensor_msgs::image_encodings::BGRA16},
    // Grayscale formats
    {AV_PIX_FMT_GRAY8, sensor_msgs::image_encodings::MONO8},
    {AV_PIX_FMT_GRAY16, sensor_msgs::image_encodings::MONO16},
    // bayer formats
    {AV_PIX_FMT_BAYER_RGGB8, sensor_msgs::image_encodings::BAYER_RGGB8},
    {AV_PIX_FMT_BAYER_RGGB16, sensor_msgs::image_encodings::BAYER_RGGB16},
    {AV_PIX_FMT_BAYER_BGGR8, sensor_msgs::image_encodings::BAYER_BGGR8},
    {AV_PIX_FMT_BAYER_BGGR16, sensor_msgs::image_encodings::BAYER_BGGR16},
    {AV_PIX_FMT_BAYER_GRBG8, sensor_msgs::image_encodings::BAYER_GRBG8},
    {AV_PIX_FMT_BAYER_GRBG16, sensor_msgs::image_encodings::BAYER_GRBG16},
    {AV_PIX_FMT_BAYER_GBRG8, sensor_msgs::image_encodings::BAYER_GBRG8},
    {AV_PIX_FMT_BAYER_GBRG16, sensor_msgs::image_encodings::BAYER_GBRG16},
    // YUV formats
    {AV_PIX_FMT_UYVY422, sensor_msgs::image_encodings::UYVY},
    {AV_PIX_FMT_YUYV422, sensor_msgs::image_encodings::YUYV},
    {AV_PIX_FMT_NV21, sensor_msgs::image_encodings::NV21},
    {AV_PIX_FMT_NV24, sensor_msgs::image_encodings::NV24}};

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

static inline AVChannelLayout to_channel_layout(const std::string &str) {
  AVChannelLayout ch_layout;
  if (const int ret = av_channel_layout_from_string(&ch_layout, str.c_str()); ret < 0) {
    ch_layout.nb_channels = 0; // make it invalid if parsing fails
  }
  return ch_layout;
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