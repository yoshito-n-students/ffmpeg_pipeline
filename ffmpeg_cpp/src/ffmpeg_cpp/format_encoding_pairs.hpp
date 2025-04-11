#ifndef FFMEPG_CPP_FORMAT_ENCODING_PAIRS_HPP
#define FFMEPG_CPP_FORMAT_ENCODING_PAIRS_HPP

#include <utility> // for std::pair<>

extern "C" {
#include <libavutil/pixfmt.h>
}

#include <sensor_msgs/image_encodings.hpp>

namespace ffmpeg_cpp {

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

} // namespace ffmpeg_cpp

#endif