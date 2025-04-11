#include <algorithm>
#include <iterator> // for std::begin(), std::end()
#include <numeric>  // for std::partial_sum()

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace ffmpeg_cpp {

// =======
// Utility
// =======

std::string err2str(const int errnum) {
  char buf[AV_ERROR_MAX_STRING_SIZE];
  av_strerror(errnum, buf, sizeof(buf));
  return buf;
}

constexpr std::pair<AVPixelFormat, const char *> format_encoding_pairs[] = {
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

std::string to_ros_image_encoding(const std::string &ffmpeg_format_name) {
  const AVPixelFormat ffmpeg_format = av_get_pix_fmt(ffmpeg_format_name.c_str());
  const auto found_it =
      std::find_if(std::begin(format_encoding_pairs), std::end(format_encoding_pairs),
                   [ffmpeg_format](const auto &pair) { return pair.first == ffmpeg_format; });
  return found_it != std::end(format_encoding_pairs) ? found_it->second : "";
}

std::string to_ffmpeg_format_name(const std::string &ros_image_encoding) {
  const auto found_it = std::find_if(
      std::begin(format_encoding_pairs), std::end(format_encoding_pairs),
      [&ros_image_encoding](const auto &pair) { return pair.second == ros_image_encoding; });
  return found_it != std::end(format_encoding_pairs) ? av_get_pix_fmt_name(found_it->first) : "";
}

} // namespace ffmpeg_cpp