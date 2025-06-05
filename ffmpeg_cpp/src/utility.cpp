#include <algorithm>
#include <cstdint>  // for UINT64_C macro suppressing errors from libavutil
#include <iterator> // for std::begin(), std::end()

extern "C" {
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

#include "internal.hpp"

namespace ffmpeg_cpp {

// =======
// Utility
// =======

void set_log_level(const std::string &level_name) {
  constexpr std::pair<const char *, int> name_level_pairs[] = {
      {"quiet", AV_LOG_QUIET},     {"panic", AV_LOG_PANIC},     {"fatal", AV_LOG_FATAL},
      {"error", AV_LOG_ERROR},     {"warning", AV_LOG_WARNING}, {"info", AV_LOG_INFO},
      {"verbose", AV_LOG_VERBOSE}, {"debug", AV_LOG_DEBUG},     {"trace", AV_LOG_TRACE}};
  const auto found_it =
      std::find_if(std::begin(name_level_pairs), std::end(name_level_pairs),
                   [&level_name](const auto &pair) { return pair.first == level_name; });
  if (found_it != std::end(name_level_pairs)) {
    av_log_set_level(found_it->second);
  } else {
    throw Error("set_log_level(): " + level_name + " is not a valid log level");
  }
}

std::string err2str(const int errnum) {
  char buf[AV_ERROR_MAX_STRING_SIZE];
  av_strerror(errnum, buf, sizeof(buf));
  return buf;
}

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

// Deleters
template <> void Deleter<AVPacket>::operator()(AVPacket *packet) const { av_packet_free(&packet); }
template <> void Deleter<AVFrame>::operator()(AVFrame *frame) const { av_frame_free(&frame); }
template <> void Deleter<AVDictionary>::operator()(AVDictionary *dict) const {
  av_dict_free(&dict);
}
template <> void Deleter<AVCodecParameters>::operator()(AVCodecParameters *params) const {
  avcodec_parameters_free(&params);
}
template <> void Deleter<AVFormatContext>::operator()(AVFormatContext *format_ctx) const {
  if (format_ctx->iformat) {
    // If the context is configured as an input device, close the input
    avformat_close_input(&format_ctx);
  } else if (format_ctx->oformat) {
    // If the context is configured as an output device, write the trailer and close the output
    av_write_trailer(format_ctx);
    if (!(format_ctx->oformat->flags & AVFMT_NOFILE)) {
      avio_closep(&format_ctx->pb);
    }
    avformat_free_context(format_ctx);
  } else {
    // If the context is not configured as an input or output device, just free found_it
    avformat_free_context(format_ctx);
  }
}
template <> void Deleter<AVCodecParserContext>::operator()(AVCodecParserContext *parser_ctx) const {
  av_parser_close(parser_ctx);
}
template <> void Deleter<AVCodecContext>::operator()(AVCodecContext *codec_ctx) const {
  avcodec_free_context(&codec_ctx);
}
template <> void Deleter<AVAudioFifo>::operator()(AVAudioFifo *fifo) const {
  av_audio_fifo_free(fifo);
}
template <> void Deleter<SwsContext>::operator()(SwsContext *sws_ctx) const {
  sws_freeContext(sws_ctx);
}
template <> void Deleter<SwrContext>::operator()(SwrContext *swr_ctx) const { swr_free(&swr_ctx); }

} // namespace ffmpeg_cpp