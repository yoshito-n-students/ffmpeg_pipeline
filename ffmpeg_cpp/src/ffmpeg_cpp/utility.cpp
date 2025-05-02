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

std::string err2str(const int errnum) {
  char buf[AV_ERROR_MAX_STRING_SIZE];
  av_strerror(errnum, buf, sizeof(buf));
  return buf;
}

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
    // If the context is not configured as an input or output device, just free it
    avformat_free_context(format_ctx);
  }
}
template <> void Deleter<AVCodecParserContext>::operator()(AVCodecParserContext *parser_ctx) const {
  av_parser_close(parser_ctx);
}
template <> void Deleter<AVCodecContext>::operator()(AVCodecContext *codec_ctx) const {
  avcodec_free_context(&codec_ctx);
}

} // namespace ffmpeg_cpp