#include <algorithm>
#include <cstdint>  // for UINT64_C macro suppressing errors from libavutil
#include <iterator> // for std::begin(), std::end()

extern "C" {
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

#include "format_encoding_pairs.hpp"

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

} // namespace ffmpeg_cpp