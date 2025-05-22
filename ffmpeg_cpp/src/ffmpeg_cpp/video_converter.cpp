#include <array>
#include <cstdint> // for UINT64_C macro suppressing errors from libavutil
#include <numeric> // for std::partial_sum()

extern "C" {
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

#include "internal.hpp"

namespace ffmpeg_cpp {

// ============================================
// VideoConverter - RAII wrapper for SwsContext
// ============================================

VideoConverter VideoConverter::null() { return VideoConverter(nullptr); }

VideoConverter VideoConverter::create(const int src_width, const int src_height,
                                      const std::string &src_format_name, //
                                      const int dst_width, const int dst_height,
                                      const std::string &dst_format_name) {
  VideoConverter converter(
      sws_getContext(src_width, src_height, av_get_pix_fmt(src_format_name.c_str()), //
                     dst_width, dst_height, av_get_pix_fmt(dst_format_name.c_str()), //
                     0, nullptr, nullptr, nullptr));
  if (!converter) {
    throw Error("VideoConverter::create(): Failed to create SwsContext");
  }
  return converter;
}

Frame VideoConverter::convert(const Frame &src_frame) {
  Frame dst_frame = Frame::create();
  if (const int ret = sws_scale_frame(get(), dst_frame.get(), src_frame.get()); ret < 0) {
    throw Error("VideoConverter::convert(): Failed to convert frame", ret);
  }
  return dst_frame;
}

int VideoConverter::src_width() const { return get_int64(get(), "srcw", 0); }

int VideoConverter::src_height() const { return get_int64(get(), "srch", 0); }

std::string VideoConverter::src_format_name() const {
  return get() ? to_string(get_pixel_format(get(), "src_format")) : "";
}

int VideoConverter::dst_width() const { return get_int64(get(), "dstw", 0); }

int VideoConverter::dst_height() const { return get_int64(get(), "dsth", 0); }

std::string VideoConverter::dst_format_name() const {
  return get() ? to_string(get_pixel_format(get(), "dst_format")) : "";
}

} // namespace ffmpeg_cpp