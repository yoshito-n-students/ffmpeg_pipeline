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

VideoConverter::VideoConverter(const std::size_t src_width, const std::size_t src_height,
                               const std::string &src_format_name, //
                               const std::size_t dst_width, const std::size_t dst_height,
                               const std::string &dst_format_name)
    : sws_ctx_(nullptr, &sws_freeContext) {
  sws_ctx_.reset(sws_getContext(src_width, src_height, av_get_pix_fmt(src_format_name.c_str()), //
                                dst_width, dst_height, av_get_pix_fmt(dst_format_name.c_str()), //
                                0, nullptr, nullptr, nullptr));
  if (!sws_ctx_) {
    throw Error("VideoConverter::VideoConverter(): Failed to create SwsContext");
  }
}

Frame VideoConverter::convert(const Frame &src_frame) {
  Frame dst_frame;
  if (const int ret = sws_scale_frame(sws_ctx_.get(), dst_frame.get(), src_frame.get()); ret < 0) {
    throw Error("VideoConverter::convert(): Failed to convert frame", ret);
  }
  return dst_frame;
}

std::vector<std::uint8_t> VideoConverter::convert_to_vector(const Frame &src_frame) {
  // Get the pixel format of the destination image
  const AVPixelFormat dst_format = get_pixel_format(sws_ctx_.get(), "dst_format");

  // Get the layout of the destination image
  // - linesize: bytes per line for each plane
  std::array<int, 4> dst_linesize;
  if (const int ret = av_image_fill_linesizes(dst_linesize.data(), dst_format, dst_width());
      ret < 0) {
    throw Error("VideoConverter::convert(): Failed to get destination linesizes", ret);
  }
  // - plane size: bytes per plane
  std::array<std::size_t, 4> dst_plane_size;
  if (const int ret =
          av_image_fill_plane_sizes(dst_plane_size.data(), dst_format, dst_height(),
                                    std::array<std::ptrdiff_t, 4>{dst_linesize[0], dst_linesize[1],
                                                                  dst_linesize[2], dst_linesize[3]}
                                        .data());
      ret < 0) {
    throw Error("VideoConverter::convert(): Failed to get destination plane sizes", ret);
  }
  // - data offset: bytes to the start of each plane
  std::array<std::ptrdiff_t, 4> dst_data_offset;
  std::partial_sum(dst_plane_size.begin(), dst_plane_size.end(), dst_data_offset.begin());

  // Convert the pixel data
  std::vector<std::uint8_t> dst_data(dst_data_offset[3]);
  sws_scale(sws_ctx_.get(),
            // src description
            src_frame->data, src_frame->linesize, 0, src_frame->height,
            // dst description
            std::array<std::uint8_t *const, 4>{
                dst_data.data(), dst_data.data() + dst_data_offset[0],
                dst_data.data() + dst_data_offset[1], dst_data.data() + dst_data_offset[2]}
                .data(),
            dst_linesize.data());

  return dst_data;
}

std::size_t VideoConverter::src_width() const { return get_int64(sws_ctx_.get(), "srcw", 0); }

std::size_t VideoConverter::src_height() const { return get_int64(sws_ctx_.get(), "srch", 0); }

std::string VideoConverter::src_format_name() const {
  return to_string(get_pixel_format(sws_ctx_.get(), "src_format"));
}

std::size_t VideoConverter::dst_width() const { return get_int64(sws_ctx_.get(), "dstw", 0); }

std::size_t VideoConverter::dst_height() const { return get_int64(sws_ctx_.get(), "dsth", 0); }

std::string VideoConverter::dst_format_name() const {
  return to_string(get_pixel_format(sws_ctx_.get(), "dst_format"));
}

} // namespace ffmpeg_cpp