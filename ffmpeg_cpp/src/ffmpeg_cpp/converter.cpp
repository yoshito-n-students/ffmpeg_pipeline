#include <array>
#include <cstdint> // for UINT64_C macro suppressing errors from libavutil
#include <numeric> // for std::partial_sum()

extern "C" {
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_cpp {

// =======================================
// Converter - RAII wrapper for SwsContext
// =======================================

Converter::Converter()
    : sws_ctx_(nullptr, &sws_freeContext), width_(0), height_(0), src_format_(AV_PIX_FMT_NONE),
      dst_format_(AV_PIX_FMT_NONE) {}

Converter::Converter(const std::size_t width, const std::size_t height,
                     const std::string &src_format_name, const std::string &dst_format_name)
    : sws_ctx_(nullptr, &sws_freeContext), width_(width), height_(height),
      src_format_(av_get_pix_fmt(src_format_name.c_str())),
      dst_format_(av_get_pix_fmt(dst_format_name.c_str())) {
  sws_ctx_.reset(sws_getContext(width_, height_, src_format_, width_, height_, dst_format_,
                                SWS_BILINEAR, nullptr, nullptr, nullptr));
  if (!sws_ctx_) {
    throw Error("Converter::reconfigure(): Failed to create SwsContext");
  }
}

bool Converter::is_supported(const std::size_t width, const std::size_t height,
                             const std::string &src_format_name,
                             const std::string &dst_format_name) const {
  return sws_ctx_ && (width_ == width) && (height_ == height) &&
         (src_format_ == av_get_pix_fmt(src_format_name.c_str())) &&
         (dst_format_ == av_get_pix_fmt(dst_format_name.c_str()));
}

void Converter::convert(const Frame &src_frame, std::vector<std::uint8_t> *const dst_data) {
  if (!sws_ctx_) {
    throw Error("Converter::convert(): Converter context is not configured");
  }

  // Get the layout of the destination image
  // - linesize: bytes per line for each plane
  std::array<int, 4> dst_linesize;
  if (const int ret = av_image_fill_linesizes(dst_linesize.data(), dst_format_, width_); ret < 0) {
    throw Error("Converter::convert(): Failed to get destination linesizes", ret);
  }
  // - plane size: bytes per plane
  std::array<std::size_t, 4> dst_plane_size;
  if (const int ret =
          av_image_fill_plane_sizes(dst_plane_size.data(), dst_format_, height_,
                                    std::array<std::ptrdiff_t, 4>{dst_linesize[0], dst_linesize[1],
                                                                  dst_linesize[2], dst_linesize[3]}
                                        .data());
      ret < 0) {
    throw Error("Converter::convert(): Failed to get destination plane sizes", ret);
  }
  // - data offset: bytes to the start of each plane
  std::array<std::ptrdiff_t, 4> dst_data_offset;
  std::partial_sum(dst_plane_size.begin(), dst_plane_size.end(), dst_data_offset.begin());

  // Resize the destination buffer to the sum of the plane sizes
  dst_data->resize(dst_data_offset[3]);

  // Convert the pixel data
  sws_scale(sws_ctx_.get(),
            // src description
            src_frame->data, src_frame->linesize, 0, src_frame->height,
            // dst description
            std::array<std::uint8_t *const, 4>{
                dst_data->data(), dst_data->data() + dst_data_offset[0],
                dst_data->data() + dst_data_offset[1], dst_data->data() + dst_data_offset[2]}
                .data(),
            dst_linesize.data());
}

std::string Converter::src_format_name() const { return av_get_pix_fmt_name(src_format_); }

std::string Converter::dst_format_name() const { return av_get_pix_fmt_name(dst_format_); }

} // namespace ffmpeg_cpp