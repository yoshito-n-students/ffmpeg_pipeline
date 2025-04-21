#include <string>

extern "C" {
#include <libswresample/swresample.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

#include "internal.hpp"

namespace ffmpeg_cpp {
// ============================================
// AudioConverter - RAII wrapper for SwrContext
// ============================================

AudioConverter::AudioConverter(const std::string &in_ch_layout_str,
                               const std::string &in_format_name, const int in_sample_rate,
                               const std::string &out_ch_layout_str,
                               const std::string &out_format_name, const int out_sample_rate)
    : swr_ctx_(nullptr, &free_context) {
  SwrContext *swr_ctx = nullptr;
  const AVChannelLayout in_ch_layout = to_channel_layout(in_ch_layout_str),
                        out_ch_layout = to_channel_layout(out_ch_layout_str);
  if (const int ret = swr_alloc_set_opts2(
          &swr_ctx,                                                                    //
          &out_ch_layout, av_get_sample_fmt(out_format_name.c_str()), out_sample_rate, //
          &in_ch_layout, av_get_sample_fmt(in_format_name.c_str()), in_sample_rate,    //
          0, nullptr);
      ret < 0) {
    throw Error("AudioConverter::AudioConverter(): Failed to create SwrContext", ret);
  }
  swr_ctx_.reset(swr_ctx);

  if (const int ret = swr_init(swr_ctx_.get()); ret < 0) {
    throw Error("AudioConverter::AudioConverter(): Failed to initialize SwrContext", ret);
  }
}

std::string AudioConverter::in_ch_layout_str() const {
  return to_string(get_channel_layout(swr_ctx_.get(), "in_chlayout"));
}

std::string AudioConverter::in_format_name() const {
  return to_string(get_sample_format(swr_ctx_.get(), "in_sample_fmt"));
}

std::int64_t AudioConverter::in_sample_rate() const {
  return get_int64(swr_ctx_.get(), "in_sample_rate", 0);
}

std::string AudioConverter::out_ch_layout_str() const {
  return to_string(get_channel_layout(swr_ctx_.get(), "out_chlayout"));
}

std::string AudioConverter::out_format_name() const {
  return to_string(get_sample_format(swr_ctx_.get(), "out_sample_fmt"));
}

std::int64_t AudioConverter::out_sample_rate() const {
  return get_int64(swr_ctx_.get(), "out_sample_rate", 0);
}

Frame AudioConverter::convert(const Frame &in_frame) {
  // Fill required fields in the output frame
  Frame out_frame;
  const AVChannelLayout out_ch_layout = get_channel_layout(swr_ctx_.get(), "out_chlayout");
  av_channel_layout_copy(&out_frame->ch_layout, &out_ch_layout);
  out_frame->format = get_sample_format(swr_ctx_.get(), "out_sample_fmt");
  out_frame->sample_rate = get_int64(swr_ctx_.get(), "out_sample_rate", 0);

  // Convert the input frame to the output format
  if (const int ret = swr_convert_frame(swr_ctx_.get(), out_frame.get(), in_frame.get()); ret < 0) {
    throw Error("AudioConverter::convert(): Failed to convert frame", ret);
  }

  return out_frame;
}

void AudioConverter::free_context(SwrContext *swr_ctx) { swr_free(&swr_ctx); }

} // namespace ffmpeg_cpp