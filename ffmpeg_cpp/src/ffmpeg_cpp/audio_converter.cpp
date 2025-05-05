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

AudioConverter AudioConverter::null() { return AudioConverter(nullptr); }

AudioConverter AudioConverter::create(const std::string &in_ch_layout_str,
                                      const std::string &in_format_name, const int in_sample_rate,
                                      const std::string &out_ch_layout_str,
                                      const std::string &out_format_name,
                                      const int out_sample_rate) {
  SwrContext *swr_ctx = nullptr;
  const AVChannelLayout in_ch_layout = to_channel_layout(in_ch_layout_str),
                        out_ch_layout = to_channel_layout(out_ch_layout_str);
  if (const int ret = swr_alloc_set_opts2(
          &swr_ctx,                                                                    //
          &out_ch_layout, av_get_sample_fmt(out_format_name.c_str()), out_sample_rate, //
          &in_ch_layout, av_get_sample_fmt(in_format_name.c_str()), in_sample_rate,    //
          0, nullptr);
      ret < 0) {
    throw Error("AudioConverter::create(): Failed to create SwrContext", ret);
  }
  AudioConverter converter(swr_ctx);

  if (const int ret = swr_init(converter.get()); ret < 0) {
    throw Error("AudioConverter::create(): Failed to initialize SwrContext", ret);
  }
  return converter;
}

// Helper to convert to the channel layout used internally by libswresample
static AVChannelLayout to_swr_channel_layout(const AVChannelLayout &ch_layout) {
  if (ch_layout.order != AV_CHANNEL_ORDER_UNSPEC) {
    return ch_layout;
  } else {
    AVChannelLayout swr_ch_layout;
    av_channel_layout_default(&swr_ch_layout, ch_layout.nb_channels);
    return swr_ch_layout;
  }
}

std::string AudioConverter::in_ch_layout_str() const {
  return to_string(to_swr_channel_layout(get_channel_layout(get(), "in_chlayout")));
}

std::string AudioConverter::in_format_name() const {
  return to_string(get_sample_format(get(), "in_sample_fmt"));
}

int AudioConverter::in_sample_rate() const { return get_int64(get(), "in_sample_rate", 0); }

std::string AudioConverter::out_ch_layout_str() const {
  return to_string(to_swr_channel_layout(get_channel_layout(get(), "out_chlayout")));
}

std::string AudioConverter::out_format_name() const {
  return to_string(get_sample_format(get(), "out_sample_fmt"));
}

int AudioConverter::out_sample_rate() const { return get_int64(get(), "out_sample_rate", 0); }

Frame AudioConverter::convert(const Frame &_in_frame) {
  // Prepare the input frame with the modified channel layout
  Frame in_frame(_in_frame); // This is a shallow copy
  const AVChannelLayout swr_in_ch_layout = to_swr_channel_layout(in_frame->ch_layout);
  if (const int ret = av_channel_layout_copy(&in_frame->ch_layout, &swr_in_ch_layout); ret < 0) {
    throw Error("AudioConverter::convert(): Failed to set input channel layout", ret);
  }

  // Prepare the output frame with the required fields
  Frame out_frame = Frame::create();
  const AVChannelLayout swr_out_ch_layout =
      to_swr_channel_layout(get_channel_layout(get(), "out_chlayout"));
  if (const int ret = av_channel_layout_copy(&out_frame->ch_layout, &swr_out_ch_layout); ret < 0) {
    throw Error("AudioConverter::convert(): Failed to set output channel layout", ret);
  }
  out_frame->format = get_sample_format(get(), "out_sample_fmt");
  out_frame->sample_rate = get_int64(get(), "out_sample_rate", 0);

  // Convert the input frame to the output format
  if (const int ret = swr_convert_frame(get(), out_frame.get(), in_frame.get()); ret < 0) {
    throw Error("AudioConverter::convert(): Failed to convert frame", ret);
  }

  // Copy the timestamps from the input frame to the output frame
  out_frame->pts = in_frame->pts;
  out_frame->pkt_dts = in_frame->pkt_dts;

  return out_frame;
}

} // namespace ffmpeg_cpp