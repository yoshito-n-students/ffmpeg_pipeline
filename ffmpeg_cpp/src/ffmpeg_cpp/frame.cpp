extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/samplefmt.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

#include "internal.hpp"

namespace ffmpeg_cpp {

// ================================
// Frame - RAII wrapper for AVFrame
// ================================

Frame Frame::null() { return Frame(nullptr); }

Frame Frame::create() {
  Frame frame(av_frame_alloc());
  if (!frame) {
    throw Error("Frame::create(): Failed to allocate AVFrame");
  }
  return frame;
}

Frame::Frame(const Frame &other) : std::unique_ptr<AVFrame, Deleter<AVFrame>>() {
  if (other) {
    reset(av_frame_clone(other.get()));
    if (!get()) {
      throw Error("Frame::Frame(): Failed to clone frame");
    }
  } else {
    *this = null();
  }
}

Frame Frame::transfer_data() const {
  Frame dst = Frame::create();
  if (const int ret = av_hwframe_transfer_data(dst.get(), get(), 0); ret < 0) {
    throw Error("Frame::transfer_data(): Error transferring data", ret);
  }
  return dst;
}

std::string Frame::format_name() const {
  // There is no field in AVFrame to indicate the data type, so we use heuristics to determine it
  if (get()->width > 0 && get()->height > 0) {
    return av_get_pix_fmt_name(static_cast<AVPixelFormat>(get()->format));
  } else if (get()->nb_samples > 0) {
    return av_get_sample_fmt_name(static_cast<AVSampleFormat>(get()->format));
  }
  return "";
}

std::string Frame::ch_layout_str() const { return to_string(get()->ch_layout); }

ffmpeg_pipeline_msgs::msg::Frame Frame::to_msg(const rclcpp::Time &stamp) const {
  ffmpeg_pipeline_msgs::msg::Frame msg;
  msg.header.stamp = stamp;
  // Common fields for video and audio frames
  msg.format = format_name();
  {
    const int data_size = av_samples_get_buffer_size(
        nullptr, get()->ch_layout.nb_channels, get()->nb_samples,
        static_cast<AVSampleFormat>(get()->format), 1 /* 1: no_alignment */);
    if (data_size < 0) {
      throw Error("Frame::to_msg(): Failed to get buffer size", data_size);
    }
    // TODO: Warn or throw if data[1] is not null
    msg.data.assign(get()->data[0], get()->data[0] + data_size);
  }
  msg.pts = get()->pts;
  msg.pkt_dts = get()->pkt_dts;
  msg.time_base.num = get()->time_base.num;
  msg.time_base.den = get()->time_base.den;
  // Audio-specific fields
  msg.nb_samples = get()->nb_samples;
  msg.sample_rate = get()->sample_rate;
  msg.ch_layout = ch_layout_str();
  msg.duration = get()->duration;
  // Video-specific fields
  msg.width = get()->width;
  msg.height = get()->height;
  return msg;
}

} // namespace ffmpeg_cpp