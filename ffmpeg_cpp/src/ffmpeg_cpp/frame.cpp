#include <cstring> // for std::memcpy()

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

Frame Frame::create(const std::uint8_t *const data, const std::size_t size) {
  Frame frame = Frame::create();
  frame->buf[0] = av_buffer_alloc(size);
  if (!frame->buf[0]) {
    throw Error("Frame::create(): Failed to allocate AVBuffer for frame");
  }
  std::memcpy(frame->buf[0]->data, data, size);
  frame->data[0] = frame->buf[0]->data;
  return frame;
}

Frame Frame::create(const ffmpeg_pipeline_msgs::msg::Frame &msg) {
  Frame frame = Frame::create(msg.data.data(), msg.data.size());
  if (msg.width > 0 && msg.height > 0) {
    frame->format = av_get_pix_fmt(msg.format.c_str());
  } else if (msg.nb_samples > 0) {
    frame->format = av_get_sample_fmt(msg.format.c_str());
  }
  frame->pts = msg.pts;
  frame->pkt_dts = msg.pkt_dts;
  frame->time_base.num = msg.time_base.num;
  frame->time_base.den = msg.time_base.den;
  // Audio-specific fields
  frame->nb_samples = msg.nb_samples;
  frame->sample_rate = msg.sample_rate;
  frame->ch_layout = to_channel_layout(msg.ch_layout);
  frame->duration = msg.duration;
  // Video-specific fields
  frame->width = msg.width;
  frame->height = msg.height;
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
  if (get()) {
    // There is no field in AVFrame to indicate the data type, so we use heuristics to determine it
    if (get()->width > 0 && get()->height > 0) {
      return to_string(static_cast<AVPixelFormat>(get()->format));
    } else if (get()->nb_samples > 0) {
      return to_string(static_cast<AVSampleFormat>(get()->format));
    }
  }
  return "";
}

std::string Frame::ch_layout_str() const { return get() ? to_string(get()->ch_layout) : ""; }

ffmpeg_pipeline_msgs::msg::Frame Frame::to_msg(const rclcpp::Time &stamp) const {
  ffmpeg_pipeline_msgs::msg::Frame msg;
  msg.header.stamp = stamp;
  // Common fields for video and audio frames
  if (get()->width > 0 && get()->height > 0) {
    // Video frame
    msg.format = to_string(static_cast<AVPixelFormat>(get()->format));
    const int data_size =
        av_image_get_buffer_size(static_cast<AVPixelFormat>(get()->format), get()->width,
                                 get()->height, 1 /* 1: no_alignment */);
    if (data_size < 0) {
      throw Error("Frame::to_msg(): Failed to get image buffer size", data_size);
    }
    msg.data.assign(get()->data[0], get()->data[0] + data_size);
  } else if (get()->nb_samples > 0) {
    // Audio frame
    msg.format = to_string(static_cast<AVSampleFormat>(get()->format));
    const int data_size = av_samples_get_buffer_size(
        nullptr, get()->ch_layout.nb_channels, get()->nb_samples,
        static_cast<AVSampleFormat>(get()->format), 1 /* 1: no_alignment */);
    if (data_size < 0) {
      throw Error("Frame::to_msg(): Failed to get audio buffer size", data_size);
    }
    msg.data.assign(get()->data[0], get()->data[0] + data_size);
  }
  // TODO: Warn or throw if data[1] is not null
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