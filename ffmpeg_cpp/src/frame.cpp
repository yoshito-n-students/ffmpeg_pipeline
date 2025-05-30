#include <cstring> // for std::memcpy()

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/samplefmt.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/frame.hpp>
#include <sensor_msgs/msg/image.hpp>

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
  // Common fields for any frame type
  frame->pts = msg.pts;
  frame->pkt_dts = msg.pkt_dts;
  frame->time_base.num = msg.time_base.num;
  frame->time_base.den = msg.time_base.den;
  // Fields specific to the frame type (copy in any case not to lose the data)
  // - For video frames
  frame->width = msg.width;
  frame->height = msg.height;
  // - For audio frames
  frame->nb_samples = msg.nb_samples;
  frame->sample_rate = msg.sample_rate;
  frame->ch_layout = to_channel_layout(msg.ch_layout);
  frame->duration = msg.duration;
  // Fields that change depending on the frame type
  if (msg.width > 0 && msg.height > 0) {
    // Video frame
    frame->format = av_get_pix_fmt(msg.format.c_str());
    if (const int linesize = av_image_get_linesize(static_cast<AVPixelFormat>(frame->format),
                                                   frame->width, 0 /* 0: first plane */);
        linesize >= 0) {
      frame->linesize[0] = linesize;
    } else {
      throw Error("Frame::create(): Failed to get image linesize", linesize);
    }
  } else if (msg.nb_samples > 0) {
    // Audio frame
    frame->format = av_get_sample_fmt(msg.format.c_str());
    if (const int ret = av_samples_get_buffer_size(
            &frame->linesize[0], frame->ch_layout.nb_channels, frame->nb_samples,
            static_cast<AVSampleFormat>(frame->format), 1 /* 1: no_alignment */);
        ret < 0) {
      throw Error("Frame::create(): Failed to get audio linesize", ret);
    }
  }
  return frame;
}

Frame Frame::create(const sensor_msgs::msg::Image &msg) {
  Frame frame = Frame::create(msg.data.data(), msg.data.size());
  // Assume msg.encoding is a ROS encoding name and convert it to FFmpeg pixel format.
  // If conversion fails, use msg.encoding directly as FFmpeg pixel format name.
  frame->format = av_get_pix_fmt(to_ffmpeg_format_name(msg.encoding).c_str());
  if (frame->format == AV_PIX_FMT_NONE) {
    frame->format = av_get_pix_fmt(msg.encoding.c_str());
  }
  frame->width = msg.width;
  frame->height = msg.height;
  frame->linesize[0] = msg.step;
  return frame;
}

Frame::Frame(const Frame &other) : UniquePtr<AVFrame>() {
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

ffmpeg_pipeline_msgs::msg::Frame Frame::to_frame_msg(const rclcpp::Time &stamp) const {
  ffmpeg_pipeline_msgs::msg::Frame msg;
  msg.header.stamp = stamp;
  // Common fields for any frame type
  msg.pts = get()->pts;
  msg.pkt_dts = get()->pkt_dts;
  msg.time_base.num = get()->time_base.num;
  msg.time_base.den = get()->time_base.den;
  // Fields specific to the frame type (copy in any case not to lose the data)
  // - For video frames
  msg.width = get()->width;
  msg.height = get()->height;
  // - For audio frames
  msg.nb_samples = get()->nb_samples;
  msg.sample_rate = get()->sample_rate;
  msg.ch_layout = ch_layout_str();
  msg.duration = get()->duration;
  // Fields that change depending on the frame type
  if (get()->width > 0 && get()->height > 0) {
    // Video frame
    msg.format = to_string(static_cast<AVPixelFormat>(get()->format));
    const int data_size =
        av_image_get_buffer_size(static_cast<AVPixelFormat>(get()->format), get()->width,
                                 get()->height, 1 /* 1: no_alignment */);
    if (data_size < 0) {
      throw Error("Frame::to_frame_msg(): Failed to get image buffer size", data_size);
    }
    msg.data.assign(get()->data[0], get()->data[0] + data_size);
  } else if (get()->nb_samples > 0) {
    // Audio frame
    msg.format = to_string(static_cast<AVSampleFormat>(get()->format));
    const int data_size = av_samples_get_buffer_size(
        nullptr, get()->ch_layout.nb_channels, get()->nb_samples,
        static_cast<AVSampleFormat>(get()->format), 1 /* 1: no_alignment */);
    if (data_size < 0) {
      throw Error("Frame::to_frame_msg(): Failed to get audio buffer size", data_size);
    }
    msg.data.assign(get()->data[0], get()->data[0] + data_size);
  }
  // TODO: Warn or throw if data[1] is not null
  return msg;
}

sensor_msgs::msg::Image Frame::to_image_msg(const rclcpp::Time &stamp,
                                            const std::string &encoding) const {
  sensor_msgs::msg::Image msg;
  msg.header.stamp = stamp;
  msg.encoding = encoding;
  if (get()->width > 0 && get()->height > 0) {
    msg.height = get()->height;
    msg.width = get()->width;
    const int data_size =
        av_image_get_buffer_size(static_cast<AVPixelFormat>(get()->format), get()->width,
                                 get()->height, 1 /* 1: no_alignment */);
    if (data_size < 0) {
      throw Error("Frame::to_image_msg(): Failed to get image buffer size", data_size);
    }
    msg.data.assign(get()->data[0], get()->data[0] + data_size);
    const int step = av_image_get_linesize(static_cast<AVPixelFormat>(get()->format), get()->width,
                                           0 /* 0: first plane */);
    if (step < 0) {
      throw Error("Frame::to_image_msg(): Failed to get image linesize", step);
    }
    msg.step = step;
  } else {
    // throw?
  }
  return msg;
}

} // namespace ffmpeg_cpp