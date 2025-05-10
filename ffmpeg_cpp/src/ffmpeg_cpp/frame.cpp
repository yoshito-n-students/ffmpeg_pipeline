extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
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
  // TODO: Copy the data from the frame to the message
  return msg;
}

} // namespace ffmpeg_cpp