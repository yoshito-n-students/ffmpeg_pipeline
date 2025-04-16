#ifndef FFMPEG_CONTROLLERS_FRAME_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_FRAME_BROADCASTER_HPP

#include <optional>
#include <stdexcept>
#include <string>

#include <ffmpeg_controllers/frame_broadcaster_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/frame.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class FrameBroadcaster : public FrameBroadcasterBase<ffmpeg_pipeline_msgs::msg::Frame> {
private:
  using Base = FrameBroadcasterBase<ffmpeg_pipeline_msgs::msg::Frame>;

public:
  FrameBroadcaster() : Base(/* default_input_name = */ "input", /* topic = */ "~/frame") {}

  std::optional<Message> on_update(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/,
                                   const ffmpeg_cpp::Frame &frame) override {
    try {
      // Build the image message
      Message msg;
      msg.header.stamp.sec = frame->pkt_dts / 1'000'000;
      msg.header.stamp.nanosec = (frame->pkt_dts % 1'000'000) * 1'000;
      // TODO: fill data field
      // msg.data.assign(frame->data[0], frame->linesize[0] * frame->height);
      return msg;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while decoding packet: %s", error.what());
      return std::nullopt;
    }
  }
};

} // namespace ffmpeg_controllers

#endif