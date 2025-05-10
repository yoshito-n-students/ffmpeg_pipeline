#ifndef FFMPEG_CONTROLLERS_FRAME_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_FRAME_BROADCASTER_HPP

#include <optional>

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/frame.hpp>

namespace ffmpeg_controllers {

class FrameBroadcaster
    : public ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
                            output_options::Publish<ffmpeg_pipeline_msgs::msg::Frame>> {
private:
  using Base = ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
                              output_options::Publish<ffmpeg_pipeline_msgs::msg::Frame>>;

protected:
  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Name of interprocess topic to publish
    output_topic_ = "~/frame";

    return NodeReturn::SUCCESS;
  }

  OnGenerateReturn<output_options::Publish<ffmpeg_pipeline_msgs::msg::Frame>>
  on_generate(const rclcpp::Time &time, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Frame &input_frame) override {
    // Generate the message with the new packet
    return {ControllerReturn::OK, input_frame.to_msg(time)};
  }
};

} // namespace ffmpeg_controllers

#endif