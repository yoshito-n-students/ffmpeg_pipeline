#ifndef FFMPEG_CONTROLLERS_IMAGE_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_IMAGE_BROADCASTER_HPP

#include <optional>
#include <utility>

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ffmpeg_controllers {

class ImageBroadcaster : public ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
                                               output_options::Publish<sensor_msgs::msg::Image>> {
private:
  using Base = ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
                              output_options::Publish<sensor_msgs::msg::Image>>;

protected:
  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Name of interprocess topic to publish
    output_topic_ = "~/image";

    return NodeReturn::SUCCESS;
  }

  OnGenerateReturn<output_options::Publish<sensor_msgs::msg::Image>>
  on_generate(const rclcpp::Time &time, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Frame &input_frame) override {
    OutputMessage msg;
    msg.header.stamp = time;
    msg.height = input_frame->height;
    msg.width = input_frame->width;
    msg.encoding = ffmpeg_cpp::to_ros_image_encoding(input_frame.format_name());
    msg.data.assign(input_frame->data[0],
                    input_frame->data[0] + input_frame->linesize[0] * input_frame->height);
    msg.step = input_frame->linesize[0];
    return {ControllerReturn::OK, std::move(msg)};
  }
};

} // namespace ffmpeg_controllers

#endif