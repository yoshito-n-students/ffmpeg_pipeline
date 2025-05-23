#ifndef FFMPEG_CONTROLLERS_IMAGE_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_IMAGE_BROADCASTER_HPP

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ffmpeg_controllers {

class ImageBroadcaster : public ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
                                               output_options::Publish<sensor_msgs::msg::Image>> {
protected:
  OnGenerateReturn<output_options::Publish<sensor_msgs::msg::Image>>
  on_generate(const rclcpp::Time &time, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Frame &input_frame) override {
    // TODO: Make it possible to specify the value of encoding by the parameter
    // 1. ROS format: to_ros_image_encoding(frame.format_name())
    // 2. ffmpeg format: frame.format_name()
    // 3. User specified?
    return {ControllerReturn::OK, input_frame.to_image_msg(time, ffmpeg_cpp::to_ros_image_encoding(
                                                                     input_frame.format_name()))};
  }
};

} // namespace ffmpeg_controllers

#endif