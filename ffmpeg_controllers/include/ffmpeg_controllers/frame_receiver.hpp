#ifndef FFMPEG_CONTROLLERS_FRAME_RECEIVER_HPP
#define FFMPEG_CONTROLLERS_FRAME_RECEIVER_HPP

#include <optional>
#include <stdexcept>
#include <utility>

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/frame.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class FrameReceiver
    : public ControllerBase<input_options::Subscribe<ffmpeg_pipeline_msgs::msg::Frame>,
                            output_options::Export<ffmpeg_cpp::Frame>> {
protected:
  OnGenerateReturn<output_options::Export<ffmpeg_cpp::Frame>>
  on_generate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
              const ffmpeg_pipeline_msgs::msg::Frame &input_msg) override {
    try {
      return {ControllerReturn::OK, ffmpeg_cpp::Frame::create(input_msg)};
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while updating frame: %s", error.what());
      return {ControllerReturn::ERROR, std::nullopt};
    }
  }
};

} // namespace ffmpeg_controllers

#endif