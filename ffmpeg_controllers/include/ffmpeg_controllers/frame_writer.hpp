#ifndef FFMPEG_CONTROLLERS_FRAME_WRITER_HPP
#define FFMPEG_CONTROLLERS_FRAME_WRITER_HPP

#include <optional>
#include <utility>

#include <ffmpeg_controllers/controller_base.hpp>

namespace ffmpeg_controllers {

class FrameWriter : public ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
                                          output_options::Write<ffmpeg_cpp::Frame>> {
protected:
  OnGenerateReturn<output_options::Write<ffmpeg_cpp::Frame>>
  on_generate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Frame &input_frame) override {
    return {ControllerReturn::OK, input_frame};
  }
};

} // namespace ffmpeg_controllers

#endif