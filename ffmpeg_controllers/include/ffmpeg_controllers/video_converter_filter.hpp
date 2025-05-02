#ifndef FFMPEG_CONTROLLERS_VIDEO_CONVERTER_FILTER_HPP
#define FFMPEG_CONTROLLERS_VIDEO_CONVERTER_FLITER_HPP

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class VideoConverterFilter : public ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
                                                   output_options::Export<ffmpeg_cpp::Frame>> {
private:
  using Base = ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
                              output_options::Export<ffmpeg_cpp::Frame>>;

protected:
  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    try {
      dst_width_ = get_user_parameter<int>("dst_width", 0);
      dst_height_ = get_user_parameter<int>("dst_height", 0);
      dst_format_ = get_user_parameter<std::string>("dst_format", "");
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while getting parameter value: %s", error.what());
      return NodeReturn::ERROR;
    }

    return NodeReturn::SUCCESS;
  }

  OnGenerateReturn<output_options::Export<ffmpeg_cpp::Frame>>
  on_generate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Frame &input_frame) override {
    try {
      // Ensure the converter is configured
      if (!converter_) {
        converter_ = ffmpeg_cpp::VideoConverter(
            input_frame->width, input_frame->height, input_frame.format_name(),
            // Keep the original size and pixel format if dst_{width_, height_, format_} are not set
            dst_width_ > 0 ? dst_width_ : input_frame->width,
            dst_height_ > 0 ? dst_height_ : input_frame->height,
            !dst_format_.empty() ? dst_format_ : input_frame.format_name());
        RCLCPP_INFO(get_logger(), "Configured converter ([%s] %zdx%zd -> [%s] %zdx%zd)",
                    converter_.src_format_name().c_str(), converter_.src_width(),
                    converter_.src_height(), //
                    converter_.dst_format_name().c_str(), converter_.dst_width(),
                    converter_.dst_height());
      }

      // Convert the frame to the destination format
      ffmpeg_cpp::Frame output_frame = converter_.convert(input_frame);
      return {ControllerReturn::OK, std::move(output_frame)};
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while decoding packet: %s", error.what());
      return {ControllerReturn::ERROR, std::nullopt};
    }
  }

protected:
  std::size_t dst_width_, dst_height_;
  std::string dst_format_;
  ffmpeg_cpp::VideoConverter converter_;
};

} // namespace ffmpeg_controllers

#endif