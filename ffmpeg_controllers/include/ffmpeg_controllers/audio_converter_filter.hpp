#ifndef FFMPEG_CONTROLLERS_AUDIO_CONVERTER_FILTER_HPP
#define FFMPEG_CONTROLLERS_AUDIO_CONVERTER_FLITER_HPP

#include <optional>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class AudioConverterFilter : public ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
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
      out_ch_layout_str_ = get_user_parameter<std::string>("out_channel_layout", "");
      out_format_name_ = get_user_parameter<std::string>("out_format", "");
      out_sample_rate_ = get_user_parameter<int>("out_sample_rate", 0);
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
      // Ensure the converter is configured for the input frame
      if (!converter_) {
        converter_ = ffmpeg_cpp::AudioConverter::create(
            input_frame.ch_layout_str(), input_frame.format_name(), input_frame->sample_rate,
            // Keep the original channel layout, format and sample rate if out_xxx_ are not set
            !out_ch_layout_str_.empty() ? out_ch_layout_str_ : input_frame.ch_layout_str(),
            !out_format_name_.empty() ? out_format_name_ : input_frame.format_name(),
            out_sample_rate_ > 0 ? out_sample_rate_ : input_frame->sample_rate);
        RCLCPP_INFO(get_logger(), "Configured converter ([%s] %s @ %dHz -> [%s] %s @ %dHz)",
                    converter_.in_ch_layout_str().c_str(), converter_.in_format_name().c_str(),
                    converter_.in_sample_rate(), //
                    converter_.out_ch_layout_str().c_str(), converter_.out_format_name().c_str(),
                    converter_.out_sample_rate());
      }

      // Convert the input frame to the destination format
      ffmpeg_cpp::Frame output_frame = converter_.convert(input_frame);
      if (output_frame.empty()) {
        return {ControllerReturn::ERROR, std::nullopt};
      }

      return {ControllerReturn::OK, std::move(output_frame)};
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while converting frames: %s", error.what());
      return {ControllerReturn::ERROR, std::nullopt};
    }
  }

protected:
  std::string out_ch_layout_str_, out_format_name_;
  int out_sample_rate_;
  ffmpeg_cpp::AudioConverter converter_ = ffmpeg_cpp::AudioConverter::null();
};

} // namespace ffmpeg_controllers

#endif