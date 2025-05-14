#ifndef FFMPEG_CONTROLLERS_AUDIO_FIFO_FILTER_HPP
#define FFMPEG_CONTROLLERS_AUDIO_FIFO_FLITER_HPP

#include <optional>
#include <stdexcept>
#include <string>

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class AudioFifoFilter : public ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
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
      ch_layout_str_ = get_user_parameter<std::string>("channel_layout", "");
      nb_samples_ = get_user_parameter<int>("nb_samples");
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
      // Ensure the FIFO is configured
      if (!fifo_) {
        fifo_ = ffmpeg_cpp::AudioFifo::create(
            // Keep the original channel layout if ch_layout_str_ is not set
            !ch_layout_str_.empty() ? ch_layout_str_ : input_frame.ch_layout_str(),
            input_frame.format_name(), input_frame->sample_rate);
        RCLCPP_INFO(get_logger(), "Configured FIFO ([%s] %s @ %dHz)", fifo_.ch_layout_str().c_str(),
                    fifo_.format_name().c_str(), fifo_.sample_rate());
      }

      // Push the input frame to the FIFO
      fifo_.write(input_frame);

      // Take the latest frame from the FIFO
      ffmpeg_cpp::Frame output_frame = ffmpeg_cpp::Frame::null();
      while (true) {
        if (ffmpeg_cpp::Frame incoming_frame = fifo_.read(nb_samples_); !incoming_frame.empty()) {
          if (!output_frame.empty()) {
            RCLCPP_WARN(get_logger(),
                        "Multiple frames available in the FIFO, discarding the previous one");
          }
          output_frame = std::move(incoming_frame); // Keep the latest frame
        } else {
          break; // No more frames available
        }
      }
      if (output_frame.empty()) {
        // No frames available
        return {ControllerReturn::OK, std::nullopt};
      }

      output_frame->pts = input_frame->pts;
      output_frame->pkt_dts = input_frame->pkt_dts;
      return {ControllerReturn::OK, std::move(output_frame)};
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while converting frames: %s", error.what());
      return {ControllerReturn::ERROR, std::nullopt};
    }
  }

protected:
  std::string ch_layout_str_;
  int nb_samples_;
  ffmpeg_cpp::AudioFifo fifo_ = ffmpeg_cpp::AudioFifo::null();
};

} // namespace ffmpeg_controllers

#endif