#ifndef FFMPEG_CONTROLLERS_AUDIO_FIFO_FILTER_HPP
#define FFMPEG_CONTROLLERS_AUDIO_FIFO_FLITER_HPP

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_controllers {

class AudioFifoFilter : public FilterBase {
protected:
  // ===================
  // AS a lifecycle node
  // ===================

  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = FilterBase::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    try {
      ch_layout_str_ = get_node()->declare_parameter("channel_layout", "");
      nb_samples_ = get_node()->declare_parameter<int>("nb_samples");
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while getting parameter value: %s", error.what());
      return CallbackReturn::ERROR;
    }

    // Names of intraprocess read-only variables to be exported
    exported_state_interface_names_ = {"frame"};

    return NodeReturn::SUCCESS;
  }

  NodeReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
    // Activate the base class first
    if (const NodeReturn base_ret = FilterBase::on_activate(previous_state);
        base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Register the frame to state interface owned by this controller
    if (!set_state_from_pointer("frame", &frame_)) {
      return NodeReturn::ERROR;
    }

    // Reset the previous dts
    prev_dts_ = 0;

    return NodeReturn::SUCCESS;
  }

  NodeReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
    // Unregister the frame from state interface owned by this controller
    if (!set_state_from_pointer("frame", nullptr)) {
      return NodeReturn::ERROR;
    }

    // Deactivate the base class in the end
    return FilterBase::on_deactivate(previous_state);
  }

  // ===============
  // As a controller
  // ===============

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {input_name_ + "/frame"}};
  }

  ControllerReturn on_update(const rclcpp::Time & /*time*/,
                             const rclcpp::Duration & /*period*/) override {
    // Try to get the input frame owned by the hardware or other chained controller
    const ffmpeg_cpp::Frame *const in_frame = get_state_as_pointer<ffmpeg_cpp::Frame>("frame");
    if (!in_frame) {
      RCLCPP_WARN(get_logger(), "Failed to get input frame. Will skip this update.");
      return ControllerReturn::OK;
    }

    // Skip converting the frame if it is not new
    if ((*in_frame)->pkt_dts <= prev_dts_) {
      return ControllerReturn::OK;
    }

    try {
      // Ensure the FIFO is configured
      if (!fifo_.valid()) {
        fifo_ = ffmpeg_cpp::AudioFifo(
            // Keep the original channel layout if ch_layout_str_ is not set
            !ch_layout_str_.empty() ? ch_layout_str_ : in_frame->ch_layout_str(),
            in_frame->format_name(), (*in_frame)->sample_rate);
        RCLCPP_INFO(get_logger(), "Configured FIFO ([%s] %s @ %dHz)", fifo_.ch_layout_str().c_str(),
                    fifo_.format_name().c_str(), fifo_.sample_rate());
      }

      // Push the input frame to the FIFO
      fifo_.write(*in_frame);
      prev_dts_ = (*in_frame)->pkt_dts;

      // Update the output frame with the FIFO data if available
      if (ffmpeg_cpp::Frame frame = fifo_.read(nb_samples_); !frame.empty()) {
        frame->pts = (*in_frame)->pts;
        frame->pkt_dts = (*in_frame)->pkt_dts;
        frame_ = std::move(frame);
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while converting frames: %s", error.what());
      return ControllerReturn::ERROR;
    }

    return ControllerReturn::OK;
  }

protected:
  std::string ch_layout_str_;
  int nb_samples_;
  ffmpeg_cpp::AudioFifo fifo_;
  ffmpeg_cpp::Frame frame_;
  decltype(ffmpeg_cpp::Packet()->dts) prev_dts_;
};

} // namespace ffmpeg_controllers

#endif