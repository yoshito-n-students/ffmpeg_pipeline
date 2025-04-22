#ifndef FFMPEG_CONTROLLERS_VIDEO_CONVERTER_FILTER_HPP
#define FFMPEG_CONTROLLERS_VIDEO_CONVERTER_FLITER_HPP

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_controllers {

class VideoConverterFilter : public FilterBase {
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
      dst_width_ = get_node()->declare_parameter("dst_width", 0);
      dst_height_ = get_node()->declare_parameter("dst_height", 0);
      dst_format_ = get_node()->declare_parameter<std::string>("dst_format");
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
    const ffmpeg_cpp::Frame *const src_frame = get_state_as_pointer<ffmpeg_cpp::Frame>("frame");
    if (!src_frame) {
      RCLCPP_WARN(get_logger(), "Failed to get input frame. Will skip this update.");
      return ControllerReturn::OK;
    }

    // Skip converting the frame if it is not new
    if ((*src_frame)->pkt_dts <= prev_dts_) {
      return ControllerReturn::OK;
    }

    try {
      // Ensure the converter is configured
      if (!converter_.valid()) {
        converter_ = ffmpeg_cpp::VideoConverter(
            (*src_frame)->width, (*src_frame)->height, src_frame->format_name(),
            // Keep the original size if dst_{width_, height_} are not set
            dst_width_ > 0 ? dst_width_ : (*src_frame)->width,
            dst_height_ > 0 ? dst_height_ : (*src_frame)->height, dst_format_);
        RCLCPP_INFO(get_logger(), "Configured converter ([%s] %zdx%zd -> [%s] %zdx%zd)",
                    converter_.src_format_name().c_str(), converter_.src_width(),
                    converter_.src_height(), //
                    converter_.dst_format_name().c_str(), converter_.dst_width(),
                    converter_.dst_height());
      }

      // Convert the frame to the destination format
      frame_ = converter_.convert(*src_frame);
      prev_dts_ = frame_->pkt_dts;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while decoding packet: %s", error.what());
      return ControllerReturn::ERROR;
    }

    return ControllerReturn::OK;
  }

protected:
  std::size_t dst_width_, dst_height_;
  std::string dst_format_;
  ffmpeg_cpp::VideoConverter converter_;
  ffmpeg_cpp::Frame frame_;
  decltype(ffmpeg_cpp::Packet()->dts) prev_dts_;
};

} // namespace ffmpeg_controllers

#endif