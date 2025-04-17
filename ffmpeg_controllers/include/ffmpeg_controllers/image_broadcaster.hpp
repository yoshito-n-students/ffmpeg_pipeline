#ifndef FFMPEG_CONTROLLERS_IMAGE_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_IMAGE_BROADCASTER_HPP

#include <optional>

#include <ffmpeg_controllers/broadcaster_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ffmpeg_controllers {

class ImageBroadcaster : public BroadcasterBase<sensor_msgs::msg::Image> {
private:
  using Base = BroadcasterBase<sensor_msgs::msg::Image>;

protected:
  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Name of interprocess topic to be subscribed to
    topic_ = "~/image";

    return NodeReturn::SUCCESS;
  }

  NodeReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
    // Activate the base class first
    if (const NodeReturn base_ret = Base::on_activate(previous_state);
        base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Reset the previous dts
    prev_dts_ = 0;

    return NodeReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {input_name_ + "/frame"}};
  }

  std::optional<Message> on_update(const rclcpp::Time &time,
                                   const rclcpp::Duration & /*period*/) override {
    // Try to get the frame from the state interfaces
    const ffmpeg_cpp::Frame *const frame = get_state_as_pointer<ffmpeg_cpp::Frame>("frame");
    if (!frame) {
      RCLCPP_WARN(get_logger(), "Failed to get frame. Will skip this update.");
      return std::nullopt;
    }

    // Skip publishing if the frame is not new
    if ((*frame)->pkt_dts <= prev_dts_) {
      return std::nullopt;
    }

    // Generate the message with the new frame
    Message msg;
    msg.header.stamp = time;
    msg.height = (*frame)->height;
    msg.width = (*frame)->width;
    RCLCPP_INFO(get_logger(), "Image size: %dx%d, format: %s", msg.width, msg.height,
                frame->format_name().c_str());
    msg.encoding = ffmpeg_cpp::to_ros_image_encoding(frame->format_name());
    msg.data.assign((*frame)->data[0],
                    (*frame)->data[0] + (*frame)->linesize[0] * (*frame)->height);
    msg.step = (*frame)->linesize[0];
    prev_dts_ = (*frame)->pkt_dts;
    return msg;
  }

protected:
  decltype(ffmpeg_cpp::Frame()->pkt_dts) prev_dts_;
};

} // namespace ffmpeg_controllers

#endif