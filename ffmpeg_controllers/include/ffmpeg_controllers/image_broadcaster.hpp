#ifndef FFMPEG_CONTROLLERS_IMAGE_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_IMAGE_BROADCASTER_HPP

#include <optional>
#include <stdexcept>
#include <string>

#include <ffmpeg_controllers/frame_broadcaster_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ffmpeg_controllers {

class ImageBroadcaster : public FrameBroadcasterBase<sensor_msgs::msg::Image> {
private:
  using Base = FrameBroadcasterBase<sensor_msgs::msg::Image>;

public:
  ImageBroadcaster() : Base(/* default_sensor_name = */ "camera", /* topic = */ "~/image") {}

  CallbackReturn on_init() override {
    // Call the base class on_init
    if (const auto ret = Base::on_init(); ret != CallbackReturn::SUCCESS) {
      return ret;
    }

    // Load additional parameters
    dst_encoding_ =
        get_node()->declare_parameter("dst_encoding", sensor_msgs::image_encodings::BGR8);
    dst_format_name_ = ffmpeg_cpp::to_ffmpeg_format_name(dst_encoding_);
    if (dst_format_name_.empty()) {
      RCLCPP_ERROR(get_logger(), "Unsupported destination encoding: %s", dst_encoding_.c_str());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  std::optional<Message> on_update(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/,
                                   const ffmpeg_cpp::Frame &frame) override {
    try {
      // Ensure the converter is configured for this frame
      if (!converter_.valid()) {
        converter_ = ffmpeg_cpp::Converter(frame->width, frame->height, frame.format_name(),
                                           dst_format_name_);
        RCLCPP_INFO(get_logger(), "Configured converter (src: %s, dst: %s, size: %zdx%zd)",
                    converter_.src_format_name().c_str(), converter_.dst_format_name().c_str(),
                    converter_.width(), converter_.height());
      }

      // Build the image message
      Message msg;
      msg.header.stamp.sec = frame->pts / 1'000'000;
      msg.header.stamp.nanosec = (frame->pts % 1'000'000) * 1'000;
      msg.height = frame->height;
      msg.width = frame->width;
      msg.encoding = dst_encoding_;
      converter_.convert(frame, &msg.data);
      msg.step = msg.data.size() / frame->height;
      return msg;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while decoding packet: %s", error.what());
      return std::nullopt;
    }
  }

protected:
  std::string dst_format_name_, dst_encoding_;
  ffmpeg_cpp::Converter converter_;
};

} // namespace ffmpeg_controllers

#endif