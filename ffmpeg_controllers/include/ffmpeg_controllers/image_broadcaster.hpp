#ifndef FFMPEG_CONTROLLERS_IMAGE_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_IMAGE_BROADCASTER_HPP

#include <optional>
#include <stdexcept>
#include <string>

#include <ffmpeg_controllers/broadcaster_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ffmpeg_controllers {

class ImageBroadcaster : public BroadcasterBase<sensor_msgs::msg::Image> {
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
                                   const std::string &codec_name,
                                   const ffmpeg_cpp::Packet &packet) override {
    try {
      // Ensure the decoder is configured for the codec
      if (!decoder_.valid()) {
        decoder_ = ffmpeg_cpp::Decoder(codec_name);
        RCLCPP_INFO(get_logger(), "Configured decoder (codec: %s, hw: %s)",
                    decoder_.codec_name().c_str(), decoder_.hw_device_type().c_str());
      }

      // Put the compressed data into the decoder
      decoder_.send_packet(packet);

      // Extract as many frames as possible from the decoder and keep only the latest frame.
      // According to the ffmpeg's reference, there should be only one frame per video packet
      // so no frames should be dropped.
      ffmpeg_cpp::Frame frame;
      while (true) {
        ffmpeg_cpp::Frame tmp_frame = decoder_.receive_frame();
        if (!tmp_frame.empty()) {
          frame = std::move(tmp_frame); // Keep the latest frame
        } else {
          break; // No more frames available
        }
      }
      if (frame.empty()) {
        RCLCPP_WARN(get_logger(), "No frames available after decoding packet");
        return std::nullopt;
      }

      // If the frame data is in a hardware device,
      // transfer the data to the CPU-accessible memory before conversion
      if (frame.is_hw_frame()) {
        frame = frame.transfer_data();
      }

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
      msg.header.stamp.sec = packet->pts / 1'000'000;
      msg.header.stamp.nanosec = (packet->pts % 1'000'000) * 1'000;
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

  ffmpeg_cpp::Decoder decoder_;
  ffmpeg_cpp::Converter converter_;
};

} // namespace ffmpeg_controllers

#endif