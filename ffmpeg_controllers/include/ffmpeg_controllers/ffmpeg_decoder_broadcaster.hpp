#ifndef FFMPEG_CONTROLLERS_FFMPEG_DECODER_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_FFMPEG_DECODER_BROADCASTER_HPP

#include <algorithm>
#include <memory>
#include <optional>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace ffmpeg_controllers {

class FFmpegDecoderBroadcaster : public controller_interface::ControllerInterface {
public:
  CallbackReturn on_init() override {
    // Load parameters
    sensor_name_ = get_node()->declare_parameter("sensor_name", "camera");
    dst_encoding_ =
        get_node()->declare_parameter("dst_encoding", sensor_msgs::image_encodings::BGR8);
    dst_format_name_ = ffmpeg_cpp::to_ffmpeg_format_name(dst_encoding_);
    if (dst_format_name_.empty()) {
      RCLCPP_ERROR(get_logger(), "Unsupported destination encoding: %s", dst_encoding_.c_str());
      return CallbackReturn::ERROR;
    }

    prev_pts_ = 0;

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // Configure the publisher
      underlying_publisher_ = get_node()->create_publisher<sensor_msgs::msg::Image>(
          "~/image", rclcpp::SystemDefaultsQoS());
      async_publisher_ =
          std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>>(
              underlying_publisher_);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Error while creating publishers: %s", error.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    // No command interfaces needed as this controller is only for broadcasting
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    // Request "foo_camera/packet" state interface
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {sensor_name_ + "/codec", sensor_name_ + "/packet"}};
  }

  controller_interface::return_type update(const rclcpp::Time & /*time*/,
                                           const rclcpp::Duration & /*period*/) override {
    // Try to get the packet and codec name from the state interfaces
    const auto packet = get_state_as_pointer<ffmpeg_cpp::Packet>(sensor_name_, "packet");
    const auto codec_name = get_state_as_pointer<std::string>(sensor_name_, "codec");
    if (!packet || !codec_name) {
      RCLCPP_WARN(get_logger(), "Failed to get packet or codec name. Will skip this update.");
      return controller_interface::return_type::OK;
    }

    // Skip publishing if the packet is not new
    if ((*packet)->pts <= prev_pts_) {
      return controller_interface::return_type::OK;
    }

    try {
      // Ensure the decoder is configured for the codec
      if (!decoder_.is_supported(*codec_name)) {
        decoder_.reconfigure(*codec_name);
        RCLCPP_INFO(get_logger(), "Configured decoder (codec: %s, hw: %s)",
                    decoder_.codec_name().c_str(), decoder_.hw_device_type().c_str());
      }

      // Put the compressed data into the decoder
      decoder_.send_packet(*packet);

      // Repeatedly receive uncompressed frames from the decoder
      ffmpeg_cpp::Frame frame;
      while (decoder_.receive_frame(&frame)) {
        // If the frame data is in a hardware device,
        // transfer the data to the CPU-accessible memory before conversion
        if (frame.is_hw_frame()) {
          ffmpeg_cpp::Frame sw_frame;
          frame.transfer_data(&sw_frame);
          frame = std::move(sw_frame);
        }

        // Ensure the converter is configured for this frame
        if (const auto src_format_name = frame.format_name(); !converter_.is_supported(
                frame->width, frame->height, src_format_name, dst_format_name_)) {
          converter_.reconfigure(frame->width, frame->height, src_format_name, dst_format_name_);
          RCLCPP_INFO(get_logger(), "Configured converter (src: %s, dst: %s, size: %dx%d)",
                      src_format_name.c_str(), dst_format_name_.c_str(), frame->width,
                      frame->height);
        }

        // Build and trigger publishing the image message
        if (async_publisher_->trylock()) {
          // Fill the message properties
          async_publisher_->msg_.header.stamp.sec = (*packet)->pts / 1'000'000;
          async_publisher_->msg_.header.stamp.nanosec = ((*packet)->pts % 1'000'000) * 1'000;
          async_publisher_->msg_.height = frame->height;
          async_publisher_->msg_.width = frame->width;
          async_publisher_->msg_.encoding = dst_encoding_;
          async_publisher_->msg_.step = 3 * frame->width;
          converter_.convert(frame, &async_publisher_->msg_.data);
          // Trigger the message to be published
          async_publisher_->unlockAndPublish();
          prev_pts_ = (*packet)->pts;
        }
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while decoding packet: %s", error.what());
      return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
  }

protected:
  rclcpp::Logger get_logger() const { return get_node()->get_logger(); }

  // Read the value from the state_interface specified by prefix_name and interface_name,
  // cast it to a pointer type, and return it. Or return nullptr on failure.
  template <typename T>
  const T *get_state_as_pointer(const std::string &prefix_name,
                                const std::string &iface_name) const {
    // Find the state interface with the given keys
    const auto iface_it =
        std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto &iface) {
          return iface.get_prefix_name() == prefix_name && iface.get_interface_name() == iface_name;
        });
    if (iface_it == state_interfaces_.end()) {
      return nullptr;
    }

    // Try to read the raw state value from the interface.
    // Due to the limitations of hardware_interface::StateInterface,
    // the value is stored as a double.
    const auto double_value = iface_it->template get_optional<double>();
    if (double_value == std::nullopt) {
      return nullptr;
    }

    // Reconstruct the pointer from the raw state value.
    // The double type has 53 bits of precision, while the Linux user memory space has 47 bits,
    // so the former can be safely converted to the latter.
    return reinterpret_cast<const T *>(static_cast<std::uintptr_t>(*double_value));
  }

protected:
  std::string sensor_name_, dst_format_name_, dst_encoding_;

  ffmpeg_cpp::Decoder decoder_;
  ffmpeg_cpp::Converter converter_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr underlying_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Image>> async_publisher_;

  decltype(ffmpeg_cpp::Packet()->pts) prev_pts_;
};

} // namespace ffmpeg_controllers

#endif