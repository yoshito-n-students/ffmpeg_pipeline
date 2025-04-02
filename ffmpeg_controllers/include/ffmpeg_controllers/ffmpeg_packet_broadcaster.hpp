#ifndef FFMPEG_CONTROLLERS_FFMPEG_PACKET_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_FFMPEG_PACKET_BROADCASTER_HPP

#include <algorithm>
#include <memory>
#include <optional>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ffmpeg_controllers {

class FFmpegPacketBroadcaster : public controller_interface::ControllerInterface {
public:
  CallbackReturn on_init() override {
    // Load parameters
    sensor_name_ = get_node()->declare_parameter("sensor_name", "camera");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // Configure the publisher
      underlying_publisher_ = get_node()->create_publisher<sensor_msgs::msg::CompressedImage>(
          "~/packet", rclcpp::SystemDefaultsQoS());
      async_publisher_ =
          std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::CompressedImage>>(
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

    if (async_publisher_->trylock()) {
      // Transfer data from the packet to the message
      async_publisher_->msg_.header.stamp.sec = (*packet)->pts / 1'000'000;
      async_publisher_->msg_.header.stamp.nanosec = ((*packet)->pts % 1'000'000) * 1'000;
      async_publisher_->msg_.format = *codec_name;
      async_publisher_->msg_.data.assign((*packet)->data, (*packet)->data + (*packet)->size);
      // Trigger the message to be published
      async_publisher_->unlockAndPublish();
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
  std::string sensor_name_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr underlying_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::CompressedImage>>
      async_publisher_;
};

} // namespace ffmpeg_controllers

#endif