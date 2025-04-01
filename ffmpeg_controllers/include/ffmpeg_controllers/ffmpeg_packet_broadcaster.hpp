#ifndef FFMPEG_CONTROLLERS_FFMPEG_PACKET_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_FFMPEG_PACKET_BROADCASTER_HPP

#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
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

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    // No command interfaces needed as this controller is only for broadcasting
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    // Request "foo_camera/packet" state interface
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {sensor_name_ + "/packet"}};
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
      RCLCPP_ERROR(get_node()->get_logger(), "Error while creating publishers: %s", error.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time & /*time*/,
                                           const rclcpp::Duration & /*period*/) override {
    if (async_publisher_->trylock()) {
      // TODO: Transfer data from state_interfaces_ to sync_publisher
      // state_interfaces_[0].get_value();
      async_publisher_->unlockAndPublish();
    }
    return controller_interface::return_type::OK;
  }

protected:
  std::string sensor_name_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr underlying_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::CompressedImage>>
      async_publisher_;
};

} // namespace ffmpeg_controllers

#endif