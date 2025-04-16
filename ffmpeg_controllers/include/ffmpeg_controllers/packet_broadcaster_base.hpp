#ifndef FFMPEG_CONTROLLERS_PACKET_BROADCASTER_BASE_HPP
#define FFMPEG_CONTROLLERS_PACKET_BROADCASTER_BASE_HPP

#include <algorithm>
#include <memory>
#include <optional>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace ffmpeg_controllers {

template <typename MessageT>
class PacketBroadcasterBase : public controller_interface::ControllerInterface {
public:
  PacketBroadcasterBase(const std::string &default_input_name, const std::string &topic)
      : default_input_name_(default_input_name), topic_(topic) {}

protected:
  using Message = MessageT;

  // ===================
  // AS a lifecycle node
  // ===================

  CallbackReturn on_init() override {
    // Load parameters
    input_name_ = get_node()->declare_parameter("input_name", default_input_name_);
    prev_pts_ = 0;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // Configure the publisher
      underlying_publisher_ =
          get_node()->create_publisher<Message>(topic_, rclcpp::SystemDefaultsQoS());
      async_publisher_ =
          std::make_unique<realtime_tools::RealtimePublisher<Message>>(underlying_publisher_);
      return CallbackReturn::SUCCESS;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Error while creating publishers: %s", error.what());
      return CallbackReturn::ERROR;
    }
  }

  // ===============
  // As a controller
  // ===============

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    // No command interfaces needed as this controller is only for broadcasting
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    // Request "foo_input/codec_parameters" and "foo_input/packet" state interfaces
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {input_name_ + "/codec_parameters", input_name_ + "/packet"}};
  }

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override {
    // Try to get the codec params and packet from the state interfaces
    const auto codec_params = get_state_as_pointer<ffmpeg_cpp::CodecParameters>("codec_parameters");
    const auto packet = get_state_as_pointer<ffmpeg_cpp::Packet>("packet");
    if (!codec_params || !packet) {
      RCLCPP_WARN(get_logger(), "Failed to get codec parameters or packet. Will skip this update.");
      return controller_interface::return_type::OK;
    }

    // Skip publishing if the packet is not new
    if ((*packet)->pts <= prev_pts_) {
      return controller_interface::return_type::OK;
    }

    // Try to build the message based on the codec name and packet
    std::optional<Message> msg = on_update(time, period, *codec_params, *packet);
    if (!msg) {
      return controller_interface::return_type::OK;
    }

    // Try to publish the message
    if (async_publisher_->trylock()) {
      async_publisher_->msg_ = std::move(*msg);
      async_publisher_->unlockAndPublish();
      prev_pts_ = (*packet)->pts;
    }

    return controller_interface::return_type::OK;
  }

  virtual std::optional<Message> on_update(const rclcpp::Time &time, const rclcpp::Duration &period,
                                           const ffmpeg_cpp::CodecParameters &codec_params,
                                           const ffmpeg_cpp::Packet &packet) = 0;

  // =================
  // Utility functions
  // =================

  rclcpp::Logger get_logger() const { return get_node()->get_logger(); }

  // Read the value from the state_interface specified by interface_name,
  // cast it to a pointer type, and return it. Or return nullptr on failure.
  template <typename T> const T *get_state_as_pointer(const std::string &iface_name) const {
    // Find the state interface with the given name
    const auto iface_it =
        std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto &iface) {
          return iface.get_prefix_name() == input_name_ && iface.get_interface_name() == iface_name;
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
  const std::string default_input_name_, topic_;
  std::string input_name_;

  typename rclcpp::Publisher<Message>::SharedPtr underlying_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<Message>> async_publisher_;

  decltype(ffmpeg_cpp::Packet()->pts) prev_pts_;
};

} // namespace ffmpeg_controllers

#endif