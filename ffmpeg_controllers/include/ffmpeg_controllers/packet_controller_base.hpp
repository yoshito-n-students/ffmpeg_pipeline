#ifndef FFMPEG_CONTROLLERS_PACKET_CONTROLLER_BASE_HPP
#define FFMPEG_CONTROLLERS_PACKET_CONTROLLER_BASE_HPP

#include <algorithm>
#include <optional>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace ffmpeg_controllers {

template <typename MessageT>
class PacketControllerBase : public controller_interface::ControllerInterface {
public:
  PacketControllerBase(const std::string &default_output_name, const std::string &topic)
      : default_output_name_(default_output_name), topic_(topic) {}

protected:
  using Message = MessageT;

  // ===================
  // AS a lifecycle node
  // ===================

  CallbackReturn on_init() override {
    // Load parameters
    output_name_ = get_node()->declare_parameter("output_name", default_output_name_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // Configure the subscription
      subscription_ = get_node()->create_subscription<Message>(
          topic_, rclcpp::SystemDefaultsQoS(),
          [this](const typename Message::ConstSharedPtr msg) { msg_buffer_.writeFromNonRT(msg); });
      return CallbackReturn::SUCCESS;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Error while creating subscription: %s", error.what());
      return CallbackReturn::ERROR;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Reset the command buffer
    msg_buffer_ = realtime_tools::RealtimeBuffer<typename Message::ConstSharedPtr>();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Reset the command buffer
    msg_buffer_ = realtime_tools::RealtimeBuffer<typename Message::ConstSharedPtr>();
    return CallbackReturn::SUCCESS;
  }

  // ===============
  // As a controller
  // ===============

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    // Request "foo_output/packet" command interfaces
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {output_name_ + "/packet"}};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    // No state interfaces needed as this controller is only for writing command
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override {
    // Try to get the reference to the packet variable owned by the output
    auto dst_packet = get_command_as_pointer<ffmpeg_cpp::Packet>("packet");
    if (!dst_packet) {
      RCLCPP_WARN(get_logger(),
                  "Failed to get reference to the packet command. Will skip this update.");
      return controller_interface::return_type::OK;
    }

    // Try to get the subscribed message
    const typename Message::ConstSharedPtr msg = *msg_buffer_.readFromRT();
    if (!msg) {
      RCLCPP_WARN(get_logger(), "Empty message obtained. Will skip this update.");
      return controller_interface::return_type::OK;
    }

    // Convert the message to a packet
    auto packet = on_update(time, period, *msg);
    if (!packet) {
      RCLCPP_WARN(get_logger(), "No packet generated. Will skip this update.");
      return controller_interface::return_type::OK;
    }

    // Write the packet to the output
    *dst_packet = std::move(*packet);

    return controller_interface::return_type::OK;
  }

  virtual std::optional<ffmpeg_cpp::Packet>
  on_update(const rclcpp::Time &time, const rclcpp::Duration &period, const Message &msg) = 0;

  // =================
  // Utility functions
  // =================

  rclcpp::Logger get_logger() const { return get_node()->get_logger(); }

  // Read the value from the command_interface specified by interface_name,
  // cast it to a pointer type, and return it. Or return nullptr on failure.
  template <typename T> T *get_command_as_pointer(const std::string &iface_name) const {
    // Find the command interface with the given keys
    const auto iface_it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                                       [&](const auto &iface) {
                                         return iface.get_prefix_name() == output_name_ &&
                                                iface.get_interface_name() == iface_name;
                                       });
    if (iface_it == command_interfaces_.end()) {
      return nullptr;
    }

    // Try to read the raw command value from the interface.
    // Due to the limitations of hardware_interface::CommandInterface,
    // the value is stored as a double.
    const auto double_value = iface_it->template get_optional<double>();
    if (double_value == std::nullopt) {
      return nullptr;
    }

    // Reconstruct the pointer from the raw command value.
    // The double type has 53 bits of precision, while the Linux user memory space has 47 bits,
    // so the former can be safely converted to the latter.
    return reinterpret_cast<T *>(static_cast<std::uintptr_t>(*double_value));
  }

protected:
  const std::string default_output_name_, topic_;
  std::string output_name_;

  realtime_tools::RealtimeBuffer<typename Message::ConstSharedPtr> msg_buffer_;
  typename rclcpp::Subscription<Message>::SharedPtr subscription_;
};

} // namespace ffmpeg_controllers

#endif