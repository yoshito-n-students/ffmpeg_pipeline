#ifndef FFMPEG_CONTROLLERS_INTERFACE_ADAPTER_HPP
#define FFMPEG_CONTROLLERS_INTERFACE_ADAPTER_HPP

#include <string>

#include <controller_interface/controller_interface_base.hpp> // for ci::return_type
#include <rclcpp/logger.hpp>

namespace ffmpeg_controllers {

template <class Interface> class InterfaceAdapter : public Interface {
protected:
  // Consistent alias names for the return types of virtual interface functions
  using NodeReturn = typename Interface::CallbackReturn;
  using ControllerReturn = controller_interface::return_type;

  // ===================
  // AS a lifecycle node
  // ===================

  // Implementation of the on_init(). Other node-related virtual functions
  // are already implemented in controller_interface::[Chainable]ControllerInterface.
  NodeReturn on_init() override { return NodeReturn::SUCCESS; }

  // ===============
  // AS a controller
  // ===============

  // Implementation for derived classes that do not use command interfaces
  // owned by hardware or other chainable controllers
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  // Implementation for derived classes that do not use state interfaces
  // owned by hardware or other chainable controllers
  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  // =================
  // Utility functions
  // =================

  rclcpp::Logger get_logger() const { return Interface::get_node()->get_logger(); }

  // This package uses hardware_interface::{Command,State}Interface
  // to exchange various types of data between hardware and controller.
  // On the other hand, due to the constraints of hardware_interface,
  // the only available type is double.
  // Therefore, as a workaround, pointer types are converted to double type for exchange.
  // The double type can represent up to 53 bits,
  // which is sufficient to represent the 47 bits of the Linux user's address space.

  // Get the pointer value stored in the state interface loaned from other classes
  template <typename T>
  const T *get_state_as_pointer(const std::string &prefix_name,
                                const std::string &iface_name) const {
    // Find the state interface specified by iface_name
    // exported from the hardware or chained controller specified by prefix_name
    for (const auto &iface : Interface::state_interfaces_) {
      if (iface.get_prefix_name() == prefix_name && iface.get_interface_name() == iface_name) {
        // Convert the double value held by the state interface to a pointer type and return it
        if (const auto double_value = iface.template get_optional<double>(); double_value) {
          return reinterpret_cast<const T *>(static_cast<std::uintptr_t>(*double_value));
        } else {
          return nullptr;
        }
      }
    }
    return nullptr;
  }

  // Get the pointer value stored in the command interface loaned from other classes
  template <typename T>
  T *get_command_as_pointer(const std::string &prefix_name, const std::string &iface_name) const {
    // Find the state interface specified by iface_name
    // exported from the hardware or chained controller specified by prefix_name
    for (const auto &iface : Interface::command_interfaces_) {
      if (iface.get_prefix_name() == prefix_name && iface.get_interface_name() == iface_name) {
        // Convert the double value held by the state interface to a pointer type and return it
        if (const auto double_value = iface.template get_optional<double>(); double_value) {
          return reinterpret_cast<T *>(static_cast<std::uintptr_t>(*double_value));
        } else {
          return nullptr;
        }
      }
    }
    return nullptr;
  }

  // Set the pointer value to the state interface exported from the derived class
  template <typename T>
  bool set_state_from_pointer(const std::string &iface_name, const T *const ptr_value) {
    // Find the state interface specified by iface_name owned by this controller
    for (const auto &iface : Interface::ordered_exported_state_interfaces_) {
      if (iface && iface->get_interface_name() == iface_name) {
        // Convert the pointer to a double value and set it to the state interface
        return iface->set_value(static_cast<double>(reinterpret_cast<std::uintptr_t>(ptr_value)));
      }
    }
    return false;
  }
};

} // namespace ffmpeg_controllers

#endif