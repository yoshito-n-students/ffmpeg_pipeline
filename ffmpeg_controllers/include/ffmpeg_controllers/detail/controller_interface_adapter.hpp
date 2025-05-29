#ifndef FFMPEG_CONTROLLERS_DETAIL_CONTROLLER_INTERFACE_ADAPTER_HPP
#define FFMPEG_CONTROLLERS_DETAIL_CONTROLLER_INTERFACE_ADAPTER_HPP

#include <cstdint>
#include <string>
#include <type_traits>

#include <controller_interface/chainable_controller_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_controllers/detail/utility.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

namespace ffmpeg_controllers {

// ==============================================
// Controller interface adapter for mixin classes
// ==============================================

template <class ControllerIface> class ControllerInterfaceAdapter;

// Common base class for all variations of ControllerInterfaceAdapter<>
template <class ControllerIface>
class ControllerInterfaceAdapterBase : public virtual ControllerIface {
protected:
  // Provide default implementations for virtual functions
  // defined but not implemented in controller_interface::[Chainable]ControllerInterface

  NodeReturn on_init() override { return NodeReturn::SUCCESS; }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  // Virtual function to be called during update regardless of the value of InheritanceOption
  virtual ControllerReturn on_update(const rclcpp::Time &time, const rclcpp::Duration &period) = 0;

  // Provide some utility functions for derived classes

  rclcpp::Logger get_logger() const { return ControllerIface::get_node()->get_logger(); }

  template <typename T> T get_user_parameter(const std::string &name) {
    if (!ControllerIface::get_node()->has_parameter(name)) {
      ControllerIface::get_node()->template declare_parameter<T>(name);
    }
    return ControllerIface::get_node()->get_parameter(name).template get_value<T>();
  }

  // Get the user parameter value with a default value.
  // Unlike ControllerIface::auto_declare(), this function does not set the parameter value
  // via declare_parameter(name, default_val) or get_parameter_or(name, default_val).
  // This ensures the parameter value is set by the user, not by other modules in the controller.
  template <typename T> T get_user_parameter(const std::string &name, const T &default_value) {
    try {
      return get_user_parameter<T>(name);
    } catch (const std::runtime_error &error) {
      return default_value;
    }
  }

  // Get the pointer value stored in the state interface loaned from other classes
  template <typename T>
  const T *get_state_as_pointer(const std::string &prefix_name,
                                const std::string &iface_name) const {
    // Find the state interface specified by iface_name
    // exported from the hardware or chained controller specified by prefix_name
    for (const auto &iface : ControllerIface::state_interfaces_) {
      if (iface.get_prefix_name() == prefix_name && iface.get_interface_name() == iface_name) {
        // Convert the double value held by the state interface to a pointer type and return it
        if (const auto double_value = iface.template get_optional<double>(); double_value) {
          return reinterpret_cast<const T *>(static_cast<std::uintptr_t>(*double_value));
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
    for (const auto &iface : ControllerIface::command_interfaces_) {
      if (iface.get_prefix_name() == prefix_name && iface.get_interface_name() == iface_name) {
        // Convert the double value held by the state interface to a pointer type and return it
        if (const auto double_value = iface.template get_optional<double>(); double_value) {
          return reinterpret_cast<T *>(static_cast<std::uintptr_t>(*double_value));
        }
      }
    }
    return nullptr;
  }

  // Set the pointer value to the state interface exported from the derived class
  template <typename T>
  std::enable_if_t<std::is_pointer_v<T> || std::is_same_v<T, std::nullptr_t>, bool>
  set_state_from_pointer(const std::string &iface_name, const T ptr_value) {
    // Find the state interface specified by iface_name owned by this controller
    for (const auto &iface : ControllerIface::ordered_exported_state_interfaces_) {
      if (iface && iface->get_interface_name() == iface_name) {
        // Convert the pointer to a double value and set it to the state interface
        return iface->set_value(static_cast<double>(reinterpret_cast<std::uintptr_t>(ptr_value)));
      }
    }
    return false;
  }
};

template <>
class ControllerInterfaceAdapter<controller_interface::ControllerInterface>
    : public virtual ControllerInterfaceAdapterBase<controller_interface::ControllerInterface> {
protected:
  ControllerReturn update(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    return on_update(time, period);
  }
};

template <>
class ControllerInterfaceAdapter<controller_interface::ChainableControllerInterface>
    : public virtual ControllerInterfaceAdapterBase<
          controller_interface::ChainableControllerInterface> {
protected:
  ControllerReturn update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                                     const rclcpp::Duration & /*period*/) override {
    return ControllerReturn::OK;
  }

  ControllerReturn update_and_write_commands(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) override {
    return on_update(time, period);
  }
};

} // namespace ffmpeg_controllers

#endif