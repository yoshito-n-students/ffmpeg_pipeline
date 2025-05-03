#ifndef FFMPEG_CONTROLLERS_DETAIL_INTERFACE_ADAPTER_HPP
#define FFMPEG_CONTROLLERS_DETAIL_INTERFACE_ADAPTER_HPP

#include <cstdint>
#include <string>
#include <type_traits>

#include <controller_interface/chainable_controller_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

namespace ffmpeg_controllers {

// ===================================
// Interface adapter for mixin classes
// ===================================

template <class Interface> class InterfaceAdapter;

// Common base class for all variations of InterfaceAdapter<>
template <class Interface> class InterfaceAdapterBase : public virtual Interface {
protected:
  // Provide consistent aliases for the return values of virtual interface functions
  using NodeReturn = typename Interface::CallbackReturn;      // for node-related functions
  using ControllerReturn = controller_interface::return_type; // for controller-related functions

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

  rclcpp::Logger get_logger() const { return Interface::get_node()->get_logger(); }

  template <typename T> T get_user_parameter(const std::string &name) {
    if (!Interface::get_node()->has_parameter(name)) {
      Interface::get_node()->template declare_parameter<T>(name);
    }
    return Interface::get_node()->get_parameter(name).template get_value<T>();
  }

  // Get the user parameter value with a default value.
  // Unlike Interface::auto_declare(), this function does not set the parameter value
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
    for (const auto &iface : Interface::state_interfaces_) {
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
    for (const auto &iface : Interface::command_interfaces_) {
      if (iface.get_prefix_name() == prefix_name && iface.get_interface_name() == iface_name) {
        // Convert the double value held by the state interface to a pointer type and return it
        if (const auto double_value = iface.template get_optional<double>(); double_value) {
          return reinterpret_cast<T *>(static_cast<std::uintptr_t>(*double_value));
        }
      }
    }
    return nullptr;
  }

  // Return SUCCESS if all results in the given range are SUCCESS, otherwise return ERROR
  template <class Range>
  static std::enable_if_t<std::is_same_v<typename Range::value_type, NodeReturn>, NodeReturn>
  merge(const Range &range) {
    return std::all_of(range.begin(), range.end(),
                       [](const auto result) { return result == NodeReturn::SUCCESS; })
               ? NodeReturn::SUCCESS
               : NodeReturn::ERROR;
  }

  // Return OK if all results in the given range are OK, otherwise return ERROR
  template <class Range>
  static std::enable_if_t<std::is_same_v<typename Range::value_type, ControllerReturn>,
                          ControllerReturn>
  merge(const Range &range) {
    return std::all_of(range.begin(), range.end(),
                       [](const auto result) { return result == ControllerReturn::OK; })
               ? ControllerReturn::OK
               : ControllerReturn::ERROR;
  }

  // Return interface configuration that covers all configurations in the given range
  template <class Range>
  static std::enable_if_t<
      std::is_same_v<typename Range::value_type, controller_interface::InterfaceConfiguration>,
      controller_interface::InterfaceConfiguration>
  merge(const Range &range) {
    using ConfigType = controller_interface::interface_configuration_type;
    if (std::any_of(range.begin(), range.end(),
                    [](const auto &config) { return config.type == ConfigType::ALL; })) {
      return {ConfigType::ALL, {}};
    } else if (std::all_of(range.begin(), range.end(),
                           [](const auto &config) { return config.type == ConfigType::NONE; })) {
      return {ConfigType::NONE, {}};
    } else {
      std::set<std::string> names;
      for (const auto &config : range) {
        if (config.type == ConfigType::INDIVIDUAL) {
          names.insert(config.names.begin(), config.names.end());
        }
      }
      return {ConfigType::INDIVIDUAL, std::vector<std::string>(names.begin(), names.end())};
    }
  }
};

template <>
class InterfaceAdapter<controller_interface::ControllerInterface>
    : public virtual InterfaceAdapterBase<controller_interface::ControllerInterface> {
private:
  using Base = InterfaceAdapterBase<controller_interface::ControllerInterface>;

protected:
  Base::ControllerReturn update(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    return on_update(time, period);
  }
};

template <>
class InterfaceAdapter<controller_interface::ChainableControllerInterface>
    : public virtual InterfaceAdapterBase<controller_interface::ChainableControllerInterface> {
private:
  using Base = InterfaceAdapterBase<controller_interface::ChainableControllerInterface>;

protected:
  Base::ControllerReturn
  update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                    const rclcpp::Duration & /*period*/) override {
    return Base::ControllerReturn::OK;
  }

  Base::ControllerReturn update_and_write_commands(const rclcpp::Time &time,
                                                   const rclcpp::Duration &period) override {
    return on_update(time, period);
  }

  // Set the pointer value to the state interface exported from the derived class
  template <typename T>
  std::enable_if_t<std::is_pointer_v<T> || std::is_same_v<T, std::nullptr_t>, bool>
  set_state_from_pointer(const std::string &iface_name, const T ptr_value) {
    // Find the state interface specified by iface_name owned by this controller
    for (const auto &iface : Base::ordered_exported_state_interfaces_) {
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