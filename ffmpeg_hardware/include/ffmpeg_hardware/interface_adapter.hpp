#ifndef FFMPEG_HARDWARE_INTERFACE_ADAPTER_HPP
#define FFMPEG_HARDWARE_INTERFACE_ADAPTER_HPP

#include <string>
#include <utility> // for std::move()

#include <hardware_interface/handle.hpp> // for hi::Interface{Description,Info}

#include <yaml-cpp/yaml.h>

namespace ffmpeg_hardware {

template <class Interface> class InterfaceAdapter : public Interface {
protected:
  template <typename T> T get_parameter_as(const std::string &key, T &&default_value) const {
    // Try to find the parameter with the given key
    const auto found_it = Interface::info_.hardware_parameters.find(key);
    if (found_it != Interface::info_.hardware_parameters.end()) {
      // If the parameter is found, try to convert it to the desired type
      try {
        return YAML::Load(found_it->second).template as<T>();
      } catch (const YAML::Exception &error) {
        // If the conversion fails, log an error and return the default value
        RCLCPP_ERROR(Interface::get_logger(),
                     "Failed to convert parameter '%s' (value: '%s') to the desired type: %s",
                     found_it->first.c_str(), found_it->second.c_str(), error.what());
        return std::move(default_value);
      }
    } else {
      // If the parameter is not found, return the default value
      return std::move(default_value);
    }
  }

  // Due to the constraints of hardware_interfae::{Command,State}Interface,
  // the command/state can only be set as a double type value.
  // Therefore, in case of getting the pointer-type command,
  // the double type on the interface is converted to pointer type at the value level.
  // In case of setting the pointer-type state, 
  // the pointer type is converted to double type at the value level for the interface.
  // The double type has 53 bits of precision,
  // which is sufficient to represent the 47 bits of the Linux user's address space.

  template <typename T> const T *get_command_as_pointer(const std::string &iface_name) const {
    return reinterpret_cast<const T *>(static_cast<std::uintptr_t>(
        Interface::get_command(make_interface_description(iface_name, "").get_name())));
  }

  template <typename T>
  void set_state_from_pointer(const std::string &iface_name, const T *const value) {
    Interface::set_state(make_interface_description(iface_name, "").get_name(),
                         static_cast<double>(reinterpret_cast<std::uintptr_t>(value)));
  }

  hardware_interface::InterfaceDescription
  make_interface_description(const std::string &iface_name, const std::string &data_type) const {
    hardware_interface::InterfaceInfo info;
    info.name = iface_name;
    info.data_type = data_type;
    return hardware_interface::InterfaceDescription{Interface::info_.name, info};
  }
};

} // namespace ffmpeg_hardware

#endif