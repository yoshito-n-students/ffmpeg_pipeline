#ifndef FFMPEG_CONTROLLERS_DETAIL_UTILITY_HPP
#define FFMPEG_CONTROLLERS_DETAIL_UTILITY_HPP

#include <algorithm>
#include <initializer_list>
#include <set>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_interface_base.hpp> // for return_type, InterfaceConfiguration
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp> // for CallbackReturn

namespace ffmpeg_controllers {

// Type aliases for the return values of virtual interface functions
// - for node-related functions
using NodeReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
// - for controller-related functions
using ControllerReturn = controller_interface::return_type;

// Return the most severe result from a list of NodeReturn.
// If the list is empty, default to SUCCESS.
static inline NodeReturn merge(std::initializer_list<NodeReturn> results) {
  constexpr auto to_severity = [](const NodeReturn result) {
    switch (result) {
    case NodeReturn::SUCCESS:
      return 0;
    case NodeReturn::FAILURE:
      return 1;
    case NodeReturn::ERROR:
      return 2;
    default:
      throw std::runtime_error("Unknown NodeReturn value: " +
                               std::to_string(static_cast<int>(result)));
    }
  };
  constexpr auto more_severe = [to_severity](const NodeReturn a, const NodeReturn b) {
    return to_severity(a) >= to_severity(b) ? a : b;
  };
  // Reduce the list of results to the most severe one with an initial value of SUCCESS
  return std::reduce(results.begin(), results.end(), NodeReturn::SUCCESS, more_severe);
}

// Return the most severe result from a list of ControllerReturn.
// If the list is empty, default to OK.
static inline ControllerReturn merge(std::initializer_list<ControllerReturn> results) {
  constexpr auto to_severity = [](const ControllerReturn result) {
    switch (result) {
    case ControllerReturn::OK:
      return 0;
    case ControllerReturn::ERROR:
      return 1;
    default:
      throw std::runtime_error("Unknown ControllerReturn value: " +
                               std::to_string(static_cast<int>(result)));
    }
  };
  constexpr auto more_severe = [to_severity](const ControllerReturn a, const ControllerReturn b) {
    return to_severity(a) >= to_severity(b) ? a : b;
  };
  // Reduce the list of results to the most severe one with an initial value of OK
  return std::reduce(results.begin(), results.end(), ControllerReturn::OK, more_severe);
}

// Return interface configuration that covers all given configurations.
// If the list is empty, default to NONE configuration.
static inline controller_interface::InterfaceConfiguration
merge(std::initializer_list<controller_interface::InterfaceConfiguration> configs) {
  // Type aliases for convenience
  using ConfigType = controller_interface::interface_configuration_type;
  using Config = controller_interface::InterfaceConfiguration;
  // Helper functions
  constexpr auto to_coverage = [](const ConfigType type) {
    switch (type) {
    case ConfigType::NONE:
      return 0;
    case ConfigType::INDIVIDUAL:
      return 1;
    case ConfigType::ALL:
      return 2;
    default:
      throw std::runtime_error("Unknown interface_configuration_type value: " +
                               std::to_string(static_cast<int>(type)));
    }
  };
  constexpr auto binary_merge = [to_coverage](const Config &a, const Config &b) {
    Config result;
    result.type = (to_coverage(a.type) >= to_coverage(b.type) ? a.type : b.type);
    // fill the names field which is only relevant for INDIVIDUAL type
    if (result.type == ConfigType::INDIVIDUAL) {
      if (a.type == ConfigType::INDIVIDUAL && b.type == ConfigType::INDIVIDUAL) {
        std::set<std::string> names;
        names.insert(a.names.begin(), a.names.end());
        names.insert(b.names.begin(), b.names.end());
        result.names.assign(names.begin(), names.end());
      } else if (a.type == ConfigType::INDIVIDUAL) {
        result.names = a.names;
      } else { // b.type == ConfigType::INDIVIDUAL
        result.names = b.names;
      }
    }
    return result;
  };
  // Merge the list of configurations with an initial value of NONE configuration
  return std::reduce(configs.begin(), configs.end(), Config{ConfigType::NONE, {}}, binary_merge);
}

// Get QoS settings of ROS 2 message transport from a profile name
static inline rclcpp::QoS to_qos(const std::string &profile) {
  if (profile == "best_available") {
    return rclcpp::BestAvailableQoS();
  } else if (profile == "clock") {
    return rclcpp::ClockQoS();
  } else if (profile == "parameters") {
    return rclcpp::ParametersQoS();
  } else if (profile == "parameter_events") {
    return rclcpp::ParameterEventsQoS();
  } else if (profile == "rosout") {
    return rclcpp::RosoutQoS();
  } else if (profile == "sensor_data") {
    return rclcpp::SensorDataQoS();
  } else if (profile == "services") {
    return rclcpp::ServicesQoS();
  } else if (profile == "system_defaults") {
    return rclcpp::SystemDefaultsQoS();
  } else {
    throw std::runtime_error("Unknown QoS profile: " + profile);
  }
}

} // namespace ffmpeg_controllers

#endif