#ifndef FFMPEG_CONTROLLERS_DETAIL_UTILITY_HPP
#define FFMPEG_CONTROLLERS_DETAIL_UTILITY_HPP

#include <algorithm>
#include <initializer_list>
#include <set>
#include <stdexcept>

// for CallbackReturn, return_type, InterfaceConfiguration
#include <controller_interface/controller_interface_base.hpp>
#include <rclcpp/qos.hpp>

namespace ffmpeg_controllers {

// Type aliases for the return values of virtual interface functions
// - for node-related functions
using NodeReturn = controller_interface::ControllerInterfaceBase::CallbackReturn;
// - for controller-related functions
using ControllerReturn = controller_interface::return_type;

// Return SUCCESS if all given results are SUCCESS, otherwise return ERROR
static inline NodeReturn merge(std::initializer_list<NodeReturn> results) {
  return std::all_of(results.begin(), results.end(),
                     [](const auto result) { return result == NodeReturn::SUCCESS; })
             ? NodeReturn::SUCCESS
             : NodeReturn::ERROR;
}

// Return OK if all given results are OK, otherwise return ERROR
static inline ControllerReturn merge(std::initializer_list<ControllerReturn> results) {
  return std::all_of(results.begin(), results.end(),
                     [](const auto result) { return result == ControllerReturn::OK; })
             ? ControllerReturn::OK
             : ControllerReturn::ERROR;
}

// Return interface configuration that covers all given configurations
static inline controller_interface::InterfaceConfiguration
merge(std::initializer_list<controller_interface::InterfaceConfiguration> configs) {
  using ConfigType = controller_interface::interface_configuration_type;
  if (std::any_of(configs.begin(), configs.end(),
                  [](const auto &config) { return config.type == ConfigType::ALL; })) {
    return {ConfigType::ALL, {}};
  } else if (std::all_of(configs.begin(), configs.end(),
                         [](const auto &config) { return config.type == ConfigType::NONE; })) {
    return {ConfigType::NONE, {}};
  } else {
    std::set<std::string> names;
    for (const auto &config : configs) {
      if (config.type == ConfigType::INDIVIDUAL) {
        names.insert(config.names.begin(), config.names.end());
      }
    }
    return {ConfigType::INDIVIDUAL, std::vector<std::string>(names.begin(), names.end())};
  }
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