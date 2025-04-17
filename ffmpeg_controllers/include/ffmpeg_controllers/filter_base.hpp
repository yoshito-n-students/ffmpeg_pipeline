#ifndef FFMPEG_CONTROLLERS_FILTER_BASE_HPP
#define FFMPEG_CONTROLLERS_FILTER_BASE_HPP

#include <stdexcept>
#include <string>

#include <controller_interface/chainable_controller_interface.hpp>
#include <ffmpeg_controllers/interface_adapter.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class FilterBase : public InterfaceAdapter<controller_interface::ChainableControllerInterface> {
private:
  using Base = InterfaceAdapter<controller_interface::ChainableControllerInterface>;

protected:
  // ===================
  // AS a lifecycle node
  // ===================

  CallbackReturn on_init() override {
    // Initialize the base class first
    if (const CallbackReturn base_ret = Base::on_init(); base_ret != CallbackReturn::SUCCESS) {
      return base_ret;
    }

    try {
      // Name of the hardware or chained controller that owns the state to be broadcasted
      input_name_ = Base::get_node()->declare_parameter<std::string>("input_name");
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  // ===============
  // As a controller
  // ===============

  Base::ControllerReturn
  update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                    const rclcpp::Duration & /*period*/) override {
    return Base::ControllerReturn::OK;
  }

  Base::ControllerReturn update_and_write_commands(const rclcpp::Time &time,
                                                   const rclcpp::Duration &period) override {
    return on_update(time, period);
  }

  virtual Base::ControllerReturn on_update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) = 0;

  // =================
  // Utility functions
  // =================

  template <typename T> const T *get_state_as_pointer(const std::string &iface_name) const {
    return Base::get_state_as_pointer<T>(input_name_, iface_name);
  }

protected:
  std::string input_name_;
};

} // namespace ffmpeg_controllers

#endif