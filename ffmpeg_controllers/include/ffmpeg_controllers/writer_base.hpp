#ifndef FFMPEG_CONTROLLERS_WRITER_BASE_HPP
#define FFMPEG_CONTROLLERS_WRITER_BASE_HPP

#include <stdexcept>
#include <string>

#include <ffmpeg_controllers/filter_base.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class WriterBase : public FilterBase {
protected:
  // ===================
  // AS a lifecycle node
  // ===================

  CallbackReturn on_init() override {
    // Initialize the base class first
    if (const CallbackReturn base_ret = FilterBase::on_init();
        base_ret != CallbackReturn::SUCCESS) {
      return base_ret;
    }

    try {
      // Name of the hardware or chained controller that owns the state to be broadcasted
      output_name_ = get_node()->declare_parameter<std::string>("output_name");
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while getting parameter value: %s", error.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  // =================
  // Utility functions
  // =================

  template <typename T> T *get_command_as_pointer(const std::string &iface_name) const {
    return FilterBase::get_command_as_pointer<T>(output_name_, iface_name);
  }

protected:
  std::string output_name_;
};

} // namespace ffmpeg_controllers

#endif