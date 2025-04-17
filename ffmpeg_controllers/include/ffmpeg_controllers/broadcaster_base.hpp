#ifndef FFMPEG_CONTROLLERS_BROADCASTER_BASE_HPP
#define FFMPEG_CONTROLLERS_BROADCASTER_BASE_HPP

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_controllers/interface_adapter.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace ffmpeg_controllers {

template <typename MessageT>
class BroadcasterBase : public InterfaceAdapter<controller_interface::ControllerInterface> {
private:
  using Base = InterfaceAdapter<controller_interface::ControllerInterface>;

protected:
  using Message = MessageT;

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
      // topic_ should be set in on_init() of the derived class
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override {
    // Configure the base class first
    if (const CallbackReturn base_ret = Base::on_configure(previous_state);
        base_ret != CallbackReturn::SUCCESS) {
      return base_ret;
    }

    try {
      // Configure the publisher
      underlying_publisher_ =
          get_node()->create_publisher<Message>(topic_, rclcpp::SystemDefaultsQoS());
      async_publisher_ =
          std::make_unique<realtime_tools::RealtimePublisher<Message>>(underlying_publisher_);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Error while creating publishers: %s", error.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  // ===============
  // As a controller
  // ===============

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override {
    // Publish a message if the message is generated and the publisher can be locked
    if (std::optional<Message> msg = on_update(time, period); msg) {
      if (async_publisher_->trylock()) {
        async_publisher_->msg_ = std::move(*msg);
        async_publisher_->unlockAndPublish();
      }
    }

    return controller_interface::return_type::OK;
  }

  virtual std::optional<Message> on_update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) = 0;

  // =================
  // Utility functions
  // =================

  template <typename T> const T *get_state_as_pointer(const std::string &iface_name) const {
    return Base::get_state_as_pointer<T>(input_name_, iface_name);
  }

protected:
  std::string topic_, input_name_;
  typename rclcpp::Publisher<Message>::SharedPtr underlying_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<Message>> async_publisher_;
};

} // namespace ffmpeg_controllers

#endif