#ifndef FFMPEG_CONTROLLERS_RECEIVER_BASE_HPP
#define FFMPEG_CONTROLLERS_RECEIVER_BASE_HPP

#include <string>

#include <controller_interface/chainable_controller_interface.hpp>
#include <ffmpeg_controllers/interface_adapter.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace ffmpeg_controllers {

template <typename MessageT>
class ReceiverBase : public InterfaceAdapter<controller_interface::ChainableControllerInterface> {
private:
  using Base = InterfaceAdapter<controller_interface::ChainableControllerInterface>;

protected:
  using Message = MessageT;

  // ===================
  // AS a lifecycle node
  // ===================

  Base::NodeReturn on_configure(const rclcpp_lifecycle::State &previous_state) override {
    // Configure the base class first
    if (const Base::NodeReturn base_ret = Base::on_configure(previous_state);
        base_ret != Base::NodeReturn::SUCCESS) {
      return base_ret;
    }

    try {
      // Configure the subscription
      subscription_ = get_node()->create_subscription<Message>(
          topic_, rclcpp::SystemDefaultsQoS(),
          [this](const typename Message::ConstSharedPtr msg) { msg_buffer_.writeFromNonRT(msg); });
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Error while creating subscription: %s", error.what());
      return Base::NodeReturn::ERROR;
    }

    return Base::NodeReturn::SUCCESS;
  }

  Base::NodeReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
    // Activate the base class first
    if (const Base::NodeReturn base_ret = Base::on_activate(previous_state);
        base_ret != Base::NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Reset the command buffer
    msg_buffer_ = realtime_tools::RealtimeBuffer<typename Message::ConstSharedPtr>();

    return Base::NodeReturn::SUCCESS;
  }

  Base::NodeReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
    // Reset the command buffer
    msg_buffer_ = realtime_tools::RealtimeBuffer<typename Message::ConstSharedPtr>();

    // Deactivate the base class in the end
    return Base::on_deactivate(previous_state);
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
    if (const typename Message::ConstSharedPtr msg = *msg_buffer_.readFromRT(); msg) {
      // If the message is available, process it by the derived class
      return on_update(time, period, *msg);
    } else {
      // If no message is available, just return OK
      return Base::ControllerReturn::OK;
    }
  }

  virtual Base::ControllerReturn on_update(const rclcpp::Time &time, const rclcpp::Duration &period,
                                           const Message &msg) = 0;

protected:
  std::string topic_;
  realtime_tools::RealtimeBuffer<typename Message::ConstSharedPtr> msg_buffer_;
  typename rclcpp::Subscription<Message>::SharedPtr subscription_;
};

} // namespace ffmpeg_controllers

#endif