#ifndef FFMPEG_CONTROLLERS_PACKET_RECEIVER_HPP
#define FFMPEG_CONTROLLERS_PACKET_RECEIVER_HPP

#include <stdexcept>

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/packet.hpp>

namespace ffmpeg_controllers {

class PacketReceiver : public ReceiverBase<ffmpeg_pipeline_msgs::msg::Packet> {
private:
  using Base = ReceiverBase<ffmpeg_pipeline_msgs::msg::Packet>;

protected:
  // ===================
  // As a lifecycle node
  // ===================

  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Name of interprocess topic to be subscribed to
    input_topic_ = "~/packet";
    // Names of intraprocess read-only variables to be exported
    exported_state_interface_names_ = {"packet"};

    return NodeReturn::SUCCESS;
  }

  NodeReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
    // Activate the base class first
    if (const NodeReturn base_ret = Base::on_activate(previous_state);
        base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Register the packet to state interface owned by this controller
    if (!set_state_from_pointer("packet", &packet_)) {
      return NodeReturn::ERROR;
    }

    return NodeReturn::SUCCESS;
  }

  NodeReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
    // Unregister the packet from state interface owned by this controller
    if (!set_state_from_pointer("packet", nullptr)) {
      return NodeReturn::ERROR;
    }

    // Deactivate the base class in the end
    return Base::on_deactivate(previous_state);
  }

  // ===============
  // As a controller
  // ===============

  ControllerReturn on_process(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
                              const ffmpeg_pipeline_msgs::msg::Packet &msg) override {
    try {
      // Update the packet with the received message
      packet_ = ffmpeg_cpp::Packet(msg);
      return ControllerReturn::OK;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while updating packet: %s", error.what());
      return ControllerReturn::ERROR;
    }
  }

private:
  ffmpeg_cpp::Packet packet_;
};

} // namespace ffmpeg_controllers

#endif