#ifndef FFMPEG_CONTROLLERS_PACKET_WRITER_HPP
#define FFMPEG_CONTROLLERS_PACKET_WRITER_HPP

#include <ffmpeg_controllers/writer_base.hpp>

namespace ffmpeg_controllers {

class PacketWriter : public WriterBase {
protected:
  // ===================
  // AS a lifecycle node
  // ===================

  NodeReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
    // Activate the base class first
    if (const NodeReturn base_ret = WriterBase::on_activate(previous_state);
        base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Reset the previous dts
    prev_dts_ = 0;

    return NodeReturn::SUCCESS;
  }
  
  // ===============
  // As a controller
  // ===============

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {input_name_ + "/packet"}};
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {output_name_ + "/packet"}};
  }

  ControllerReturn on_update(const rclcpp::Time & /*time*/,
                             const rclcpp::Duration & /*period*/) override {
    // Try to get the input and output packets owned by the hardware or other chained controller
    const ffmpeg_cpp::Packet *const input_packet =
        get_state_as_pointer<ffmpeg_cpp::Packet>("packet");
    ffmpeg_cpp::Packet *const output_packet = get_command_as_pointer<ffmpeg_cpp::Packet>("packet");
    if (!input_packet || !output_packet) {
      RCLCPP_WARN(get_logger(), "Failed to get input or output packets. Will skip this update.");
      return ControllerReturn::OK;
    }

    // Skip writing if the packet is not new
    if ((*input_packet)->dts <= prev_dts_) {
      return ControllerReturn::OK;
    }

    // Copy the input packet to the output packet
    *output_packet = *input_packet;
    prev_dts_ = (*input_packet)->dts;

    return ControllerReturn::OK;
  }

protected:
  decltype(ffmpeg_cpp::Packet()->dts) prev_dts_;
};

} // namespace ffmpeg_controllers

#endif