#ifndef FFMPEG_HARDWARE_FFMPEG_OUTPUT_HPP
#define FFMPEG_HARDWARE_FFMPEG_OUTPUT_HPP

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_hardware/interface_adapter.hpp>
#include <hardware_interface/actuator_interface.hpp>

namespace ffmpeg_hardware {

class FFmpegOutput : public InterfaceAdapter<hardware_interface::ActuatorInterface> {
protected:
  // ============================
  // Behavior as a lifecycle node
  // ============================

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // TODO: Open and configure the output
      return CallbackReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to configure the output device: %s", error.what());
      return CallbackReturn::ERROR;
    }
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // TODO: Finalize and close the output
      return CallbackReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to close the output device: %s", error.what());
      return CallbackReturn::ERROR;
    }
  }

  // ================================
  // Behavior as a hardware component
  // ================================

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_command_interface_descriptions() override {
    return {make_interface_description("packet", "ffmpeg_cpp::Packet*")};
  }

  hardware_interface::return_type read(const rclcpp::Time & /*time*/,
                                       const rclcpp::Duration & /*period*/) override {
    // Nothing to read
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time & /*time*/,
                                        const rclcpp::Duration & /*period*/) override {
    try {
      if (const auto packet = get_command_as_pointer<ffmpeg_cpp::Packet>("packet");
          packet /* && packet->dts > prev_dts */) {
        output_.write_frame(*packet);
      }
      return hardware_interface::return_type::OK;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to write the packet: %s", error.what());
      return hardware_interface::return_type::ERROR;
    }
  }

protected:
  ffmpeg_cpp::Output output_;
};

} // namespace ffmpeg_hardware

#endif