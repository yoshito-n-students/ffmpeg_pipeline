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
      // Try to get the parameters for the output from the hardware_info, or use default values
      const auto format = get_parameter_as<std::string>("format", "pulse"),
                 filename = get_parameter_as<std::string>("filename", "default");
      const auto codec_params = get_parameter_as<ffmpeg_cpp::CodecParameters>(
          "codec_parameters", ffmpeg_cpp::CodecParameters());
      const auto options = get_parameter_as<std::map<std::string, std::string>>("options", {});

      // Open the input with the parameters
      output_ = ffmpeg_cpp::Output(format, filename, codec_params, options);
      RCLCPP_INFO(get_logger(), "Configured the output (filename: %s, format: %s)",
                  filename.c_str(), format.c_str());

      return CallbackReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to configure the output device: %s", error.what());
      return CallbackReturn::ERROR;
    }
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // Close the output device
      output_ = ffmpeg_cpp::Output();
      RCLCPP_INFO(get_logger(), "Closed the output device");
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