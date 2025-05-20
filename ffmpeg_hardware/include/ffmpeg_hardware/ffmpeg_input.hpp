#ifndef FFMPEG_HARDWARE_FFMPEG_INPUT_HPP
#define FFMPEG_HARDWARE_FFMPEG_INPUT_HPP

#include <chrono>
#include <string>
#include <thread>
#include <utility> // for std::move()
#include <vector>

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_hardware/interface_adapter.hpp>
#include <hardware_interface/handle.hpp> // for hi::Interface{Description,Info}
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

namespace ffmpeg_hardware {

class FFmpegInput : public InterfaceAdapter<hardware_interface::SensorInterface> {
protected:
  // ============================
  // Behavior as a lifecycle node
  // ============================

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // Try to get the parameters for the input from the hardware_info, or use default values
      const auto ffmpeg_log_level = get_parameter_as<std::string>("ffmpeg_log_level", ""),
                 url = get_parameter_as<std::string>("url", "/dev/video0"),
                 format = get_parameter_as<std::string>("format", "v4l2"),
                 media_type = get_parameter_as<std::string>("media_type", "video");
      const auto options = get_parameter_as<ffmpeg_cpp::Dictionary>(
          "options", ffmpeg_cpp::Dictionary::create("{timestamps: abs}"));

      if (!ffmpeg_log_level.empty()) {
        ffmpeg_cpp::set_log_level(ffmpeg_log_level);
      }

      // Open the input with the parameters
      input_ = ffmpeg_cpp::Input::create(url, format, options, media_type);
      RCLCPP_INFO(get_logger(), "Configured the input ([%s] %s)", input_->iformat->name,
                  input_->url);

      // Initialize the codec params and packet by reading them from the input
      codec_params_ = input_.codec_parameters();
      while (true) {
        packet_ = input_.read_frame();
        if (!packet_.empty()) {
          break;
        }
        RCLCPP_INFO(get_logger(), "Waiting for the first packet...");
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }

      // Register the initialized state variables to the interface
      set_state_from_pointer("codec_parameters", &codec_params_);
      set_state_from_pointer("packet", &packet_);

      return CallbackReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to configure the input: %s", error.what());
      return CallbackReturn::ERROR;
    }
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // Unregister the state variables to the interface
      set_state_from_pointer("codec_parameters", nullptr);
      set_state_from_pointer("packet", nullptr);

      // Close the input device
      input_ = ffmpeg_cpp::Input::null();
      RCLCPP_INFO(get_logger(), "Closed the input device");
      return CallbackReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to close the input device: %s", error.what());
      return CallbackReturn::ERROR;
    }
  }

  // ================================
  // Behavior as a hardware component
  // ================================

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override {
    return {make_interface_description("codec_parameters"), make_interface_description("packet")};
  }

  hardware_interface::return_type read(const rclcpp::Time & /*time*/,
                                       const rclcpp::Duration & /*period*/) override {
    try {
      // Update the packet
      if (ffmpeg_cpp::Packet packet = input_.read_frame(); !packet.empty()) {
        packet_ = std::move(packet);
      }
      return hardware_interface::return_type::OK;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to read the packet: %s", error.what());
      return hardware_interface::return_type::ERROR;
    }
  }

protected:
  ffmpeg_cpp::Input input_ = ffmpeg_cpp::Input::null();
  ffmpeg_cpp::CodecParameters codec_params_ = ffmpeg_cpp::CodecParameters::null();
  ffmpeg_cpp::Packet packet_ = ffmpeg_cpp::Packet::null();
};

} // namespace ffmpeg_hardware

#endif