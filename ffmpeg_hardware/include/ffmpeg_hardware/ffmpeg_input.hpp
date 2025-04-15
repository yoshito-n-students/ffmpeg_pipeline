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
    // Try to get the parameters for the input from the hardware_info, or use default values
    const auto url = get_parameter_as<std::string>("url", "/dev/video0"),
               input_format = get_parameter_as<std::string>("input_format", "v4l2"),
               media_type = get_parameter_as<std::string>("media_type", "video");
    const auto options = get_parameter_as<std::map<std::string, std::string>>(
        "options", {{"input_format", "h264"},
                    {"video_size", "1920x1080"},
                    {"framerate", "30"},
                    {"timestamps", "abs"}});

    // Reset the input with the parameters
    try {
      input_ = ffmpeg_cpp::Input(url, input_format, options, media_type);
      RCLCPP_INFO(get_logger(), "Configured the input (URL: %s, format: %s)", url.c_str(),
                  input_format.c_str());
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to configure the input: %s", error.what());
      return CallbackReturn::ERROR;
    }

    // Initialize the codec params and packet
    try {
      codec_params_ = input_.codec_parameters();
      while (true) {
        packet_ = input_.read_frame();
        if (!packet_.empty()) {
          break;
        }
        RCLCPP_INFO(get_logger(), "Waiting for the first packet...");
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize codec name: %s", error.what());
      return CallbackReturn::ERROR;
    }

    // Set the codec name and packet to the state interface
    set_state_from_pointer("codec_parameters", &codec_params_);
    set_state_from_pointer("packet", &packet_);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Close the input device
    try {
      input_ = ffmpeg_cpp::Input();
      RCLCPP_INFO(get_logger(), "Closed the input device");
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to close the input device: %s", error.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  // ================================
  // Behavior as a hardware component
  // ================================

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_state_interface_descriptions() override {
    return {make_interface_description("codec_parameters", "ffmpeg_cpp::CodecParameters*"),
            make_interface_description("packet", "ffmpeg_cpp::Packet*")};
  }

  hardware_interface::return_type read(const rclcpp::Time & /*time*/,
                                       const rclcpp::Duration & /*period*/) override {
    // Update the packet
    try {
      if (ffmpeg_cpp::Packet packet = input_.read_frame(); !packet.empty()) {
        packet_ = std::move(packet);
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to read the packet: %s", error.what());
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

protected:
  ffmpeg_cpp::Input input_;
  ffmpeg_cpp::CodecParameters codec_params_;
  ffmpeg_cpp::Packet packet_;
};

} // namespace ffmpeg_hardware

#endif