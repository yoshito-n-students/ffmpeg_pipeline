#ifndef FFMPEG_HARDWARE_DUMP_INFO_OUTPUT_HPP
#define FFMPEG_HARDWARE_DUMP_INFO_OUTPUT_HPP

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_hardware/interface_adapter.hpp>
#include <hardware_interface/actuator_interface.hpp>

namespace ffmpeg_hardware {

class DumpInfoOutput : public InterfaceAdapter<hardware_interface::ActuatorInterface> {
protected:
  // ============================
  // Behavior as a lifecycle node
  // ============================

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      const auto ffmpeg_log_level = get_parameter_as<std::string>("ffmpeg_log_level", "");

      if (!ffmpeg_log_level.empty()) {
        ffmpeg_cpp::set_log_level(ffmpeg_log_level);
      }

      // Print available parameters for debugging purposes
      RCLCPP_INFO(get_logger(), "Hardware parameters for '%s':", info_.name.c_str());
      for (const auto &[key, value] : info_.hardware_parameters) {
        RCLCPP_INFO(get_logger(), "  '%s': '%s'", key.c_str(), value.c_str());
      }

      // Initialize the command variables and register them to the interface
      packet_ = ffmpeg_cpp::Packet::create();
      frame_ = ffmpeg_cpp::Frame::create();
      packet_->dts = frame_->pkt_dts = prev_dts_ = 0;
      set_command_from_pointer("packet", &packet_);
      set_command_from_pointer("frame", &frame_);
      return CallbackReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to configure the mock output: %s", error.what());
      return CallbackReturn::ERROR;
    }
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // Unregister the command variables to the interface and free them
      set_command_from_pointer("packet", nullptr);
      set_command_from_pointer("frame", nullptr);
      packet_ = ffmpeg_cpp::Packet::null();
      frame_ = ffmpeg_cpp::Frame::null();
      return CallbackReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate the mock output: %s", error.what());
      return CallbackReturn::ERROR;
    }
  }

  // ================================
  // Behavior as a hardware component
  // ================================

  std::vector<hardware_interface::InterfaceDescription>
  export_unlisted_command_interface_descriptions() override {
    return {make_interface_description("packet"), make_interface_description("frame")};
  }

  hardware_interface::return_type read(const rclcpp::Time & /*time*/,
                                       const rclcpp::Duration & /*period*/) override {
    // Nothing to read from the output device
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time & /*time*/,
                                        const rclcpp::Duration & /*period*/) override {
    try {
      // Write packet_ and frame_ to the output device
      // if their timestamp is newer than the previous one.
      // If both timestamps are newer, write the one with the older timestamp first.
      if (packet_->dts > prev_dts_ && frame_->pkt_dts > prev_dts_) {
        if (packet_->dts > frame_->pkt_dts) {
          // prev < frame < packet
          print_frame();
          print_packet();
          prev_dts_ = packet_->dts;
        } else {
          // prev < packet <= frame
          print_packet();
          print_frame();
          prev_dts_ = frame_->pkt_dts;
        }
      } else if (packet_->dts > prev_dts_) {
        // frame <= prev < packet
        print_packet();
        prev_dts_ = packet_->dts;
      } else if (frame_->pkt_dts > prev_dts_) {
        // packet <= prev < frame
        print_frame();
        prev_dts_ = frame_->pkt_dts;
      }
      return hardware_interface::return_type::OK;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to print the frame: %s", error.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  // Utility functions

  void print_packet() const {
    RCLCPP_INFO(get_logger(), "Got packet (%ld | %d bytes)", packet_->dts, packet_->size);
  }

  void print_frame() const {
    if (frame_->width > 0 && frame_->height > 0) {
      RCLCPP_INFO(get_logger(), "Got frame (%ld | %s | %dx%d)", frame_->pkt_dts,
                  frame_.format_name().c_str(), frame_->width, frame_->height);
    } else if (frame_->sample_rate > 0) {
      RCLCPP_INFO(get_logger(), "Got frame (%ld | %s | %s | %d Hz | %d samples)", frame_->pkt_dts,
                  frame_.ch_layout_str().c_str(), frame_.format_name().c_str(), frame_->sample_rate,
                  frame_->nb_samples);
    } else {
      RCLCPP_INFO(get_logger(), "Got frame (%ld | %s)", frame_->pkt_dts);
    }
  }

protected:
  ffmpeg_cpp::Packet packet_ = ffmpeg_cpp::Packet::null();
  ffmpeg_cpp::Frame frame_ = ffmpeg_cpp::Frame::null();
  std::int64_t prev_dts_ = 0;
};

} // namespace ffmpeg_hardware

#endif