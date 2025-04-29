#ifndef FFMPEG_HARDWARE_MOCK_OUTPUT_HPP
#define FFMPEG_HARDWARE_MOCK_OUTPUT_HPP

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_hardware/interface_adapter.hpp>
#include <hardware_interface/actuator_interface.hpp>

namespace ffmpeg_hardware {

class MockOutput : public InterfaceAdapter<hardware_interface::ActuatorInterface> {
protected:
  // ============================
  // Behavior as a lifecycle node
  // ============================

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // Initialize the command variables and register them to the interface
      packet_ = ffmpeg_cpp::Packet();
      frame_ = ffmpeg_cpp::Frame();
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
      packet_ = ffmpeg_cpp::Packet();
      frame_ = ffmpeg_cpp::Frame();
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
    return {make_interface_description("packet", "ffmpeg_cpp::Packet*"),
            make_interface_description("frame", "ffmpeg_cpp::Frame*")};
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
      RCLCPP_ERROR(get_logger(), "Failed to write the frame: %s", error.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  // Utility functions

  void print_packet() const {
    // TODO: Print more packet information
    RCLCPP_INFO(get_logger(), "Got packet with dts %ld", packet_->dts);
  }

  void print_frame() const {
    // TODO: Print more frame information
    RCLCPP_INFO(get_logger(), "Got frame with dts %ld", frame_->pkt_dts);
  }

protected:
  ffmpeg_cpp::Packet packet_;
  ffmpeg_cpp::Frame frame_;
  std::common_type_t<decltype(ffmpeg_cpp::Packet()->dts), decltype(ffmpeg_cpp::Frame()->pkt_dts)>
      prev_dts_;
};

} // namespace ffmpeg_hardware

#endif