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
                 url = get_parameter_as<std::string>("url", "default");
      const auto codec_params = get_parameter_as<ffmpeg_cpp::CodecParameters>(
          "codec_parameters",
          ffmpeg_cpp::CodecParameters::create(
              "{codec: pcm_s16, format: s16, ch_layout: stereo, sample_rate: 48000}"));
      const auto options =
          get_parameter_as<ffmpeg_cpp::Dictionary>("options", ffmpeg_cpp::Dictionary::create("{}"));

      // [EXPERIMENTAL] Try to complete the codec parameters (extradata, etc)
      try {
        ffmpeg_cpp::Encoder encoder = ffmpeg_cpp::Encoder::create("", codec_params);
        if (const int ret = avcodec_parameters_from_context(codec_params.get(), encoder.get())) {
          throw ffmpeg_cpp::Error("Failed to copy codec parameters from the encoder", ret);
        }
      } catch (const std::runtime_error &error) {
        RCLCPP_WARN(get_logger(), "Skipped to complete the codec parameters: %s", error.what());
      }

      // Open the input with the parameters
      output_ = ffmpeg_cpp::Output::create(format, url, codec_params, options);
      RCLCPP_INFO(get_logger(), "Configured the output ([%s] %s)", output_.format_name().c_str(),
                  output_.url().c_str());

      // Initialize the command variables
      packet_ = ffmpeg_cpp::Packet::create();
      frame_ = ffmpeg_cpp::Frame::create();
      packet_->dts = frame_->pkt_dts = prev_dts_ = 0;

      // Register the command variables to the interface
      set_command_from_pointer("packet", &packet_);
      set_command_from_pointer("frame", &frame_);

      return CallbackReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to configure the output device: %s", error.what());
      return CallbackReturn::ERROR;
    }
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      // Unregister the command variables to the interface
      set_command_from_pointer("packet", nullptr);
      set_command_from_pointer("frame", nullptr);

      // Free the command variables
      packet_ = ffmpeg_cpp::Packet::null();
      frame_ = ffmpeg_cpp::Frame::null();

      // Close the output device
      output_ = ffmpeg_cpp::Output::null();
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
          output_.write_uncoded_frame(frame_);
          output_.write_frame(packet_);
          prev_dts_ = packet_->dts;
        } else {
          // prev < packet <= frame
          output_.write_frame(packet_);
          output_.write_uncoded_frame(frame_);
          prev_dts_ = frame_->pkt_dts;
        }
      } else if (packet_->dts > prev_dts_) {
        // frame <= prev < packet
        output_.write_frame(packet_);
        prev_dts_ = packet_->dts;
      } else if (frame_->pkt_dts > prev_dts_) {
        // packet <= prev < frame
        output_.write_uncoded_frame(frame_);
        prev_dts_ = frame_->pkt_dts;
      }
      return hardware_interface::return_type::OK;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to write the frame: %s", error.what());
      return hardware_interface::return_type::ERROR;
    }
  }

protected:
  ffmpeg_cpp::Output output_ = ffmpeg_cpp::Output::null();
  ffmpeg_cpp::Packet packet_ = ffmpeg_cpp::Packet::null();
  ffmpeg_cpp::Frame frame_ = ffmpeg_cpp::Frame::null();
  std::int64_t prev_dts_ = 0;
};

} // namespace ffmpeg_hardware

#endif