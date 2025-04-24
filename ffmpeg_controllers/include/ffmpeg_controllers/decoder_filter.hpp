#ifndef FFMPEG_CONTROLLERS_DECODER_FILTER_HPP
#define FFMPEG_CONTROLLERS_DECODER_FLITER_HPP

#include <optional>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class DecoderFilter
    : public ControllerBase<input_options::ReadPacketWithParams, output_options::ExportFrame> {
private:
  using Base = ControllerBase<input_options::ReadPacketWithParams, output_options::ExportFrame>;

protected:
  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    try {
      codec_options_ =
          ffmpeg_cpp::Dictionary(get_node()->declare_parameter<std::string>("codec_options", "{}"));
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while getting parameter value: %s", error.what());
      return NodeReturn::ERROR;
    }

    return NodeReturn::SUCCESS;
  }

  std::pair<ControllerReturn, std::optional<Outputs>>
  on_generate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Packet &input_packet,
              const ffmpeg_cpp::CodecParameters &codec_params) override {
    try {
      // Ensure the decoder is configured for the codec
      if (!decoder_.valid()) {
        ffmpeg_cpp::Dictionary options(codec_options_); // Copy codec_options_ to avoid modifying it
        decoder_ = ffmpeg_cpp::Decoder(codec_params, &options);
        if (const std::string hw_type_name = decoder_.hw_type_name(); hw_type_name == "none") {
          RCLCPP_INFO(get_logger(), "Configured decoder (%s)", decoder_.codec_name().c_str());
        } else {
          RCLCPP_INFO(get_logger(), "Configured decoder (%s|%s)", decoder_.codec_name().c_str(),
                      hw_type_name.c_str());
        }
      }

      // Put the compressed data into the decoder
      decoder_.send_packet(input_packet);

      // Extract as many frames as possible from the decoder and keep only the latest frame.
      // According to the ffmpeg's reference, there should be only one frame per video packet
      // so no frames should be dropped.
      ffmpeg_cpp::Frame frame;
      while (true) {
        if (ffmpeg_cpp::Frame tmp_frame = decoder_.receive_frame(); !tmp_frame.empty()) {
          frame = std::move(tmp_frame); // Keep the latest frame
        } else {
          break; // No more frames available
        }
      }
      if (frame.empty()) {
        RCLCPP_WARN(get_logger(), "No frames available although packet was processed");
        return {ControllerReturn::OK, std::nullopt};
      }

      // If the frame data is in a hardware device,
      // transfer the data to the CPU-accessible memory before conversion
      if (frame.is_hw_frame()) {
        frame = frame.transfer_data();
      }

      // Move the decoded frame to the exported state interface
      return {ControllerReturn::OK, {std::move(frame)}};
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while decoding packet: %s", error.what());
      return {ControllerReturn::ERROR, std::nullopt};
    }
  }

protected:
  ffmpeg_cpp::Decoder decoder_;
  ffmpeg_cpp::Dictionary codec_options_;
};

} // namespace ffmpeg_controllers

#endif