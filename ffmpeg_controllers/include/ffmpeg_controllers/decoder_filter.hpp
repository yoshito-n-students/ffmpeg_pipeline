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
    : public ControllerBase<std::tuple<input_options::Read<ffmpeg_cpp::Packet>,
                                       input_options::Read<ffmpeg_cpp::CodecParameters>>,
                            output_options::Export<ffmpeg_cpp::Frame>> {
private:
  using Base = ControllerBase<std::tuple<input_options::Read<ffmpeg_cpp::Packet>,
                                         input_options::Read<ffmpeg_cpp::CodecParameters>>,
                              output_options::Export<ffmpeg_cpp::Frame>>;

protected:
  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    try {
      decoder_name_ = get_user_parameter<std::string>("decoder_name", "");
      hw_type_name_ = get_user_parameter<std::string>("hw_type_name", "auto");
      request_format_name_ = get_user_parameter<std::string>("request_format_name", "");
      decoder_options_ =
          ffmpeg_cpp::Dictionary::create(get_user_parameter<std::string>("decoder_options", "{}"));
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while getting parameter value: %s", error.what());
      return NodeReturn::ERROR;
    }

    return NodeReturn::SUCCESS;
  }

  OnGenerateReturn<output_options::Export<ffmpeg_cpp::Frame>>
  on_generate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Packet &input_packet,
              const ffmpeg_cpp::CodecParameters &codec_params) override {
    try {
      // Ensure the decoder is configured for the codec
      if (!decoder_) {
        decoder_ = ffmpeg_cpp::Decoder::create(decoder_name_, codec_params, hw_type_name_,
                                               request_format_name_, decoder_options_);
        if (const std::string hw_type_name = decoder_.hw_type_name(); hw_type_name.empty()) {
          RCLCPP_INFO(get_logger(), "Configured decoder (%s)", decoder_->codec->name);
        } else {
          RCLCPP_INFO(get_logger(), "Configured decoder (%s|%s)", decoder_->codec->name,
                      hw_type_name.c_str());
        }
      }

      // Put the compressed data into the decoder
      decoder_.send_packet(input_packet);

      // Extract as many frames as possible from the decoder and keep only the latest frame.
      // According to the ffmpeg's reference, there should be only one frame per video packet
      // so no frames should be dropped.
      ffmpeg_cpp::Frame output_frame = ffmpeg_cpp::Frame::null();
      while (true) {
        if (ffmpeg_cpp::Frame incoming_frame = decoder_.receive_frame(); !incoming_frame.empty()) {
          if (!output_frame.empty()) {
            RCLCPP_WARN(get_logger(),
                        "Multiple frames decoded from a single packet. Will keep the latest one.");
          }
          output_frame = std::move(incoming_frame); // Keep the latest frame
        } else {
          break; // No more frames available
        }
      }
      if (output_frame.empty()) {
        RCLCPP_WARN(get_logger(), "No frames available although packet was processed");
        return {ControllerReturn::OK, std::nullopt};
      }

      // If the frame data is in a hardware device,
      // transfer the data to the CPU-accessible memory before conversion
      if (output_frame.is_hw_frame()) {
        output_frame = output_frame.transfer_data();
      }

      // Move the decoded frame to the exported state interface
      return {ControllerReturn::OK, std::move(output_frame)};
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while decoding packet: %s", error.what());
      return {ControllerReturn::ERROR, std::nullopt};
    }
  }

protected:
  std::string decoder_name_, hw_type_name_, request_format_name_;
  ffmpeg_cpp::Dictionary decoder_options_ = ffmpeg_cpp::Dictionary::null();
  ffmpeg_cpp::Decoder decoder_ = ffmpeg_cpp::Decoder::null();
};

} // namespace ffmpeg_controllers

#endif