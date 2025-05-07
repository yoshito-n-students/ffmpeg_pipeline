#ifndef FFMPEG_CONTROLLERS_ENCODER_FILTER_HPP
#define FFMPEG_CONTROLLERS_ENCODER_FLITER_HPP

#include <optional>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class EncoderFilter
    : public ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
                            std::tuple<output_options::Export<ffmpeg_cpp::Packet>,
                                       output_options::Export<ffmpeg_cpp::CodecParameters>>> {
private:
  using Base = ControllerBase<input_options::Read<ffmpeg_cpp::Frame>,
                              std::tuple<output_options::Export<ffmpeg_cpp::Packet>,
                                         output_options::Export<ffmpeg_cpp::CodecParameters>>>;

protected:
  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    try {
      encoder_name_ = get_user_parameter<std::string>("encoder_name", "");
      codec_params_ =
          ffmpeg_cpp::CodecParameters::create(get_user_parameter<std::string>("codec_parameters"));
      encoder_options_ =
          ffmpeg_cpp::Dictionary::create(get_user_parameter<std::string>("encoder_options", "{}"));
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while getting parameter value: %s", error.what());
      return NodeReturn::ERROR;
    }

    return NodeReturn::SUCCESS;
  }

  OnGenerateReturn<std::tuple<output_options::Export<ffmpeg_cpp::Packet>,
                              output_options::Export<ffmpeg_cpp::CodecParameters>>>
  on_generate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Frame &input_frame) override {
    try {
      // Ensure the encoder is configured for the codec
      if (!encoder_) {
        // Fill unspecified codec parameters with those from the source frame
        if (codec_params_->width <= 0) {
          codec_params_->width = input_frame->width;
        }
        if (codec_params_->height <= 0) {
          codec_params_->height = input_frame->height;
        }
        if (codec_params_->sample_rate <= 0) {
          codec_params_->sample_rate = input_frame->sample_rate;
        }

        // Configure the encoder with the codec parameters and options
        encoder_ = ffmpeg_cpp::Encoder::create(encoder_name_, codec_params_, encoder_options_);
        if (const std::string hw_type_name = encoder_.hw_type_name(); hw_type_name == "none") {
          RCLCPP_INFO(get_logger(), "Configured encoder (%s)", encoder_->codec->name);
        } else {
          RCLCPP_INFO(get_logger(), "Configured encoder (%s|%s)", encoder_->codec->name,
                      hw_type_name.c_str());
        }
      }

      // Put the raw frame into the encoder
      encoder_.send_frame(input_frame);

      // Extract as many packets as possible from the encoder and keep only the latest packet.
      // According to the ffmpeg's reference, there should be only one packet per frame
      // so no packets should be dropped.
      ffmpeg_cpp::Packet packet = ffmpeg_cpp::Packet::null();
      while (true) {
        if (ffmpeg_cpp::Packet tmp_packet = encoder_.receive_packet(); !tmp_packet.empty()) {
          packet = std::move(tmp_packet); // Keep the latest packet
        } else {
          break; // No more packets available
        }
      }
      if (packet.empty()) {
        RCLCPP_WARN(get_logger(), "No packets available although frame was processed");
        return {ControllerReturn::OK, std::nullopt};
      }

      // Move the encoded packet to the exported state interface
      return {ControllerReturn::OK,
              std::make_tuple(std::move(packet), ffmpeg_cpp::CodecParameters(codec_params_))};
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while encoding frame: %s", error.what());
      return {ControllerReturn::ERROR, std::nullopt};
    }
  }

protected:
  std::string encoder_name_;
  ffmpeg_cpp::CodecParameters codec_params_ = ffmpeg_cpp::CodecParameters::null();
  ffmpeg_cpp::Dictionary encoder_options_ = ffmpeg_cpp::Dictionary::null();
  ffmpeg_cpp::Encoder encoder_ = ffmpeg_cpp::Encoder::null();
};

} // namespace ffmpeg_controllers

#endif