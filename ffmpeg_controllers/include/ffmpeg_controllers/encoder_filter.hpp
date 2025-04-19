#ifndef FFMPEG_CONTROLLERS_ENCODER_FILTER_HPP
#define FFMPEG_CONTROLLERS_ENCODER_FLITER_HPP

#include <string>

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_controllers {

class EncoderFilter : public FilterBase {
protected:
  // ===================
  // AS a lifecycle node
  // ===================

  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = FilterBase::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    try {
      codec_params_ = ffmpeg_cpp::CodecParameters(
          get_node()->declare_parameter<std::string>("codec_parameters"));
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while getting parameter value: %s", error.what());
      return CallbackReturn::ERROR;
    }

    // Names of intraprocess read-only variables to be exported
    exported_state_interface_names_ = {"codec_parameters", "packet"};

    return NodeReturn::SUCCESS;
  }

  NodeReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
    // Activate the base class first
    if (const NodeReturn base_ret = FilterBase::on_activate(previous_state);
        base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Register the packet to state interface owned by this controller
    if (!set_state_from_pointer("codec_parameters", &codec_params_) ||
        !set_state_from_pointer("packet", &packet_)) {
      return NodeReturn::ERROR;
    }

    // Reset the previous dts
    prev_dts_ = 0;

    return NodeReturn::SUCCESS;
  }

  NodeReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
    // Unregister the frame from state interface owned by this controller
    if (!set_state_from_pointer("codec_parameters",
                                static_cast<ffmpeg_cpp::CodecParameters *>(nullptr)) ||
        !set_state_from_pointer("packet", static_cast<ffmpeg_cpp::Packet *>(nullptr))) {
      return NodeReturn::ERROR;
    }

    // Deactivate the base class in the end
    return FilterBase::on_deactivate(previous_state);
  }

  // ===============
  // As a controller
  // ===============

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {input_name_ + "/frame"}};
  }

  ControllerReturn on_update(const rclcpp::Time & /*time*/,
                             const rclcpp::Duration & /*period*/) override {
    // Try to get the input frame owned by the hardware or other chained controller
    const ffmpeg_cpp::Frame *const frame = get_state_as_pointer<ffmpeg_cpp::Frame>("frame");
    if (!frame) {
      RCLCPP_WARN(get_logger(), "Failed to get input frame. Will skip this update.");
      return ControllerReturn::OK;
    }

    // Skip encoding the frame if it is not new
    if ((*frame)->pkt_dts <= prev_dts_) {
      return ControllerReturn::OK;
    }

    try {
      // Ensure the encoder is configured for the codec
      if (!encoder_.valid()) {
        // Fill unspecified codec parameters with those from the source frame
        if (codec_params_->width <= 0) {
          codec_params_->width = (*frame)->width;
        }
        if (codec_params_->height <= 0) {
          codec_params_->height = (*frame)->height;
        }
        if (codec_params_->sample_rate <= 0) {
          codec_params_->sample_rate = (*frame)->sample_rate;
        }

        encoder_ = ffmpeg_cpp::Encoder(codec_params_);
        RCLCPP_INFO(get_logger(), "Configured encoder (codec: %s, hw: %s)",
                    encoder_.codec_name().c_str(), encoder_.hw_type_name().c_str());
      }

      // Put the raw frame into the encoder
      encoder_.send_frame(*frame);

      // Extract as many packets as possible from the encoder and keep only the latest packet.
      // According to the ffmpeg's reference, there should be only one packet per frame
      // so no packets should be dropped.
      ffmpeg_cpp::Packet packet;
      while (true) {
        if (ffmpeg_cpp::Packet tmp_packet = encoder_.receive_packet(); !tmp_packet.empty()) {
          packet = std::move(tmp_packet); // Keep the latest packet
        } else {
          break; // No more packets available
        }
      }
      if (packet.empty()) {
        RCLCPP_WARN(get_logger(), "No packets available although frame was processed");
        return ControllerReturn::OK;
      }

      // Move the encoded packet to the exported state interface
      packet_ = std::move(packet);
      prev_dts_ = packet_->dts;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while encoding frame: %s", error.what());
      return ControllerReturn::ERROR;
    }

    return ControllerReturn::OK;
  }

protected:
  ffmpeg_cpp::Encoder encoder_;
  ffmpeg_cpp::CodecParameters codec_params_;
  ffmpeg_cpp::Packet packet_;
  decltype(ffmpeg_cpp::Packet()->dts) prev_dts_;
};

} // namespace ffmpeg_controllers

#endif