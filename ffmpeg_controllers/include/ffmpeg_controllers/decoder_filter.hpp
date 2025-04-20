#ifndef FFMPEG_CONTROLLERS_DECODER_FILTER_HPP
#define FFMPEG_CONTROLLERS_DECODER_FLITER_HPP

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_controllers {

class DecoderFilter : public FilterBase {
protected:
  // ===================
  // AS a lifecycle node
  // ===================

  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = FilterBase::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Names of intraprocess read-only variables to be exported
    exported_state_interface_names_ = {"frame"};

    return NodeReturn::SUCCESS;
  }

  NodeReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
    // Activate the base class first
    if (const NodeReturn base_ret = FilterBase::on_activate(previous_state);
        base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Register the frame to state interface owned by this controller
    if (!set_state_from_pointer("frame", &frame_)) {
      return NodeReturn::ERROR;
    }

    // Reset the previous dts
    prev_dts_ = 0;

    return NodeReturn::SUCCESS;
  }

  NodeReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
    // Unregister the frame from state interface owned by this controller
    if (!set_state_from_pointer("frame", nullptr)) {
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
            {input_name_ + "/codec_parameters", input_name_ + "/packet"}};
  }

  ControllerReturn on_update(const rclcpp::Time & /*time*/,
                             const rclcpp::Duration & /*period*/) override {
    // Try to get the input packet owned by the hardware or other chained controller
    const ffmpeg_cpp::CodecParameters *const codec_params =
        get_state_as_pointer<ffmpeg_cpp::CodecParameters>("codec_parameters");
    const ffmpeg_cpp::Packet *const packet = get_state_as_pointer<ffmpeg_cpp::Packet>("packet");
    if (!codec_params || !packet) {
      RCLCPP_WARN(get_logger(),
                  "Failed to get input codec parameters or packet. Will skip this update.");
      return ControllerReturn::OK;
    }

    // Skip decoding the packet if it is not new
    if ((*packet)->dts <= prev_dts_) {
      return ControllerReturn::OK;
    }

    try {
      // Ensure the decoder is configured for the codec
      if (!decoder_.valid()) {
        decoder_ = ffmpeg_cpp::Decoder(*codec_params);
        RCLCPP_INFO(get_logger(), "Configured decoder (codec: %s, hw: %s)",
                    decoder_.codec_name().c_str(), decoder_.hw_type_name().c_str());
      }

      // Put the compressed data into the decoder
      decoder_.send_packet(*packet);

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
        return ControllerReturn::OK;
      }

      // If the frame data is in a hardware device,
      // transfer the data to the CPU-accessible memory before conversion
      if (frame.is_hw_frame()) {
        frame = frame.transfer_data();
      }

      // Move the decoded frame to the exported state interface
      frame_ = std::move(frame);
      prev_dts_ = frame_->pkt_dts;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while decoding packet: %s", error.what());
      return ControllerReturn::ERROR;
    }

    return ControllerReturn::OK;
  }

protected:
  ffmpeg_cpp::Decoder decoder_;
  ffmpeg_cpp::Frame frame_;
  decltype(ffmpeg_cpp::Packet()->dts) prev_dts_;
};

} // namespace ffmpeg_controllers

#endif