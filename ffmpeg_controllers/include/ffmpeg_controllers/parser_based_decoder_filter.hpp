#ifndef FFMPEG_CONTROLLERS_PARSER_BASED_DECODER_FILTER_HPP
#define FFMPEG_CONTROLLERS_PARSER_BASED_DECODER_FLITER_HPP

#include <optional>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class ParserBasedDecoderFilter : public ControllerBase<input_options::Read<ffmpeg_cpp::Packet>,
                                                       output_options::Export<ffmpeg_cpp::Frame>> {
private:
  using Base = ControllerBase<input_options::Read<ffmpeg_cpp::Packet>,
                              output_options::Export<ffmpeg_cpp::Frame>>;

protected:
  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    try {
      codec_name_ = get_user_parameter<std::string>("codec_name", "");
      codec_options_ =
          ffmpeg_cpp::Dictionary(get_user_parameter<std::string>("codec_options", "{}"));
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while getting parameter value: %s", error.what());
      return NodeReturn::ERROR;
    }

    return NodeReturn::SUCCESS;
  }

  OnGenerateReturn<output_options::Export<ffmpeg_cpp::Frame>>
  on_generate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Packet &input_packet) override {
    try {
      // Ensure the parser and decoder are configured for the codec
      if (!parser_.valid()) {
        parser_ = ffmpeg_cpp::Parser(codec_name_);
        RCLCPP_INFO(get_logger(), "Configured parser (%s)", parser_.codec_names().front().c_str());
      }
      if (!decoder_.valid()) {
        ffmpeg_cpp::Dictionary options(codec_options_); // Copy codec_options_ to avoid modifying it
        decoder_ = ffmpeg_cpp::Decoder(codec_name_, &options);
        if (const std::string hw_type_name = decoder_.hw_type_name(); hw_type_name == "none") {
          RCLCPP_INFO(get_logger(), "Configured decoder (%s)", decoder_.codec_name().c_str());
        } else {
          RCLCPP_INFO(get_logger(), "Configured decoder (%s|%s)", decoder_.codec_name().c_str(),
                      hw_type_name.c_str());
        }
      }

      // Get the decompressed frame from the input packet
      ffmpeg_cpp::Frame output_frame;
      {
        // Copy the input packet to the reference-counted buffer with padding
        ffmpeg_cpp::BufferRef buffer(input_packet->data, input_packet->size);
        // Proccess all the data in the buffer
        while (buffer.unpadded_size() > 0) {
          // Parse the buffer to further configure the decoder and get the packet
          const ffmpeg_cpp::Packet parsed_packet = parser_.parse(&buffer, &decoder_);
          if (parsed_packet.empty()) {
            continue;
          }
          // Decode the parsed packet to get the frame
          decoder_.send_packet(parsed_packet);
          while (true) {
            ffmpeg_cpp::Frame received_frame = decoder_.receive_frame();
            if (received_frame.empty()) {
              break; // No more frames available
            }
            output_frame = std::move(received_frame); // Keep the latest frame
          }
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
      // TODO: Handle the timestamps correctly
      output_frame->pts = output_frame->pkt_dts = input_packet->dts;
      return {ControllerReturn::OK, std::move(output_frame)};
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while decoding packet: %s", error.what());
      return {ControllerReturn::ERROR, std::nullopt};
    }
  }

protected:
  ffmpeg_cpp::Parser parser_;
  ffmpeg_cpp::Decoder decoder_;
  std::string codec_name_;
  ffmpeg_cpp::Dictionary codec_options_;
};

} // namespace ffmpeg_controllers

#endif