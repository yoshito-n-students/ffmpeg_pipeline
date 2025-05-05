#ifndef FFMPEG_CONTROLLERS_PARSER_FILTER_HPP
#define FFMPEG_CONTROLLERS_PARSER_FLITER_HPP

#include <optional>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class ParserFilter
    : public ControllerBase<input_options::Read<ffmpeg_cpp::Packet>,
                            std::tuple<output_options::Export<ffmpeg_cpp::Packet>,
                                       output_options::Export<ffmpeg_cpp::CodecParameters>>> {
private:
  using Base = ControllerBase<input_options::Read<ffmpeg_cpp::Packet>,
                              std::tuple<output_options::Export<ffmpeg_cpp::Packet>,
                                         output_options::Export<ffmpeg_cpp::CodecParameters>>>;

protected:
  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    try {
      parser_ = ffmpeg_cpp::Parser::create(get_user_parameter<std::string>("codec_name"));
      RCLCPP_INFO(get_logger(), "Configured parser (%s)", parser_.codec_names().front().c_str());
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while initializing parser: %s", error.what());
      return NodeReturn::ERROR;
    }

    return NodeReturn::SUCCESS;
  }

  OnGenerateReturn<std::tuple<output_options::Export<ffmpeg_cpp::Packet>,
                              output_options::Export<ffmpeg_cpp::CodecParameters>>>
  on_generate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Packet &input_packet) override {
    try {
      // Try to find the packet and codec parameters until the buffer is fully parsed
      std::int64_t pos = 0;
      while (pos < input_packet->size) {
        ffmpeg_cpp::Packet output_packet = ffmpeg_cpp::Packet::null();
        if (output_params_.codec_name() == "none") {
          std::tie(output_packet, output_params_) =
              parser_.parse_initial_packet(input_packet, &pos);
        } else {
          output_packet = parser_.parse_next_packet(input_packet, &pos);
        }
        if (!output_packet.empty()) {
          return {ControllerReturn::OK, std::make_tuple(std::move(output_packet), output_params_)};
        }
      }
      // Return OK with no packet if no packet is found
      return {ControllerReturn::OK, std::nullopt};
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while decoding packet: %s", error.what());
      return {ControllerReturn::ERROR, std::nullopt};
    }
  }

protected:
  ffmpeg_cpp::Parser parser_ = ffmpeg_cpp::Parser::null();
  ffmpeg_cpp::CodecParameters output_params_ = ffmpeg_cpp::CodecParameters::null();
};

} // namespace ffmpeg_controllers

#endif