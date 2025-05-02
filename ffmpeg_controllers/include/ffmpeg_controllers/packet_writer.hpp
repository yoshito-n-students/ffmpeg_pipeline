#ifndef FFMPEG_CONTROLLERS_PACKET_WRITER_HPP
#define FFMPEG_CONTROLLERS_PACKET_WRITER_HPP

#include <optional>
#include <utility>

#include <ffmpeg_controllers/controller_base.hpp>

namespace ffmpeg_controllers {

class PacketWriter
    : public ControllerBase<std::tuple<input_options::Read<ffmpeg_cpp::Packet>,
                                       input_options::Read<ffmpeg_cpp::CodecParameters>>,
                            std::tuple<output_options::Write<ffmpeg_cpp::Packet>,
                                       output_options::Write<ffmpeg_cpp::CodecParameters>>> {
protected:
  OnGenerateReturn<std::tuple<output_options::Write<ffmpeg_cpp::Packet>,
                              output_options::Write<ffmpeg_cpp::CodecParameters>>>
  on_generate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Packet &input_packet,
              const ffmpeg_cpp::CodecParameters &input_params) override {
    return {ControllerReturn::OK, std::make_tuple(input_packet, input_params)};
  }
};

} // namespace ffmpeg_controllers

#endif