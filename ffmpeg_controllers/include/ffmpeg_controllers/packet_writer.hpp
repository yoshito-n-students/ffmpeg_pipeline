#ifndef FFMPEG_CONTROLLERS_PACKET_WRITER_HPP
#define FFMPEG_CONTROLLERS_PACKET_WRITER_HPP

#include <optional>
#include <utility>

#include <ffmpeg_controllers/controller_base.hpp>

namespace ffmpeg_controllers {

class PacketWriter : public ControllerBase<input_options::ReadPacket, output_options::WritePacket> {
protected:
  std::pair<ControllerReturn, std::optional<Outputs>>
  on_generate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Packet &input_packet) override {
    return {ControllerReturn::OK, {input_packet}};
  }
};

} // namespace ffmpeg_controllers

#endif