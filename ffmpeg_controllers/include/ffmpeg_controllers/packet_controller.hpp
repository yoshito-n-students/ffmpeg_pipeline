#ifndef FFMPEG_CONTROLLERS_PACKET_CONTROLLER_HPP
#define FFMPEG_CONTROLLERS_PACKET_CONTROLLER_HPP

#include <optional>

#include <ffmpeg_controllers/packet_controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/packet.hpp>

namespace ffmpeg_controllers {

class PacketController : public PacketControllerBase<ffmpeg_pipeline_msgs::msg::Packet> {
private:
  using Base = PacketControllerBase<ffmpeg_pipeline_msgs::msg::Packet>;

public:
  PacketController() : Base(/* default_output_name = */ "output", /* topic = */ "~/packet") {}

protected:
  std::optional<ffmpeg_cpp::Packet>
  on_update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
            const ffmpeg_pipeline_msgs::msg::Packet & /*msg*/) override {
    ffmpeg_cpp::Packet packet;
    // TODO: convert the message to a packet
    return packet;
  }
};

} // namespace ffmpeg_controllers

#endif