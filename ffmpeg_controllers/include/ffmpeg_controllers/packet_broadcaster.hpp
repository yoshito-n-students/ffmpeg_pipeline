#ifndef FFMPEG_CONTROLLERS_PACKET_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_PACKET_BROADCASTER_HPP

#include <optional>
#include <string>

#include <ffmpeg_controllers/packet_broadcaster_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/packet.hpp>

namespace ffmpeg_controllers {

class PacketBroadcaster : public PacketBroadcasterBase<ffmpeg_pipeline_msgs::msg::Packet> {
private:
  using Base = PacketBroadcasterBase<ffmpeg_pipeline_msgs::msg::Packet>;

public:
  PacketBroadcaster() : Base(/* default_input_name = */ "input", /* topic = */ "~/packet") {}

protected:
  std::optional<Message> on_update(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/,
                                   const ffmpeg_cpp::CodecParameters &codec_params,
                                   const ffmpeg_cpp::Packet &packet) override {
    return packet.to_msg(get_node()->now(), codec_params.codec_name());
  }
};

} // namespace ffmpeg_controllers

#endif