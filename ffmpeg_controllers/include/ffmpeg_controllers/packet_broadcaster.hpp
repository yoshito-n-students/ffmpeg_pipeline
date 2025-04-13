#ifndef FFMPEG_CONTROLLERS_PACKET_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_PACKET_BROADCASTER_HPP

#include <optional>
#include <string>

#include <ffmpeg_controllers/broadcaster_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/packet.hpp>

namespace ffmpeg_controllers {

class PacketBroadcaster : public BroadcasterBase<ffmpeg_pipeline_msgs::msg::Packet> {
public:
  PacketBroadcaster() : Base(/* default_sensor_name = */ "sensor", /* topic = */ "~/packet") {}

protected:
  std::optional<Message> on_update(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/,
                                   const ffmpeg_cpp::CodecParameters &codec_params,
                                   const ffmpeg_cpp::Packet &packet) override {
    Message msg;
    msg.header.stamp.sec = packet->pts / 1'000'000;
    msg.header.stamp.nanosec = (packet->pts % 1'000'000) * 1'000;
    msg.codec = codec_params.codec_name();
    msg.data.assign(packet->data, packet->data + packet->size);
    return msg;
  }
};

} // namespace ffmpeg_controllers

#endif