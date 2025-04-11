#ifndef FFMPEG_CONTROLLERS_AUDIO_PACKET_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_AUDIO_PACKET_BROADCASTER_HPP

#include <optional>
#include <string>

#include <ffmpeg_controllers/broadcaster_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/packet.hpp>

namespace ffmpeg_controllers {

class AudioPacketBroadcaster : public BroadcasterBase<ffmpeg_pipeline_msgs::msg::Packet> {
public:
  AudioPacketBroadcaster()
      : BroadcasterBase<ffmpeg_pipeline_msgs::msg::Packet>(/* default_sensor_name = */ "mic",
                                                           /* topic = */ "~/packet") {}

protected:
  std::optional<ffmpeg_pipeline_msgs::msg::Packet>
  on_update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
            const std::string &codec_name, const ffmpeg_cpp::Packet &packet) override {
    ffmpeg_pipeline_msgs::msg::Packet msg;
    msg.header.stamp.sec = packet->pts / 1'000'000;
    msg.header.stamp.nanosec = (packet->pts % 1'000'000) * 1'000;
    msg.codec = codec_name;
    msg.data.assign(packet->data, packet->data + packet->size);
    return msg;
  }
};

} // namespace ffmpeg_controllers

#endif