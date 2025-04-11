#ifndef FFMPEG_CONTROLLERS_AUDIO_PACKET_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_AUDIO_PACKET_BROADCASTER_HPP

#include <optional>
#include <string>

#include <ffmpeg_controllers/broadcaster_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ffmpeg_controllers {

// TODO: Use more appropriate message type for audio data
class AudioPacketBroadcaster : public BroadcasterBase<sensor_msgs::msg::CompressedImage> {
public:
  AudioPacketBroadcaster()
      : BroadcasterBase<sensor_msgs::msg::CompressedImage>(/* default_sensor_name = */ "mic",
                                                           /* topic = */ "~/packet") {}

protected:
  std::optional<sensor_msgs::msg::CompressedImage>
  on_update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
            const std::string &codec_name, const ffmpeg_cpp::Packet &packet) override {
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp.sec = packet->pts / 1'000'000;
    msg.header.stamp.nanosec = (packet->pts % 1'000'000) * 1'000;
    msg.format = codec_name;
    msg.data.assign(packet->data, packet->data + packet->size);
    return msg;
  }
};

} // namespace ffmpeg_controllers

#endif