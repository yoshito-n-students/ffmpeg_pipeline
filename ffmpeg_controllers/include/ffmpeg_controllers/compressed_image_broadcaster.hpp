#ifndef FFMPEG_CONTROLLERS_COMPRESSED_IMAGE_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_COMPRESSED_IMAGE_BROADCASTER_HPP

#include <optional>
#include <string>

#include <ffmpeg_controllers/packet_broadcaster_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ffmpeg_controllers {

class CompressedImageBroadcaster : public PacketBroadcasterBase<sensor_msgs::msg::CompressedImage> {
private:
  using Base = PacketBroadcasterBase<sensor_msgs::msg::CompressedImage>;

public:
  CompressedImageBroadcaster()
      : Base(/* default_input_name = */ "camera", /* topic = */ "~/image/ffmpeg") {}

protected:
  std::optional<Message> on_update(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/,
                                   const ffmpeg_cpp::CodecParameters &codec_params,
                                   const ffmpeg_cpp::Packet &packet) override {
    Message msg;
    msg.header.stamp.sec = packet->dts / 1'000'000;
    msg.header.stamp.nanosec = (packet->dts % 1'000'000) * 1'000;
    msg.format = codec_params.codec_name();
    msg.data.assign(packet->data, packet->data + packet->size);
    return msg;
  }
};

} // namespace ffmpeg_controllers

#endif