#ifndef FFMPEG_CONTROLLERS_COMPRESSED_IMAGE_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_COMPRESSED_IMAGE_BROADCASTER_HPP

#include <optional>
#include <utility> // for std::move()

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ffmpeg_controllers {

class CompressedImageBroadcaster
    : public ControllerBase<std::tuple<input_options::Read<ffmpeg_cpp::Packet>,
                                       input_options::Read<ffmpeg_cpp::CodecParameters>>,
                            output_options::Publish<sensor_msgs::msg::CompressedImage>> {
protected:
  OnGenerateReturn<output_options::Publish<sensor_msgs::msg::CompressedImage>>
  on_generate(const rclcpp::Time &time, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Packet &input_packet,
              const ffmpeg_cpp::CodecParameters &codec_params) override {
    OutputMessage msg;
    msg.header.stamp = time;
    msg.format = codec_params.codec_name();
    msg.data.assign(input_packet->data, input_packet->data + input_packet->size);
    return {ControllerReturn::OK, std::move(msg)};
  }
};

} // namespace ffmpeg_controllers

#endif