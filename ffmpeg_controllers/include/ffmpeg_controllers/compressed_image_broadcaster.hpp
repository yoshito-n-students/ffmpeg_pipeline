#ifndef FFMPEG_CONTROLLERS_COMPRESSED_IMAGE_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_COMPRESSED_IMAGE_BROADCASTER_HPP

#include <optional>
#include <utility> // for std::move()

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ffmpeg_controllers {

class CompressedImageBroadcaster
    : public ControllerBase<input_options::ReadPacketWithParams,
                            output_options::Publish<sensor_msgs::msg::CompressedImage>> {
private:
  using Base = ControllerBase<input_options::ReadPacketWithParams,
                              output_options::Publish<sensor_msgs::msg::CompressedImage>>;

protected:
  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Name of interprocess topic to publish
    output_topic_ = "~/image/ffmpeg";

    return NodeReturn::SUCCESS;
  }

  std::pair<ControllerReturn, std::optional<Outputs>>
  on_generate(const rclcpp::Time &time, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Packet &input_packet,
              const ffmpeg_cpp::CodecParameters &codec_params) override {
    OutputMessage msg;
    msg.header.stamp = time;
    msg.format = codec_params.codec_name();
    msg.data.assign(input_packet->data, input_packet->data + input_packet->size);
    return {ControllerReturn::OK, {std::move(msg)}};
  }
};

} // namespace ffmpeg_controllers

#endif