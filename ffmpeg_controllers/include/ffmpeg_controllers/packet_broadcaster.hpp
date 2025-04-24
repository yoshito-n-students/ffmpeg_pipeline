#ifndef FFMPEG_CONTROLLERS_PACKET_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_PACKET_BROADCASTER_HPP

#include <optional>

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/packet.hpp>

namespace ffmpeg_controllers {

class PacketBroadcaster
    : public ControllerBase<input_options::ReadPacketWithParams,
                            output_options::Publish<ffmpeg_pipeline_msgs::msg::Packet>> {
private:
  using Base = ControllerBase<input_options::ReadPacketWithParams,
                              output_options::Publish<ffmpeg_pipeline_msgs::msg::Packet>>;

protected:
  NodeReturn on_init() override {
    // Initialize the base class first
    if (const NodeReturn base_ret = Base::on_init(); base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Name of interprocess topic to publish
    output_topic_ = "~/packet";

    return NodeReturn::SUCCESS;
  }

  std::pair<ControllerReturn, std::optional<Outputs>>
  on_generate(const rclcpp::Time &time, const rclcpp::Duration & /*period*/,
              const ffmpeg_cpp::Packet &input_packet,
              const ffmpeg_cpp::CodecParameters &codec_params) override {
    // Generate the message with the new packet
    return {ControllerReturn::OK, {input_packet.to_msg(time, codec_params.codec_name())}};
  }
};

} // namespace ffmpeg_controllers

#endif