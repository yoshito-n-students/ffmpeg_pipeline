#ifndef FFMPEG_CONTROLLERS_PACKET_BROADCASTER_HPP
#define FFMPEG_CONTROLLERS_PACKET_BROADCASTER_HPP

#include <optional>

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/packet.hpp>

namespace ffmpeg_controllers {

class PacketBroadcaster : public BroadcasterBase<ffmpeg_pipeline_msgs::msg::Packet> {
private:
  using Base = BroadcasterBase<ffmpeg_pipeline_msgs::msg::Packet>;

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

  NodeReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
    // Activate the base class first
    if (const NodeReturn base_ret = Base::on_activate(previous_state);
        base_ret != NodeReturn::SUCCESS) {
      return base_ret;
    }

    // Reset the previous dts
    prev_dts_ = 0;

    return NodeReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    // Usually the broadcaster does not write to command interfaces
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {input_name_ + "/codec_parameters", input_name_ + "/packet"}};
  }

  std::optional<OutputMessage> on_generate(const rclcpp::Time &time,
                                     const rclcpp::Duration & /*period*/) override {
    // Try to get the codec params and packet from the state interfaces
    const ffmpeg_cpp::CodecParameters *const codec_params =
        get_state_as_pointer<ffmpeg_cpp::CodecParameters>("codec_parameters");
    const ffmpeg_cpp::Packet *const packet = get_state_as_pointer<ffmpeg_cpp::Packet>("packet");
    if (!codec_params || !packet) {
      RCLCPP_WARN(get_logger(), "Failed to get codec parameters or packet. Will skip this update.");
      return std::nullopt;
    }

    // Skip publishing if the packet is not new
    if ((*packet)->dts <= prev_dts_) {
      return std::nullopt;
    }

    // Generate the message with the new packet
    prev_dts_ = (*packet)->dts;
    return packet->to_msg(time, codec_params->codec_name());
  }

protected:
  decltype(ffmpeg_cpp::Packet()->dts) prev_dts_;
};

} // namespace ffmpeg_controllers

#endif