#ifndef FFMPEG_CONTROLLERS_PACKET_RECEIVER_HPP
#define FFMPEG_CONTROLLERS_PACKET_RECEIVER_HPP

#include <optional>
#include <stdexcept>
#include <utility>

#include <ffmpeg_controllers/controller_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <ffmpeg_pipeline_msgs/msg/packet.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

class PacketReceiver
    : public ControllerBase<input_options::Subscribe<ffmpeg_pipeline_msgs::msg::Packet>,
                            output_options::Export<ffmpeg_cpp::Packet>> {
protected:
  OnGenerateReturn<output_options::Export<ffmpeg_cpp::Packet>>
  on_generate(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
              const ffmpeg_pipeline_msgs::msg::Packet &input_msg) override {
    try {
      return {ControllerReturn::OK, ffmpeg_cpp::Packet::create(input_msg)};
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Error while updating packet: %s", error.what());
      return {ControllerReturn::ERROR, std::nullopt};
    }
  }
};

} // namespace ffmpeg_controllers

#endif