#ifndef FFMPEG_CONTROLLERS_FRAME_BROADCASTER_BASE_HPP
#define FFMPEG_CONTROLLERS_FRAME_BROADCASTER_BASE_HPP

#include <optional>
#include <stdexcept>
#include <string>

#include <ffmpeg_controllers/packet_broadcaster_base.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/logging.hpp>

namespace ffmpeg_controllers {

template <typename MessageT> class FrameBroadcasterBase : public PacketBroadcasterBase<MessageT> {
private:
  using Base = PacketBroadcasterBase<MessageT>;

public:
  using Base::PacketBroadcasterBase;

protected:
  std::optional<typename Base::Message> on_update(const rclcpp::Time &time,
                                                  const rclcpp::Duration &period,
                                                  const ffmpeg_cpp::CodecParameters &codec_params,
                                                  const ffmpeg_cpp::Packet &packet) override final {
    try {
      // Ensure the decoder is configured for the codec
      if (!decoder_.valid()) {
        decoder_ = ffmpeg_cpp::Decoder(codec_params);
        RCLCPP_INFO(Base::get_logger(), "Configured decoder (codec: %s, hw: %s)",
                    decoder_.codec_name().c_str(), decoder_.hw_type_name().c_str());
      }

      // Put the compressed data into the decoder
      decoder_.send_packet(packet);

      // Extract as many frames as possible from the decoder and keep only the latest frame.
      // According to the ffmpeg's reference, there should be only one frame per video packet
      // so no frames should be dropped.
      ffmpeg_cpp::Frame frame;
      while (true) {
        ffmpeg_cpp::Frame tmp_frame = decoder_.receive_frame();
        if (!tmp_frame.empty()) {
          frame = std::move(tmp_frame); // Keep the latest frame
        } else {
          break; // No more frames available
        }
      }
      if (frame.empty()) {
        RCLCPP_WARN(Base::get_logger(), "No frames available after decoding packet");
        return std::nullopt;
      }

      // If the frame data is in a hardware device,
      // transfer the data to the CPU-accessible memory before conversion
      if (frame.is_hw_frame()) {
        frame = frame.transfer_data();
      }

      return on_update(time, period, frame);
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while decoding packet: %s", error.what());
      return std::nullopt;
    }
  }

  virtual std::optional<typename Base::Message> on_update(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period,
                                                          const ffmpeg_cpp::Frame &frame) = 0;

protected:
  ffmpeg_cpp::Decoder decoder_;
};

} // namespace ffmpeg_controllers

#endif