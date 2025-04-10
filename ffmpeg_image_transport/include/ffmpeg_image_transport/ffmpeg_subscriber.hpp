#ifndef FFMPEG_IMAGE_TRANSPORT_FFMPEG_SUBSCRIBER_HPP
#define FFMPEG_IMAGE_TRANSPORT_FFMPEG_SUBSCRIBER_HPP

#include <memory>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <image_transport/simple_subscriber_plugin.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ffmpeg_image_transport {

class FFmpegSubscriber
    : public image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage> {
private:
  using Base = image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage>;

public:
  std::string getTransportName() const override { return "ffmpeg"; }

protected:
  void subscribeImpl(rclcpp::Node *node, const std::string &base_topic, const Callback &callback,
                     rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions options) override {
    Base::subscribeImpl(node, base_topic, callback, custom_qos, options);
    node_ = node;
  }

  void internalCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &fragment,
                        const Callback &image_cb) override {
    try {
      // Configure the parser and decoder for this fragment if needed
      if (!parser_.is_supported(fragment->format)) {
        parser_ = ffmpeg_cpp::Parser(fragment->format);
        RCLCPP_INFO(node_->get_logger(), "Configured parser (codec: %s)",
                    parser_.codec_names().front().c_str());
      }
      if (!decoder_.is_supported(fragment->format)) {
        decoder_ = ffmpeg_cpp::Decoder(fragment->format);
        RCLCPP_INFO(node_->get_logger(), "Configured decoder (codec: %s, hw: %s)",
                    decoder_.codec_name().c_str(), decoder_.hw_device_type().c_str());
      }

      // Copy the data fragment to the reference-counted buffer with padding
      ffmpeg_cpp::BufferRef buffer(fragment->data.data(), fragment->data.size());

      // Parse the buffer and decode the compressed data
      while (buffer.unpadded_size() > 0) {
        // Parse the buffer from the current position and store the data in the packet
        const ffmpeg_cpp::Packet packet = parser_.parse(&buffer, &decoder_);

        if (packet->data) {
          // Send the packet to the decoder
          decoder_.send_packet(packet);

          // Receive and publish the decoded frames
          while (true) {
            ffmpeg_cpp::Frame frame = decoder_.receive_frame();
            if (!frame->data[0]) {
              break; // No more frames available
            }

            // If the frame data is in a hardware device,
            // transfer the data to the CPU-accessible memory before conversion
            if (frame.is_hw_frame()) {
              frame = frame.transfer_data();
            }

            // Allocate a new image message and fill properties excluding pixel data
            const auto image = std::make_shared<sensor_msgs::msg::Image>();
            image->header.stamp = fragment->header.stamp;
            image->height = frame->height;
            image->width = frame->width;

            // Fill the pixel data, maybe converting it to a ROS-compatible format
            if (const auto ros_encoding = ffmpeg_cpp::to_ros_image_encoding(frame.format_name());
                !ros_encoding.empty()) {
              // If the frame format is supported in ROS, just copy the pixel data
              image->encoding = ros_encoding;
              image->step = frame->linesize[0];
              const std::size_t data_size = frame->linesize[0] * frame->height;
              image->data.resize(data_size);
              std::memcpy(image->data.data(), frame->data[0], data_size);
            } else {
              // If the frame format is not supported in ROS, convert it to BGR
              static const std::string dst_encoding = sensor_msgs::image_encodings::BGR8;
              static const std::string dst_format_name =
                  ffmpeg_cpp::to_ffmpeg_format_name(dst_encoding);

              // Configure the image converter for this frame if needed
              if (!converter_.is_supported(frame->width, frame->height, frame.format_name(),
                                           dst_format_name)) {
                converter_.reconfigure(frame->width, frame->height, frame.format_name(),
                                       dst_format_name);
                RCLCPP_INFO(
                    node_->get_logger(), "Initialized converter (src: %s, dst: %s, size: %zdx%zd)",
                    converter_.src_format_name().c_str(), converter_.dst_format_name().c_str(),
                    converter_.width(), converter_.height());
              }

              // Make the destination image
              // by copying the frame properties and converting the pixel data
              image->encoding = dst_encoding;
              converter_.convert(frame, &image->data);
              image->step = image->data.size() / image->height;
            }

            // Invoke the callback with the converted image
            image_cb(image);
          }
        }
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", error.what());
    }
  }

private:
  rclcpp::Node *node_;
  ffmpeg_cpp::Parser parser_;
  ffmpeg_cpp::Decoder decoder_;
  ffmpeg_cpp::Converter converter_;
};

} // namespace ffmpeg_image_transport

#endif