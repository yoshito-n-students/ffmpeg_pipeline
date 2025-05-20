#ifndef FFMPEG_IMAGE_TRANSPORT_COMPRESSED_IMAGE_SUBSCRIBER_HPP
#define FFMPEG_IMAGE_TRANSPORT_COMPRESSED_IMAGE_SUBSCRIBER_HPP

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

class CompressedImageSubscriber
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
    decoder_name_ = node_->declare_parameter<std::string>("decoder_name", "");
    hw_type_name_ = node_->declare_parameter<std::string>("hw_type_name", "auto");
  }

  void internalCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &fragment,
                        const Callback &image_cb) override {
    try {
      // Configure the parser if needed.
      // The decoder that the parser is configured for is determined
      // by the decoder_name parameter if given, otherwise by the fragment format.
      if (!parser_) {
        parser_ =
            ffmpeg_cpp::Parser::create(!decoder_name_.empty() ? decoder_name_ : fragment->format);
        RCLCPP_INFO(node_->get_logger(), "Configured parser (%s)",
                    parser_.codec_names().front().c_str());
      }

      // Copy the data fragment to the reference-counted buffer with padding
      const ffmpeg_cpp::Packet buffer =
          ffmpeg_cpp::Packet::create(fragment->data.data(), fragment->data.size());

      // Parse the buffer and decode the compressed data
      std::int64_t pos = 0;
      while (pos < buffer->size) {
        // Parse the buffer and get the packet.
        // Additionally accumulate the codec parameters if the decoder is not configured yet.
        ffmpeg_cpp::Packet packet = ffmpeg_cpp::Packet::null();
        ffmpeg_cpp::CodecParameters params = ffmpeg_cpp::CodecParameters::null();
        if (!decoder_) {
          std::tie(packet, params) = parser_.parse_initial_packet(buffer, &pos);
        } else {
          packet = parser_.parse_next_packet(buffer, &pos);
        }
        if (packet.empty()) {
          continue;
        }

        // Configure the decoder for this fragment if needed
        if (!decoder_) {
          // TODO: get options from the node parameter
          decoder_ = ffmpeg_cpp::Decoder::create(
              decoder_name_ /* if empty, params->codec_id is used instead. */, params,
              hw_type_name_);
          if (const std::string hw_type_name = decoder_.hw_type_name(); hw_type_name.empty()) {
            RCLCPP_INFO(node_->get_logger(), "Configured decoder (%s)", decoder_->codec->name);
          } else {
            RCLCPP_INFO(node_->get_logger(), "Configured decoder (%s|%s)", decoder_->codec->name,
                        hw_type_name.c_str());
          }
        }

        // Send the packet to the decoder
        decoder_.send_packet(packet);

        // Receive and publish the decoded frames
        while (true) {
          ffmpeg_cpp::Frame frame = decoder_.receive_frame();
          if (frame.empty()) {
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
            if (!converter_) {
              converter_ = ffmpeg_cpp::VideoConverter::create(frame->width, frame->height,
                                                              frame.format_name(), dst_format_name);
              RCLCPP_INFO(node_->get_logger(), "Configured converter ([%s] %dx%d -> [%s])",
                          converter_.src_format_name().c_str(), converter_.src_width(),
                          converter_.src_height(), converter_.dst_format_name().c_str());
            }

            // Make the destination image
            // by copying the frame properties and converting the pixel data
            image->encoding = dst_encoding;
            image->data = converter_.convert_to_vector(frame);
            image->step = image->data.size() / image->height;
          }

          // Invoke the callback with the converted image
          image_cb(image);
        }
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", error.what());
    }
  }

private:
  rclcpp::Node *node_;
  std::string decoder_name_, hw_type_name_;
  ffmpeg_cpp::Parser parser_ = ffmpeg_cpp::Parser::null();
  ffmpeg_cpp::Decoder decoder_ = ffmpeg_cpp::Decoder::null();
  ffmpeg_cpp::VideoConverter converter_ = ffmpeg_cpp::VideoConverter::null();
};

} // namespace ffmpeg_image_transport

#endif