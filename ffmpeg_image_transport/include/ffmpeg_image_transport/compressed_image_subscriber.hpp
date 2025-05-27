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
    // Initialize the base class
    Base::subscribeImpl(node, base_topic, callback, custom_qos, options);
    node_ = node;

    // Determine the prefix name of the parameters to avoid conflicts
    // with other plugins loaded in the same node (inspired by compressed_image_transport)
    const std::string param_prefix = [](const std::string &node_ns, const std::string &base_topic,
                                        const std::string &transport_name) {
      // node_ns: /my_group/my_node
      // base_topic: /my_group/my_node/my_topic/my_subtopic
      // transport_name: ffmpeg
      // -> prefix: my_topic.my_subtopic.ffmpeg
      std::string prefix = base_topic.substr(node_ns.length()) + '.' + transport_name;
      std::replace(prefix.begin(), prefix.end(), '/', '.');
      return prefix;
    }(node->get_effective_namespace(), base_topic, getTransportName());

    // Load the decoder parameters from the node
    decoder_name_ = node_->declare_parameter<std::string>(param_prefix + ".decoder_name", "");
    hw_type_name_ = node_->declare_parameter<std::string>(param_prefix + ".hw_type_name", "auto");
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
      const ffmpeg_cpp::Packet buffer = ffmpeg_cpp::Packet::create(*fragment);

      // Parse the buffer and decode the compressed data
      std::int64_t pos = 0;
      while (pos < buffer->size) {
        // Parse the buffer and get the packet
        const ffmpeg_cpp::Packet packet = parser_.parse(buffer, &pos);
        if (packet.empty()) {
          continue;
        }

        // Configure the decoder for this fragment if possible
        if (!decoder_) {
          if (parser_->key_frame != 0) {
            // If the last parsed packet is a keyframe,
            // we can expect the full codec parameters are available and initialize the decoder.
            decoder_ = ffmpeg_cpp::Decoder::create(
                decoder_name_ /* if empty, params->codec_id is used instead. */,
                parser_.codec_parameters(), hw_type_name_);
            if (const std::string hw_type_name = decoder_.hw_type_name(); hw_type_name.empty()) {
              RCLCPP_INFO(node_->get_logger(), "Configured decoder (%s)", decoder_->codec->name);
            } else {
              RCLCPP_INFO(node_->get_logger(), "Configured decoder (%s|%s)", decoder_->codec->name,
                          hw_type_name.c_str());
            }
          } else {
            // It is not a time to initialize the decoder. Wait for the next keyframe.
            continue;
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

          // Allocate a new image message
          const auto image = std::make_shared<sensor_msgs::msg::Image>();

          // Fill the pixel data, maybe converting it to a ROS-compatible format
          if (const auto ros_encoding = ffmpeg_cpp::to_ros_image_encoding(frame.format_name());
              !ros_encoding.empty()) {
            // If the frame format is supported in ROS, just copy the pixel data
            *image = frame.to_image_msg(fragment->header.stamp, ros_encoding);
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

            // Make the destination image by converting the pixel data
            frame = converter_.convert(frame);
            *image = frame.to_image_msg(fragment->header.stamp, dst_encoding);
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