#ifndef FFMPEG_IMAGE_TRANSPORT_COMPRESSED_IMAGE_PUBLISHER_HPP
#define FFMPEG_IMAGE_TRANSPORT_COMPRESSED_IMAGE_PUBLISHER_HPP

#include <memory>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <image_transport/simple_publisher_plugin.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ffmpeg_image_transport {

class CompressedImagePublisher
    : public image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> {
private:
  using Base = image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage>;

public:
  std::string getTransportName() const override { return "ffmpeg"; }

protected:
  void advertiseImpl(rclcpp::Node *node, const std::string &base_topic,
                     rmw_qos_profile_t custom_qos, rclcpp::PublisherOptions options) override {
    // Initialize the base class
    Base::advertiseImpl(node, base_topic, custom_qos, options);
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

    // Load the encoder parameters from the node
    const std::string encoder_name =
                          node_->declare_parameter<std::string>(param_prefix + ".encoder_name", ""),
                      codec_params_yaml = node_->declare_parameter<std::string>(
                          param_prefix + ".codec_paramaters", ""),
                      hw_type_name = node_->declare_parameter<std::string>(
                          param_prefix + ".hw_type_name", "auto"),
                      encoder_options_yaml = node_->declare_parameter<std::string>(
                          param_prefix + ".encoder_options", "");

    // Configure the encoder
    encoder_ = ffmpeg_cpp::Encoder::create(
        encoder_name /* if empty, codec_params->codec_id will be used to find the encoder */,
        !codec_params_yaml.empty() ? ffmpeg_cpp::CodecParameters::create(codec_params_yaml)
                                   : ffmpeg_cpp::CodecParameters::null(),
        hw_type_name,
        !encoder_options_yaml.empty() ? ffmpeg_cpp::Dictionary::create(encoder_options_yaml)
                                      : ffmpeg_cpp::Dictionary::null());
  }

  void publish(const sensor_msgs::msg::Image &image, const PublisherT &publisher) const override {
    try {
      // Start encoding the image
      encoder_.send_frame(ffmpeg_cpp::Frame::create(image));

      // Publish the encoded packets
      while (true) {
        // Receive a packet from the encoder
        ffmpeg_cpp::Packet packet = encoder_.receive_packet();
        if (packet.empty()) {
          break; // No more packets available
        }

        // Convert the packet to a message.
        // The format field of the message is set to the codec name used for encoding,
        // as a hint for the subscriber to decode the message.
        auto comp_image = std::make_unique<sensor_msgs::msg::CompressedImage>(
            packet.to_compressed_image_msg(image.header.stamp, encoder_.codec_name()));

        // Give the ornership of the message to the publisher and let it publish the message.
        publisher->publish(std::move(comp_image));
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", error.what());
    }
  }

  void publish(sensor_msgs::msg::Image::UniquePtr image,
               const PublisherT &publisher) const override {
    publish(*image, publisher);
  }

private:
  rclcpp::Node *node_;
  mutable ffmpeg_cpp::Encoder encoder_ = ffmpeg_cpp::Encoder::null();
};

} // namespace ffmpeg_image_transport

#endif