#include <memory>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace av = ffmpeg_cpp;

int main(int argc, char *argv[]) {
  // Initialize ROS 2 node
  rclcpp::init(argc, argv);
  const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("decode_topic");

  try {
    // The decoder, which decompresses H.264 video packets
    // and the converter, which converts the decoded frames to BGR24 images
    av::Decoder decoder;
    av::Converter converter;

    // Create a publisher to publish the decoded images
    const auto publisher = node->create_publisher<sensor_msgs::msg::Image>("dst_image", 10);

    // Define the callback function to decode and publish the H.264 packets
    const auto decode_and_publish =
        [&](const sensor_msgs::msg::CompressedImage::SharedPtr packet_data) {
          try {
            // Set the packet data from the message
            if (packet_data->data.empty()) {
              return;
            }
            av::Packet packet;
            packet->data = packet_data->data.data();
            packet->size = packet_data->data.size();

            // Initialize the decoder if not already done
            if (!decoder.is_supported(packet_data->format)) {
              decoder.reconfigure(packet_data->format);
              RCLCPP_INFO(node->get_logger(), "Configured decoder (codec: %s, hw: %s)",
                          decoder.codec_name().c_str(), decoder.hw_device_type().c_str());
            }

            // Send the packet to the decoder
            decoder.send_packet(packet);

            // Receive and publish the decoded frames
            av::Frame frame;
            while (decoder.receive_frame(&frame)) {
              // Copy the frame properties to the destination image
              auto image = std::make_unique<sensor_msgs::msg::Image>();
              image->header.stamp = packet_data->header.stamp;
              image->height = frame->height;
              image->width = frame->width;
              image->encoding = sensor_msgs::image_encodings::BGR8;
              image->step = 3 * frame->width;

              // Convert the frame to BGR24 image
              if (frame.is_hw_frame()) {
                // If the frame data is in a hardware device,
                // transfer the data to the CPU-accessible memory before conversion
                av::Frame sw_frame;
                frame.transfer_data(&sw_frame);
                frame = std::move(sw_frame);
              }

              // Initialize the image converter if not already done
              if (!converter.is_supported(frame->width, frame->height, frame.format_name(),
                                          "bgr24")) {
                converter.reconfigure(frame->width, frame->height, frame.format_name(), "bgr24");
                RCLCPP_INFO(
                    node->get_logger(), "Configured converter (size: %zdx%zd, src: %s, dst: %s)",
                    converter.width(), converter.height(), converter.src_format_name().c_str(),
                    converter.dst_format_name().c_str());
              }

              // Convert the frame to BGR24 format
              converter.convert(frame, &image->data);

              // Publish the destination image
              publisher->publish(std::move(image));
            }
          } catch (const std::runtime_error &err) {
            RCLCPP_ERROR(node->get_logger(), "Error: %s", err.what());
          }
        };

    const auto subscription = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/ffmpeg", 10, decode_and_publish);

    rclcpp::spin(node);
  } catch (const std::runtime_error &err) {
    RCLCPP_ERROR(node->get_logger(), "Error: %s", err.what());
  }

  rclcpp::shutdown();

  return 0;
}
