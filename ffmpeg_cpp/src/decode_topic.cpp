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
    av::VideoConverter converter;

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
            if (!decoder.valid()) {
              av::Dictionary options;
              decoder = av::Decoder(packet_data->format,&options);
              RCLCPP_INFO(node->get_logger(), "Configured decoder (codec: %s, hw: %s)",
                          decoder.codec_name().c_str(), decoder.hw_type_name().c_str());
            }

            // Send the packet to the decoder
            decoder.send_packet(packet);

            // Receive and publish the decoded frames
            while (true) {
              av::Frame frame = decoder.receive_frame();
              if (frame.empty()) {
                break; // No more frames available
              }

              // Convert the frame to BGR24 image
              if (frame.is_hw_frame()) {
                // If the frame data is in a hardware device,
                // transfer the data to the CPU-accessible memory before conversion
                frame = frame.transfer_data();
              }

              // Initialize the image converter if not already done
              if (!converter) {
                converter =
                    av::VideoConverter(frame->width, frame->height, frame.format_name(), "bgr24");
                RCLCPP_INFO(
                    node->get_logger(), "Configured converter (size: %zdx%zd, src: %s, dst: %s)",
                    converter.src_width(), converter.src_height(),
                    converter.src_format_name().c_str(), converter.dst_format_name().c_str());
              }

              // Copy the frame properties to the destination image
              auto image = std::make_unique<sensor_msgs::msg::Image>();
              image->header.stamp = packet_data->header.stamp;
              image->height = frame->height;
              image->width = frame->width;
              image->encoding = sensor_msgs::image_encodings::BGR8;
              image->step = 3 * frame->width;
              image->data = converter.convert_to_vector(frame);

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
