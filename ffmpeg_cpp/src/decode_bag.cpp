#include <memory>
#include <stdexcept>
#include <string>

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace av = ffmpeg_cpp;

// Utility function to deserialize a message from a bag
template <class Msg>
std::unique_ptr<Msg> deserialize(rclcpp::Serialization<Msg> *const deserializer,
                                 const rosbag2_storage::SerializedBagMessage &bag_msg) {
  const rclcpp::SerializedMessage serialized_msg(*bag_msg.serialized_data);
  auto deserialized_msg = std::make_unique<Msg>();
  deserializer->deserialize_message(&serialized_msg, deserialized_msg.get());
  return deserialized_msg;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "Usage: %s <bag_file>\n", argv[0]);
    return -1;
  }
  const std::string bag_filename = argv[1];

  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared("bag_decoder");
  const auto publisher = node->create_publisher<sensor_msgs::msg::Image>("dst_image", 10);

  // Initialize the bag reader with the specified file
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_filename;
  const auto reader = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
  reader->open(storage_options);

  // Initialize the deserializer for messages in the bag
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;

  // The parser, which extracts the compressed packets from the stream,
  // the decoder, which decompresses the packets to frames,
  // and the converter, which converts the frames to BGR24 images.
  // They must be configured with the codec name or pixel format name before use.
  av::Parser parser;
  av::Decoder decoder;
  av::Converter converter;

  // Decode and convert each message in the bag
  const rclcpp::Time start_time = node->now();
  std::size_t n_decoded = 0, n_error = 0;
  while (reader->has_next() && rclcpp::ok()) {
    const auto bag_msg = reader->read_next();
    if (bag_msg->topic_name != "/image/ffmpeg") {
      continue;
    }

    const auto comp_img = deserialize(&serialization, *bag_msg);

    try {
      // Initialize the parser and decoder if not already done
      if (!parser.is_supported(comp_img->format)) {
        parser = av::Parser(comp_img->format);
        RCLCPP_INFO(node->get_logger(), "Configured parser (codec: %s)",
                    parser.codec_names().front().c_str());
      }
      if (!decoder.is_supported(comp_img->format)) {
        decoder = av::Decoder(comp_img->format);
        RCLCPP_INFO(node->get_logger(), "Configured decoder (codec: %s, hw: %s)",
                    decoder.codec_name().c_str(), decoder.hw_device_type().c_str());
      }

      // Copy the compressed data to the reference-counted buffer with padding
      av::BufferRef buffer(comp_img->data.data(), comp_img->data.size());

      // Parse the buffer and decode the compressed data
      while (buffer.unpadded_size() > 0) {
        // Parse the buffer and store the data in the packet
        const av::Packet packet = parser.parse(&buffer, &decoder);

        if (packet->data) {
          // Send the packet to the decoder
          decoder.send_packet(packet);
          RCLCPP_INFO(node->get_logger(), "Sent packet to the decoder: %d bytes", packet->size);

          // Receive and publish the decoded frames
          while (true) {
            av::Frame frame = decoder.receive_frame();
            if (!frame->data[0]) {
              break; // No more frames available
            }

            RCLCPP_INFO(node->get_logger(), "Decoded frame: %dx%d", frame->width, frame->height);
            // Copy the frame properties to the destination image
            auto image = std::make_unique<sensor_msgs::msg::Image>();
            image->header.stamp = comp_img->header.stamp;
            image->height = frame->height;
            image->width = frame->width;
            image->encoding = sensor_msgs::image_encodings::BGR8;
            image->step = 3 * frame->width;

            // If the frame data is in a hardware device,
            // transfer the data to the CPU-accessible memory before conversion
            if (frame.is_hw_frame()) {
              frame = frame.transfer_data();
            }

            // Initialize the image converter if not already done
            if (!converter.is_supported(frame->width, frame->height, frame.format_name(),
                                        "bgr24")) {
              converter.reconfigure(frame->width, frame->height, frame.format_name(), "bgr24");
              RCLCPP_INFO(node->get_logger(),
                          "Configured converter (size: %zdx%zd, src: %s, dst: %s)",
                          converter.width(), converter.height(),
                          converter.src_format_name().c_str(), converter.dst_format_name().c_str());
            }

            // Convert the frame to BGR24 format
            converter.convert(frame, &image->data);

            // Publish the destination image
            publisher->publish(std::move(image));
            ++n_decoded;
          }
        }
      }
    } catch (const std::runtime_error &error) {
      ++n_error;
      RCLCPP_ERROR(node->get_logger(), "Error while decoding: %s, will try the next message",
                   error.what());
      continue;
    }
  }

  // Print the decoding statistics
  const rclcpp::Duration elapsed_time = node->now() - start_time;
  RCLCPP_INFO(node->get_logger(), "Decoded %zu frames, %zu errors, in %f seconds (%f fps)",
              n_decoded, n_error, elapsed_time.seconds(),
              (n_decoded + n_error) / elapsed_time.seconds());

  rclcpp::shutdown();

  return 0;
}
