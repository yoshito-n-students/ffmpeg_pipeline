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

class BagReader {
public:
  BagReader(const std::string &bag_filename, const std::string &topic_name)
      : topic_name_(topic_name) {
    // Initialize the bag reader with the specified file
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_filename;
    reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
    reader_->open(storage_options);
  }

  std::unique_ptr<sensor_msgs::msg::CompressedImage> read_next() {
    while (reader_->has_next()) {
      const auto bag_msg = reader_->read_next();
      if (bag_msg->topic_name == topic_name_) {
        const rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        deserializer_.deserialize_message(&serialized_msg, msg.get());
        return msg; // Return the next compressed image message
      }
    }
    return nullptr; // No more messages available
  }

private:
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::string topic_name_;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> deserializer_;
};

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

  // Initialize the bag reader with the specified file and topic
  BagReader bag_reader(bag_filename, "/image/ffmpeg");

  // The parser, which extracts the compressed packets from the stream,
  // the decoder, which decompresses the packets to frames,
  // and the converter, which converts the frames to BGR24 images.
  // They must be configured with the codec name or pixel format name before use.
  av::Parser parser = av::Parser::null();
  av::Decoder decoder = av::Decoder::null();
  av::VideoConverter converter = av::VideoConverter::null();

  // Decode and convert each message in the bag
  const rclcpp::Time start_time = node->now();
  std::size_t n_decoded = 0, n_error = 0;
  while (rclcpp::ok()) {
    // Read the next compressed image message from the bag
    const auto comp_img = bag_reader.read_next();
    if (!comp_img) {
      break; // No more messages available
    }

    try {
      // Initialize the parser with the image format if not already done
      if (!parser) {
        parser = av::Parser::create(comp_img->format);
        RCLCPP_INFO(node->get_logger(), "Configured parser (codec: %s)",
                    parser.codec_names().front().c_str());
      }

      // Copy the compressed data to the reference-counted buffer with padding
      const av::Packet buffer = av::Packet::create(comp_img->data.data(), comp_img->data.size());

      // Parse the buffer and decode the compressed data
      std::int64_t pos = 0;
      while (pos < buffer->size) {
        // Parse the buffer to extract the compressed packet.
        // Additionally, probe the codec parameters for the decoder if it has not been initialized.
        av::Packet packet = av::Packet::null();
        av::CodecParameters codec_params = av::CodecParameters::null();
        if (!decoder) {
          std::tie(packet, codec_params) = parser.parse_initial_packet(buffer, &pos);
        } else {
          packet = parser.parse_next_packet(buffer, &pos);
        }
        if (packet.empty()) {
          continue;
        }

        // Initialize the decoder with the parameters if not already done
        if (!decoder) {
          decoder = av::Decoder::create(
              "" /* empty decoder name. codec_params->codec_id is used instead. */, codec_params,
              "auto" /* auto select the hardware acceleration type */);
          RCLCPP_INFO(node->get_logger(), "Configured decoder (%s)", decoder->codec->name);
        }

        // Send the packet to the decoder
        RCLCPP_INFO(node->get_logger(), "Start decoding packet (%d bytes)", packet->size);
        decoder.send_packet(packet);

        // Receive and publish the decoded frames
        while (true) {
          av::Frame frame = decoder.receive_frame();
          if (frame.empty()) {
            break; // No more frames available
          }
          RCLCPP_INFO(node->get_logger(), "Got frame ([%s] %dx%d)", frame.format_name().c_str(),
                      frame->width, frame->height);

          // If the frame data is in a hardware device,
          // transfer the data to the CPU-accessible memory before conversion
          if (frame.is_hw_frame()) {
            frame = frame.transfer_data();
          }

          // Initialize the image converter if not already done
          if (!converter) {
            converter = av::VideoConverter::create(frame->width, frame->height, frame.format_name(),
                                                   "bgr24");
            RCLCPP_INFO(node->get_logger(), "Configured converter ([%s -> %s] %dx%d)",
                        converter.src_format_name().c_str(), converter.dst_format_name().c_str(),
                        converter.src_width(), converter.src_height());
          }

          // Convert the frame to a BGR24 image
          frame = converter.convert(frame);

          // Make the destination image message by copying the frame data
          auto image = std::make_unique<sensor_msgs::msg::Image>(
              frame.to_image_msg(comp_img->header.stamp, sensor_msgs::image_encodings::BGR8));

          // Publish the destination image
          publisher->publish(std::move(image));
          ++n_decoded;
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
