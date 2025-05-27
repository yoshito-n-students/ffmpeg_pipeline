#include <memory>
#include <sstream>
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

struct Stats {
  const rclcpp::Time start_time;
  std::size_t n_msgs = 0, n_packets = 0, n_images = 0;

  std::string to_string(const rclcpp::Time &current_time) const {
    const double elapsed = (current_time - start_time).seconds();
    std::ostringstream str;
    str << n_msgs << " messages read, " << n_packets << " packets parsed, " << n_images
        << " images decoded, in " << elapsed << " s (" << n_images / elapsed << " fps)";
    return str.str();
  }
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

  try {
    // Decode and convert each message in the bag
    Stats stats{node->now()};
    while (rclcpp::ok()) {
      // Read the next compressed image message from the bag
      const auto comp_img = bag_reader.read_next();
      if (!comp_img) {
        break; // No more messages available
      }
      ++stats.n_msgs;

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
        const av::Packet packet = parser.parse(buffer, &pos);
        if (packet.empty()) {
          continue;
        }
        ++stats.n_packets;

        // Initialize the decoder with the parsed codec parameters if not already done and possible
        if (!decoder) {
          if (parser->key_frame != 0) {
            // If the last parsed packet is a keyframe,
            // we can expect the full codec parameters are available and initialize the decoder.
            decoder = av::Decoder::create(
                "" /* empty decoder name. codec_params->codec_id is used instead. */,
                parser.codec_parameters(), "auto" /* auto select the hardware acceleration type */);
            if (const std::string hw_type_name = decoder.hw_type_name(); hw_type_name.empty()) {
              RCLCPP_INFO(node->get_logger(), "Configured decoder (%s)", decoder->codec->name);
            } else {
              RCLCPP_INFO(node->get_logger(), "Configured decoder (%s|%s)", decoder->codec->name,
                          hw_type_name.c_str());
            }
          } else {
            // Wait for the next keyframe to initialize the decoder.
            continue;
          }
        }

        // Send the packet to the decoder
        decoder.send_packet(packet);

        // Receive and publish the decoded frames
        while (true) {
          av::Frame frame = decoder.receive_frame();
          if (frame.empty()) {
            break; // No more frames available
          }

          // If the frame data is in a hardware device,
          // transfer the data to the CPU-accessible memory before conversion
          if (frame.is_hw_frame()) {
            frame = frame.transfer_data();
          }

          // Initialize the image converter if not already done
          if (!converter) {
            converter = av::VideoConverter::create(frame->width, frame->height, frame.format_name(),
                                                   "bgr24");
            RCLCPP_INFO(node->get_logger(), "Configured converter ([%s] %dx%d -> [%s])",
                        converter.src_format_name().c_str(), converter.src_width(),
                        converter.src_height(), converter.dst_format_name().c_str());
          }

          // Convert the frame to a BGR24 image
          frame = converter.convert(frame);

          // Make the destination image message by copying the frame data
          auto image = std::make_unique<sensor_msgs::msg::Image>(
              frame.to_image_msg(comp_img->header.stamp, sensor_msgs::image_encodings::BGR8));

          // Publish the destination image
          publisher->publish(std::move(image));
          ++stats.n_images;

          // Print intermediate stats
          if (stats.n_msgs % 50 == 0) {
            RCLCPP_INFO(node->get_logger(), "%s", stats.to_string(node->now()).c_str());
          }
        }
      }
    }

    // Print the final stats
    RCLCPP_INFO(node->get_logger(), "%s", stats.to_string(node->now()).c_str());
    RCLCPP_INFO(node->get_logger(), "Processed all messages!");
  } catch (const std::runtime_error &error) {
    RCLCPP_ERROR(node->get_logger(), "Error while processing messages: %s", error.what());
  }

  rclcpp::shutdown();

  return 0;
}
