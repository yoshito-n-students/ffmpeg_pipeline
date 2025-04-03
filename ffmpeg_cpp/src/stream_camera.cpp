#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility> // for std::move()

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

using namespace std::chrono_literals;
namespace av = ffmpeg_cpp;

int main(int argc, char *argv[]) {
  // ROS 2 node initialization
  rclcpp::init(argc, argv);
  const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("stream_camera");

  // Parameters for the input device
  const std::string url = node->declare_parameter("url", "/dev/video0");
  const std::map<std::string, std::string> option_map = {
      {"input_format", node->declare_parameter("input_format", "h264")},
      {"video_size", node->declare_parameter("video_size", "1920x1080")},
      {"framerate", node->declare_parameter("framerate", "30")},
      {"timestamps", node->declare_parameter("timestamps", "abs")}};

  // Setup the destination packet publisher
  const auto publisher =
      node->create_publisher<sensor_msgs::msg::CompressedImage>("image/ffmpeg", 10);

  try {
    // Open the input device
    av::Input input;
    input.reconfigure(url, "v4l2", option_map);

    // Continuously read frames from the input device and publish them
    while (rclcpp::ok()) {
      // Read a packet from the input device
      av::Packet packet;
      input.read_frame(&packet, 1'000ms);

      // Copy the packet data to a ROS 2 message
      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      // packet->pts is timestamp in kernel time, in microseconds
      msg->header.stamp.sec = packet->pts / 1'000'000;
      msg->header.stamp.nanosec = (packet->pts % 1'000'000) * 1'000;
      msg->format = input.codec_name();
      msg->data.assign(packet->data, packet->data + packet->size);

      // Publish the message
      publisher->publish(std::move(msg));
    }
  } catch (const std::runtime_error &error) {
    RCLCPP_ERROR(node->get_logger(), "Error: %s", error.what());
  }

  rclcpp::shutdown();

  return 0;
}