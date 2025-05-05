#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
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
  const std::string
      url = node->declare_parameter("url", "/dev/video0"),
      options_str = node->declare_parameter(
          "options", "{input_format: h264, video_size: 1920x1080, framerate: 30, timestamps: abs}");

  // Setup the destination packet publisher
  const auto publisher =
      node->create_publisher<sensor_msgs::msg::CompressedImage>("image/ffmpeg", 10);

  try {
    // Open the input device
    av::Dictionary options = av::Dictionary::create(options_str);
    av::Input input = av::Input::create(url, "v4l2", &options, "video");
    const std::string codec_name = input.codec_parameters().codec_name();

    // Continuously read frames from the input device and publish them
    while (rclcpp::ok()) {
      // Try to read a packet from the input device
      const av::Packet packet = input.read_frame();
      if (packet.empty()) {
        std::this_thread::sleep_for(25ms);
        continue;
      }

      // Copy the packet data to a ROS 2 message
      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      // packet->pts is timestamp in kernel time, in microseconds
      msg->header.stamp.sec = packet->pts / 1'000'000;
      msg->header.stamp.nanosec = (packet->pts % 1'000'000) * 1'000;
      msg->format = codec_name;
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