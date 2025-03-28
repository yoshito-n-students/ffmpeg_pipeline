#ifndef FFMPEG_IMAGE_TRANSPORT_FFMPEG_SUBSCRIBER_HPP
#define FFMPEG_IMAGE_TRANSPORT_FFMPEG_SUBSCRIBER_HPP

#include <string>

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <image_transport/simple_subscriber_plugin.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription_options.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace ffmpeg_image_transport {

class FFmpegSubscriber
    : public image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage> {
public:
  FFmpegSubscriber() {}
  virtual ~FFmpegSubscriber() = default;

  std::string getTransportName() const override { return "ffmpeg"; }

protected:
  void subscribeImpl(rclcpp::Node *, const std::string & /*base_topic*/,
                     const Callback & /*callback*/, rmw_qos_profile_t /*custom_qos*/,
                     rclcpp::SubscriptionOptions /*options*/) override {}

  void internalCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & /*message*/,
                        const Callback & /*user_cb*/) override {}
};

} // namespace ffmpeg_image_transport

#endif