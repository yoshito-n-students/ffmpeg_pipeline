#ifndef FFMPEG_IMAGE_TRANSPORT_FFMPEG_SUBSCRIBER_HPP
#define FFMPEG_IMAGE_TRANSPORT_FFMPEG_SUBSCRIBER_HPP

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

class FFmpegSubscriber
    : public image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage> {
private:
  using Base = image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage>;

public:
  std::string getTransportName() const override { return "ffmpeg"; }

protected:
  void subscribeImpl(rclcpp::Node *node, const std::string &base_topic, const Callback &callback,
                     rmw_qos_profile_t custom_qos, rclcpp::SubscriptionOptions options) override {
    Base::subscribeImpl(node, base_topic, callback, custom_qos, options);
    node_ = node;
  }

  void internalCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &fragment,
                        const Callback &image_cb) override {
    try {
      // Configure the parser and decoder for this fragment if needed
      if (!parser_.is_supported(fragment->format)) {
        parser_.reconfigure(fragment->format);
        RCLCPP_INFO(node_->get_logger(), "Configured parser (codec: %s)",
                    parser_.codec_names().front().c_str());
      }
      if (!decoder_.is_supported(fragment->format)) {
        decoder_.reconfigure(fragment->format);
        RCLCPP_INFO(node_->get_logger(), "Configured decoder (codec: %s, hw: %s)",
                    decoder_.codec_name().c_str(), decoder_.hw_device_type().c_str());
      }

      // Copy the data fragment to the reference-counted buffer with padding
      const ffmpeg_cpp::BufferRef buffer(fragment->data.data(), fragment->data.size());

      // Parse the buffer and decode the compressed data
      for (std::size_t pos = 0; pos < buffer.unpadded_size();) {
        // Parse the buffer from the current position and store the data in the packet
        ffmpeg_cpp::Packet packet;
        pos += parser_.parse(buffer, &decoder_, &packet, pos);

        if (packet->data) {
          // Send the packet to the decoder
          decoder_.send_packet(packet);

          // Receive and publish the decoded frames
          ffmpeg_cpp::Frame frame;
          while (decoder_.receive_frame(&frame)) {
            // If the frame data is in a hardware device,
            // transfer the data to the CPU-accessible memory before conversion
            if (frame.is_hw_frame()) {
              ffmpeg_cpp::Frame sw_frame;
              frame.transfer_data(&sw_frame);
              frame = std::move(sw_frame);
            }

            // Allocate a new image message and fill properties excluding pixel data
            const auto image = std::make_shared<sensor_msgs::msg::Image>();
            image->header.stamp = fragment->header.stamp;
            image->height = frame->height;
            image->width = frame->width;

            // Fill the pixel data, maybe converting it to a ROS-compatible format
            if (const auto ros_encoding = to_ROS_encoding(frame->format); !ros_encoding.empty()) {
              // If the frame format is supported in ROS, just copy the pixel data
              image->encoding = ros_encoding;
              image->step = frame->linesize[0];
              const std::size_t data_size = frame->linesize[0] * frame->height;
              image->data.resize(data_size);
              std::memcpy(image->data.data(), frame->data[0], data_size);
            } else {
              // If the frame format is not supported in ROS, convert it to BGR

              // Configure the image converter for this frame if needed
              if (!converter_.is_supported(frame->width, frame->height, frame.format_name(),
                                           "bgr24")) {
                converter_.reconfigure(frame->width, frame->height, frame.format_name(), "bgr24");
                RCLCPP_INFO(
                    node_->get_logger(), "Initialized converter (src: %s, dst: %s, size: %zdx%zd)",
                    converter_.src_format_name().c_str(), converter_.dst_format_name().c_str(),
                    converter_.width(), converter_.height());
              }

              // Make the destination image
              // by copying the frame properties and converting the pixel data
              image->encoding = sensor_msgs::image_encodings::BGR8;
              image->step = 3 * frame->width;
              converter_.convert(frame, &image->data);
            }

            // Invoke the callback with the converted image
            image_cb(image);
          }
        }
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(node_->get_logger(), "Error: %s", error.what());
    }
  }

  static std::string to_ROS_encoding(const int ffmpeg_format) {
    switch (ffmpeg_format) {
    // RGB formats
    case AV_PIX_FMT_RGB24:
      return sensor_msgs::image_encodings::RGB8;
    case AV_PIX_FMT_RGBA:
      return sensor_msgs::image_encodings::RGBA8;
    case AV_PIX_FMT_RGB48:
      return sensor_msgs::image_encodings::RGB16;
    case AV_PIX_FMT_RGBA64:
      return sensor_msgs::image_encodings::RGBA16;
    // BGR formats
    case AV_PIX_FMT_BGR24:
      return sensor_msgs::image_encodings::BGR8;
    case AV_PIX_FMT_BGRA:
      return sensor_msgs::image_encodings::BGRA8;
    case AV_PIX_FMT_BGR48:
      return sensor_msgs::image_encodings::BGR16;
    case AV_PIX_FMT_BGRA64:
      return sensor_msgs::image_encodings::BGRA16;
    // Grayscale formats
    case AV_PIX_FMT_GRAY8:
      return sensor_msgs::image_encodings::MONO8;
    case AV_PIX_FMT_GRAY16:
      return sensor_msgs::image_encodings::MONO16;
      // bayer formats
    case AV_PIX_FMT_BAYER_RGGB8:
      return sensor_msgs::image_encodings::BAYER_RGGB8;
    case AV_PIX_FMT_BAYER_RGGB16:
      return sensor_msgs::image_encodings::BAYER_RGGB16;
    case AV_PIX_FMT_BAYER_BGGR8:
      return sensor_msgs::image_encodings::BAYER_BGGR8;
    case AV_PIX_FMT_BAYER_BGGR16:
      return sensor_msgs::image_encodings::BAYER_BGGR16;
    case AV_PIX_FMT_BAYER_GRBG8:
      return sensor_msgs::image_encodings::BAYER_GRBG8;
    case AV_PIX_FMT_BAYER_GRBG16:
      return sensor_msgs::image_encodings::BAYER_GRBG16;
    case AV_PIX_FMT_BAYER_GBRG8:
      return sensor_msgs::image_encodings::BAYER_GBRG8;
    case AV_PIX_FMT_BAYER_GBRG16:
      return sensor_msgs::image_encodings::BAYER_GBRG16;
    // YUV formats
    case AV_PIX_FMT_UYVY422:
      return sensor_msgs::image_encodings::UYVY;
    case AV_PIX_FMT_YUYV422:
      return sensor_msgs::image_encodings::YUYV;
    case AV_PIX_FMT_NV21:
      return sensor_msgs::image_encodings::NV21;
    case AV_PIX_FMT_NV24:
      return sensor_msgs::image_encodings::NV24;
    default:
      return "";
    }
  }

private:
  rclcpp::Node *node_;
  ffmpeg_cpp::Parser parser_;
  ffmpeg_cpp::Decoder decoder_;
  ffmpeg_cpp::Converter converter_;
};

} // namespace ffmpeg_image_transport

#endif