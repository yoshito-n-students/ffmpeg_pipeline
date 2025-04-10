#include <ffmpeg_image_transport/video_packet_subscriber.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ffmpeg_image_transport::VideoPacketSubscriber,
                       image_transport::SubscriberPlugin)