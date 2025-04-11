#include <ffmpeg_image_transport/compressed_image_subscriber.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ffmpeg_image_transport::CompressedImageSubscriber,
                       image_transport::SubscriberPlugin)