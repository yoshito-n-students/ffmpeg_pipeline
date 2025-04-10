#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_controllers/video_decoder_broadcaster.hpp>
#include <ffmpeg_controllers/video_packet_broadcaster.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::VideoDecoderBroadcaster,
                       controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::VideoPacketBroadcaster,
                       controller_interface::ControllerInterface)