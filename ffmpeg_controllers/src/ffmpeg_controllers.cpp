#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_controllers/ffmpeg_packet_broadcaster.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::FFmpegPacketBroadcaster,
                       controller_interface::ControllerInterface)