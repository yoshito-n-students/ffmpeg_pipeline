#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_controllers/compressed_image_broadcaster.hpp>
#include <ffmpeg_controllers/frame_broadcaster.hpp>
#include <ffmpeg_controllers/image_broadcaster.hpp>
#include <ffmpeg_controllers/packet_broadcaster.hpp>
#include <ffmpeg_controllers/packet_controller.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::CompressedImageBroadcaster,
                       controller_interface::ControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::FrameBroadcaster,
                       controller_interface::ControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::ImageBroadcaster,
                       controller_interface::ControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::PacketBroadcaster,
                       controller_interface::ControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::PacketController,
                       controller_interface::ControllerInterface);