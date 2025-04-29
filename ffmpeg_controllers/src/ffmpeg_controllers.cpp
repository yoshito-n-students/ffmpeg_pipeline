#include <controller_interface/chainable_controller_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_controllers/audio_converter_filter.hpp>
#include <ffmpeg_controllers/audio_fifo_filter.hpp>
#include <ffmpeg_controllers/compressed_image_broadcaster.hpp>
#include <ffmpeg_controllers/decoder_filter.hpp>
#include <ffmpeg_controllers/encoder_filter.hpp>
#include <ffmpeg_controllers/frame_writer.hpp>
#include <ffmpeg_controllers/image_broadcaster.hpp>
#include <ffmpeg_controllers/packet_broadcaster.hpp>
#include <ffmpeg_controllers/packet_receiver.hpp>
#include <ffmpeg_controllers/packet_writer.hpp>
#include <ffmpeg_controllers/video_converter_filter.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::AudioConverterFilter,
                       controller_interface::ChainableControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::AudioFifoFilter,
                       controller_interface::ChainableControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::CompressedImageBroadcaster,
                       controller_interface::ControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::DecoderFilter,
                       controller_interface::ChainableControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::EncoderFilter,
                       controller_interface::ChainableControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::FrameWriter, controller_interface::ControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::ImageBroadcaster,
                       controller_interface::ControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::PacketBroadcaster,
                       controller_interface::ControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::PacketReceiver,
                       controller_interface::ChainableControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::PacketWriter, controller_interface::ControllerInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_controllers::VideoConverterFilter,
                       controller_interface::ChainableControllerInterface);