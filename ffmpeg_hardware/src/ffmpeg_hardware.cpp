#include <ffmpeg_hardware/dump_info_output.hpp>
#include <ffmpeg_hardware/ffmpeg_input.hpp>
#include <ffmpeg_hardware/ffmpeg_output.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ffmpeg_hardware::FFmpegInput, hardware_interface::SensorInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_hardware::FFmpegOutput, hardware_interface::ActuatorInterface);
PLUGINLIB_EXPORT_CLASS(ffmpeg_hardware::DumpInfoOutput, hardware_interface::ActuatorInterface);