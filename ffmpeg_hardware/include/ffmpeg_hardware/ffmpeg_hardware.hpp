#ifndef FFMPEG_HARDWARE_FFMPEG_HARDWARE_HPP
#define FFMPEG_HARDWARE_FFMPEG_HARDWARE_HPP

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility> // for std::swap()
#include <vector>

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <hardware_interface/handle.hpp> // for hi::{State,Command}Interface
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <yaml-cpp/yaml.h>

namespace ffmpeg_hardware {

class FFmpegHardware : public hardware_interface::SystemInterface {
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override {
    // Initialize the base class. info_ is copied from hardware_info,
    // and *_interfaces_ are constructed according to the contents of info_.
    if (const auto res = hardware_interface::SystemInterface::on_init(hardware_info);
        res != CallbackReturn::SUCCESS) {
      return res;
    }

    // Check if exactly two sensor state interfaces exist
    if (sensor_state_interfaces_.size() != 2) {
      RCLCPP_ERROR(get_logger(),
                   "Exactly two sensor state interfaces are required while %zd are found",
                   sensor_state_interfaces_.size());
      return CallbackReturn::ERROR;
    } else if (const auto &sensor_name = info_.sensors[0].name;
               sensor_state_interfaces_.count(sensor_name + "/codec") == 0 ||
               sensor_state_interfaces_.count(sensor_name + "/packet") == 0) {
      RCLCPP_ERROR(get_logger(),
                   "Sensor state interfaces must belong to the first sensor component '%s' "
                   "and be named 'codec' and 'packet'",
                   sensor_name.c_str());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Try to get the parameters for the input from the hardware_info, or use default values
    const auto &sensor_params = info_.sensors[0].parameters;
    const auto url = get_parameter_as<std::string>(sensor_params, "url", "/dev/video0"),
               input_format = get_parameter_as<std::string>(sensor_params, "input_format", "v4l2");
    const auto options =
        get_parameter_as<std::map<std::string, std::string>>(sensor_params, "options",
                                                             {{"input_format", "h264"},
                                                              {"video_size", "1920x1080"},
                                                              {"framerate", "30"},
                                                              {"timestamps", "abs"}});

    // Configure the input with the parameters
    try {
      input_.reconfigure(url, input_format, options);
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(get_logger(), "Failed to configure the input: %s", error.what());
      return CallbackReturn::ERROR;
    }

    // Start reading packets from the input
    stop_requested_ = false;
    write_thread_ = std::thread([this]() {
      try {
        while (!stop_requested_) {
          ffmpeg_cpp::Packet packet;
          input_.read_frame(&packet, std::chrono::milliseconds(500));
          {
            std::unique_lock<std::mutex> lock(write_packet_mutex_);
            write_packet_ = std::move(packet);
          }
        }
      } catch (const std::runtime_error &error) {
        RCLCPP_ERROR(get_logger(), "Stopped reading packets from the input because of error: %s",
                     error.what());
      }
    });

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Stop reading packets from the camera
    stop_requested_ = true;
    if (write_thread_.joinable()) {
      write_thread_.join();
    }

    // TODO: Close the input device
    
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(const rclcpp::Time & /*time*/,
                                       const rclcpp::Duration & /*period*/) override {
    // Move the write packet to the read packet if the fomer is newer than the latter.
    // However, this step will be skipped if the write side thread locks the write packet.
    // This procedure avoids blocking on the read side while obtaining the latest packet.
    if (const std::unique_lock<std::mutex> try_lock(write_packet_mutex_, std::try_to_lock);
        try_lock.owns_lock() && read_packet_->pts < write_packet_->pts) {
      std::swap(read_packet_, write_packet_);
    }

    set_state_from_pointer(info_.sensors[0].name + "/packet", &read_packet_);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time & /*time*/,
                                        const rclcpp::Duration & /*period*/) override {
    // Nothing to do
    return hardware_interface::return_type::OK;
  }

protected:
  template <typename T, class Map>
  T get_parameter_as(const Map &params, const std::string &key, T &&default_value) {
    // Try to find the parameter with the given key
    const auto found_it = params.find(key);
    if (found_it != params.end()) {
      // If the parameter is found, try to convert it to the desired type
      try {
        return YAML::Load(found_it->second).template as<T>();
      } catch (const YAML::Exception &error) {
        // If the conversion fails, log an error and return the default value
        RCLCPP_ERROR(get_logger(),
                     "Failed to convert parameter '%s' (value: '%s') to the desired type: %s",
                     found_it->first.c_str(), found_it->second.c_str(), error.what());
        return std::move(default_value);
      }
    } else {
      // If the parameter is not found, return the default value
      return std::move(default_value);
    }
  }

  template <typename T> void set_state_from_pointer(const std::string &iface_name, const T *const value) {
    // Due to the constraints of hardware_interfae::StateInterface,
    // the state can only be set as a double type value.
    // Therefore, the pointer type is converted to double type at the value level and set.
    // The double type has 53 bits of precision,
    // which is sufficient to represent the 47 bits of the Linux user's address space.
    set_state(iface_name, static_cast<double>(reinterpret_cast<std::uintptr_t>(value)));
  }

protected:
  ffmpeg_cpp::Input input_;

  std::thread write_thread_;
  std::atomic_bool stop_requested_;
  ffmpeg_cpp::Packet write_packet_, read_packet_;
  std::mutex write_packet_mutex_;
};

} // namespace ffmpeg_hardware

#endif