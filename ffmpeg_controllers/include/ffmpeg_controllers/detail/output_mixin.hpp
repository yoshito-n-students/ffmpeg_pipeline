#ifndef FFMPEG_CONTROLLERS_DETAIL_OUTPUT_MIXIN_HPP
#define FFMPEG_CONTROLLERS_DETAIL_OUTPUT_MIXIN_HPP

#include <stdexcept>
#include <string>
#include <tuple>

#include <controller_interface/controller_interface_base.hpp> // for controller_interface::return_type
#include <ffmpeg_controllers/controller_options.hpp>
#include <ffmpeg_controllers/detail/controller_traits.hpp>
#include <ffmpeg_controllers/detail/interface_adapter.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace ffmpeg_controllers {

// ========================================================================
// Helper class to define the pure virtual function OutputMixin::on_write()
// which is used to write the input
// ========================================================================

template <typename OutputOption> class OnWriteContract {
protected:
  // Default version
  virtual controller_interface::return_type on_write(const rclcpp::Time &time,
                                                     const rclcpp::Duration &period,
                                                     OutputFor<OutputOption> &&output) = 0;
};
template <typename... OutputOptions> class OnWriteContract<std::tuple<OutputOptions...>> {
protected:
  // Tuple version
  virtual controller_interface::return_type on_write(const rclcpp::Time &time,
                                                     const rclcpp::Duration &period,
                                                     OutputFor<OutputOptions> &&...outputs) = 0;
};

// =====================================
// Mixin class depending on OutputOption
// =====================================

template <typename OutputOption, class Interface> class OutputMixin;

template <class Interface>
class OutputMixin<output_options::Export<ffmpeg_cpp::Frame>, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnWriteContract<output_options::Export<ffmpeg_cpp::Frame>> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  typename Base::NodeReturn on_init() override {
    // Names of intraprocess read-only variables to be exported
    Base::exported_state_interface_names_ = {"frame"};
    return Base::NodeReturn::SUCCESS;
  }

  typename Base::NodeReturn
  on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Unregister the frame from state interface owned by this controller
    return Base::set_state_from_pointer("frame", nullptr) ? Base::NodeReturn::SUCCESS
                                                          : Base::NodeReturn::ERROR;
  }

  typename Base::ControllerReturn on_write(const rclcpp::Time & /*time*/,
                                           const rclcpp::Duration & /*period*/,
                                           ffmpeg_cpp::Frame &&input_frame) override {
    // Export the given frame to the state interface owned by this controller
    output_frame_ = std::move(input_frame);
    return Base::set_state_from_pointer("frame", &output_frame_) ? Base::ControllerReturn::OK
                                                                 : Base::ControllerReturn::ERROR;
  }

private:
  ffmpeg_cpp::Frame output_frame_;
};

template <class Interface>
class OutputMixin<output_options::Export<ffmpeg_cpp::Packet>, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnWriteContract<output_options::Export<ffmpeg_cpp::Packet>> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  typename Base::NodeReturn on_init() override {
    // Names of intraprocess read-only variables to be exported
    Base::exported_state_interface_names_ = {"packet"};
    return Base::NodeReturn::SUCCESS;
  }

  typename Base::NodeReturn
  on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Unregister the frame from state interface owned by this controller
    return Base::set_state_from_pointer("packet", nullptr) ? Base::NodeReturn::SUCCESS
                                                           : Base::NodeReturn::ERROR;
  }

  typename Base::ControllerReturn on_write(const rclcpp::Time & /*time*/,
                                           const rclcpp::Duration & /*period*/,
                                           ffmpeg_cpp::Packet &&input_packet) override {
    // Export the given codec parameters and packet to the state interfaces owned by this controller
    output_packet_ = std::move(input_packet);
    return Base::set_state_from_pointer("packet", &output_packet_) ? Base::ControllerReturn::OK
                                                                   : Base::ControllerReturn::ERROR;
  }

private:
  ffmpeg_cpp::Packet output_packet_;
};

template <class Interface>
class OutputMixin<output_options::Export<ffmpeg_cpp::CodecParameters>, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnWriteContract<output_options::Export<ffmpeg_cpp::CodecParameters>> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  typename Base::NodeReturn on_init() override {
    // Names of intraprocess read-only variables to be exported
    Base::exported_state_interface_names_ = {"codec_parameters"};
    return Base::NodeReturn::SUCCESS;
  }

  typename Base::NodeReturn
  on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Unregister the frame from state interface owned by this controller
    return (Base::set_state_from_pointer("codec_parameters", nullptr)) ? Base::NodeReturn::SUCCESS
                                                                       : Base::NodeReturn::ERROR;
  }

  typename Base::ControllerReturn on_write(const rclcpp::Time & /*time*/,
                                           const rclcpp::Duration & /*period*/,
                                           ffmpeg_cpp::CodecParameters &&codec_params) override {
    // Export the given codec parameters and packet to the state interfaces owned by this controller
    codec_params_ = std::move(codec_params);
    return Base::set_state_from_pointer("codec_parameters", &codec_params_)
               ? Base::ControllerReturn::OK
               : Base::ControllerReturn::ERROR;
  }

private:
  ffmpeg_cpp::CodecParameters codec_params_;
};

template <class Interface>
class OutputMixin<output_options::Write<ffmpeg_cpp::Packet>, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnWriteContract<output_options::Write<ffmpeg_cpp::Packet>> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller to which the command interface is written
      output_name_ = Base::template declare_or_get_parameter<std::string>("output_name");
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  typename Base::ControllerReturn on_write(const rclcpp::Time & /*time*/,
                                           const rclcpp::Duration & /*period*/,
                                           ffmpeg_cpp::Packet &&input_packet) override {
    if (const auto output_packet =
            Base::template get_command_as_pointer<ffmpeg_cpp::Packet>(output_name_, "packet");
        output_packet) {
      // Update the command variable with the given packet
      *output_packet = std::move(input_packet);
      return Base::ControllerReturn::OK;
    } else {
      // It is still OK if the command variable is not available
      RCLCPP_WARN(Base::get_logger(), "Failed to get output packet. Will skip this update.");
      return Base::ControllerReturn::OK;
    }
  }

private:
  std::string output_name_;
};

template <typename Message, class Interface>
class OutputMixin<output_options::Publish<Message>, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnWriteContract<output_options::Publish<Message>> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  using OutputMessage = Message;

  typename Base::NodeReturn on_init() override {
    output_topic_ = "~/output";
    return Base::NodeReturn::SUCCESS;
  }

  typename Base::NodeReturn
  on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      underlying_publisher_ = Base::get_node()->template create_publisher<OutputMessage>(
          output_topic_, rclcpp::SystemDefaultsQoS());
      async_publisher_ =
          std::make_unique<realtime_tools::RealtimePublisher<OutputMessage>>(underlying_publisher_);
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while creating publishers: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  typename Base::ControllerReturn on_write(const rclcpp::Time & /*time*/,
                                           const rclcpp::Duration & /*period*/,
                                           OutputMessage &&input_msg) override {
    // Publish a message if the message is generated and the publisher can be locked
    if (async_publisher_->trylock()) {
      async_publisher_->msg_ = std::move(input_msg);
      async_publisher_->unlockAndPublish();
    }
    return Base::ControllerReturn::OK;
  }

protected:
  std::string output_topic_;

private:
  typename rclcpp::Publisher<OutputMessage>::SharedPtr underlying_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<OutputMessage>> async_publisher_;
};

template <typename... OutputOptions, class Interface>
class OutputMixin<std::tuple<OutputOptions...>, Interface>
    : public OutputMixin<OutputOptions, Interface>...,
      public OnWriteContract<std::tuple<OutputOptions...>> {
private:
  using BaseCommon = InterfaceAdapter<Interface>;
  template <typename OutputOption> using BaseOutput = OutputMixin<OutputOption, Interface>;

protected:
  typename BaseCommon::NodeReturn on_init() override {
    const std::array<typename BaseCommon::NodeReturn, sizeof...(OutputOptions)> results = {
        BaseOutput<OutputOptions>::on_init()...};
    return std::all_of(results.begin(), results.end(),
                       [](const auto result) { return result == BaseCommon::NodeReturn::SUCCESS; })
               ? BaseCommon::NodeReturn::SUCCESS
               : BaseCommon::NodeReturn::ERROR;
  }

  typename BaseCommon::NodeReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
    const std::array<typename BaseCommon::NodeReturn, sizeof...(OutputOptions)> results = {
        BaseOutput<OutputOptions>::on_deactivate(previous_state)...};
    return std::all_of(results.begin(), results.end(),
                       [](const auto result) { return result == BaseCommon::NodeReturn::SUCCESS; })
               ? BaseCommon::NodeReturn::SUCCESS
               : BaseCommon::NodeReturn::ERROR;
  }

  typename BaseCommon::ControllerReturn on_write(const rclcpp::Time &time,
                                                 const rclcpp::Duration &period,
                                                 OutputFor<OutputOptions> &&...outputs) override {
    // Call on_read() for each input option and collect the results
    const std::array<typename BaseCommon::ControllerReturn, sizeof...(OutputOptions)> results = {
        BaseOutput<OutputOptions>::on_write(time, period,
                                            std::forward<decltype(outputs)>(outputs))...};
    return std::all_of(results.begin(), results.end(),
                       [](const auto result) { return result == BaseCommon::ControllerReturn::OK; })
               ? BaseCommon::ControllerReturn::OK
               : BaseCommon::ControllerReturn::ERROR;
  }
};

} // namespace ffmpeg_controllers
#endif