#ifndef FFMPEG_CONTROLLERS_DETAIL_INPUT_MIXIN_HPP
#define FFMPEG_CONTROLLERS_DETAIL_INPUT_MIXIN_HPP

#include <optional>
#include <stdexcept>
#include <string>
#include <utility> // for std::pair<>

#include <ffmpeg_controllers/detail/input_output_options.hpp>
#include <ffmpeg_controllers/detail/interface_adapter.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace ffmpeg_controllers {

// ====================================
// Mixin class depending on InputOption
// ====================================

template <typename InputOption, class Interface> class InputMixin;

// By inheriting this class in each InputMixin<>, we force the intended on_read() signature
template <typename Inputs> class OnReadDefinition {
protected:
  using CRefInputs = ElementwiseCRef<Inputs>;

  virtual std::pair<controller_interface::return_type, std::optional<CRefInputs>>
  on_read(const rclcpp::Time &time, const rclcpp::Duration &period) = 0;
};

template <class Interface>
class InputMixin<input_options::ReadFrame, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnReadDefinition<InputsFor<input_options::ReadFrame>> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller from which the state interface is loaned
      input_name_ = Base::get_node()->template declare_parameter<std::string>("input_name");
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  typename Base::NodeReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    prev_dts_ = 0;
    return Base::NodeReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {input_name_ + "/frame"}};
  }

  std::pair<typename Base::ControllerReturn, std::optional<CRefInputs>>
  on_read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (const auto input_frame =
            Base::template get_state_as_pointer<ffmpeg_cpp::Frame>(input_name_, "frame");
        input_frame && (*input_frame)->pkt_dts > prev_dts_) {
      // Return the input frame if it is new
      prev_dts_ = (*input_frame)->pkt_dts;
      return {Base::ControllerReturn::OK, {std::cref(*input_frame)}};
    } else if (input_frame) {
      // Return OK with no frame if the frame is not new
      return {Base::ControllerReturn::OK, std::nullopt};
    } else {
      // It is still OK if the input frame is not available
      RCLCPP_WARN(Base::get_logger(), "Failed to get input frame. Will skip this update.");
      return {Base::ControllerReturn::OK, std::nullopt};
    }
  }

private:
  std::string input_name_;
  decltype(ffmpeg_cpp::Frame()->pkt_dts) prev_dts_;
};

template <class Interface>
class InputMixin<input_options::ReadPacket, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnReadDefinition<InputsFor<input_options::ReadPacket>> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller from which the state interface is loaned
      input_name_ = Base::get_node()->template declare_parameter<std::string>("input_name");
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  typename Base::NodeReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    prev_dts_ = 0;
    return Base::NodeReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {input_name_ + "/packet"}};
  }

  std::pair<typename Base::ControllerReturn, std::optional<CRefInputs>>
  on_read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (const auto input_packet =
            Base::template get_state_as_pointer<ffmpeg_cpp::Packet>(input_name_, "packet");
        input_packet && (*input_packet)->dts > prev_dts_) {
      // Return the input packet if it is new
      prev_dts_ = (*input_packet)->dts;
      return {Base::ControllerReturn::OK, {std::cref(*input_packet)}};
    } else if (input_packet) {
      // Return OK with no packet if the packet is not new
      return {Base::ControllerReturn::OK, std::nullopt};
    } else {
      // It is still OK if the input packet is not available
      RCLCPP_WARN(Base::get_logger(), "Failed to get input packet. Will skip this update.");
      return {Base::ControllerReturn::OK, std::nullopt};
    }
  }

private:
  std::string input_name_;
  decltype(ffmpeg_cpp::Packet()->dts) prev_dts_;
};

template <class Interface>
class InputMixin<input_options::ReadPacketWithParams, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnReadDefinition<InputsFor<input_options::ReadPacketWithParams>> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller from which the state interface is loaned
      input_name_ = Base::get_node()->template declare_parameter<std::string>("input_name");
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  typename Base::NodeReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    prev_dts_ = 0;
    return Base::NodeReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {input_name_ + "/packet", input_name_ + "/codec_parameters"}};
  }

  std::pair<typename Base::ControllerReturn, std::optional<CRefInputs>>
  on_read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    const auto input_packet =
        Base::template get_state_as_pointer<ffmpeg_cpp::Packet>(input_name_, "packet");
    const auto codec_params = Base::template get_state_as_pointer<ffmpeg_cpp::CodecParameters>(
        input_name_, "codec_parameters");
    if (input_packet && codec_params && (*input_packet)->dts > prev_dts_) {
      // Return the input packet if it is new
      prev_dts_ = (*input_packet)->dts;
      return {Base::ControllerReturn::OK,
              CRefInputs{std::cref(*input_packet), std::cref(*codec_params)}};
    } else if (input_packet && codec_params) {
      // Return OK with no packet if the packet is not new
      return {Base::ControllerReturn::OK, std::nullopt};
    } else {
      // It is still OK if the input packet is not available
      RCLCPP_WARN(Base::get_logger(),
                  "Failed to get input packet or codec parameters. Will skip this update.");
      return {Base::ControllerReturn::OK, std::nullopt};
    }
  }

private:
  std::string input_name_;
  decltype(ffmpeg_cpp::Packet()->dts) prev_dts_;
};

template <typename Message, class Interface>
class InputMixin<input_options::Subscribe<Message>, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnReadDefinition<InputsFor<input_options::Subscribe<Message>>> {
private:
  using Base = InterfaceAdapter<Interface>;
  using BaseRead = OnReadDefinition<InputsFor<input_options::Subscribe<Message>>>;

protected:
  using InputMessage = Message;

  typename Base::NodeReturn on_init() override {
    input_topic_ = "~/input";
    return Base::NodeReturn::SUCCESS;
  }

  typename Base::NodeReturn
  on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      subscription_ = Base::get_node()->template create_subscription<InputMessage>(
          input_topic_, rclcpp::SystemDefaultsQoS(),
          [this](const typename InputMessage::ConstSharedPtr msg) {
            msg_buffer_.writeFromNonRT(msg);
          });
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while creating subscription: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  typename Base::NodeReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    msg_buffer_ = realtime_tools::RealtimeBuffer<typename InputMessage::ConstSharedPtr>();
    return Base::NodeReturn::SUCCESS;
  }

  std::pair<typename Base::ControllerReturn, std::optional<typename BaseRead::CRefInputs>>
  on_read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    if (const typename InputMessage::ConstSharedPtr msg = *msg_buffer_.readFromRT(); msg) {
      // Return the input message if it is available
      // TODO: Check if the message is new?
      return {Base::ControllerReturn::OK, {std::cref(*msg)}};
    } else {
      // It is still OK if the input message is not available
      return {Base::ControllerReturn::OK, std::nullopt};
    }
  }

protected:
  std::string input_topic_;

private:
  realtime_tools::RealtimeBuffer<typename InputMessage::ConstSharedPtr> msg_buffer_;
  typename rclcpp::Subscription<InputMessage>::SharedPtr subscription_;
};

} // namespace ffmpeg_controllers

#endif