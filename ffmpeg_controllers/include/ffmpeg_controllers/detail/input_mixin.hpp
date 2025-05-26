#ifndef FFMPEG_CONTROLLERS_DETAIL_INPUT_MIXIN_HPP
#define FFMPEG_CONTROLLERS_DETAIL_INPUT_MIXIN_HPP

#include <optional>
#include <stdexcept>
#include <string>
#include <utility> // for std::pair<>

#include <ffmpeg_controllers/controller_options.hpp>
#include <ffmpeg_controllers/detail/controller_interface_adapter.hpp>
#include <ffmpeg_controllers/detail/controller_traits.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <message_filters/message_event.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_buffer.hpp>

namespace ffmpeg_controllers {

// ======================================================================
// Helper class to define the pure virtual function InputMixin::on_read()
// which is used to read the input
// ======================================================================

// Return type of InputMixin::on_read()
template <typename InputOption> struct GetOnReadReturn {
  using Result = std::pair<controller_interface::return_type,
                           std::optional<std::reference_wrapper<const InputFor<InputOption>>>>;
};
template <typename... InputOptions> struct GetOnReadReturn<std::tuple<InputOptions...>> {
  using Result =
      std::pair<controller_interface::return_type,
                std::optional<std::tuple<std::reference_wrapper<const InputFor<InputOptions>>...>>>;
};
template <typename InputOption> using OnReadReturn = typename GetOnReadReturn<InputOption>::Result;

// Definition of InputOutputMixin::on_read()
template <typename InputOption> class OnReadContract {
protected:
  // The third parameter is a dummy InputOption, never used at runtime but required to
  // alter the signature per InputOption, allowing multiple OnReadContract inheritance.
  virtual OnReadReturn<InputOption> on_read(const rclcpp::Time &time,
                                            const rclcpp::Duration &period, InputOption) = 0;
};

// ====================================
// Mixin class depending on InputOption
// ====================================

template <typename InputOption, class ControllerIface> class InputMixin;

template <class ControllerIface>
class InputMixin<input_options::Read<ffmpeg_cpp::Frame>, ControllerIface>
    : public virtual ControllerInterfaceAdapter<ControllerIface>,
      public OnReadContract<input_options::Read<ffmpeg_cpp::Frame>> {
private:
  using Base = ControllerInterfaceAdapter<ControllerIface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller from which the state interface is loaned
      input_name_ = Base::template get_user_parameter<std::string>("input_name");
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
            {input_name_ + "/" + HardwareInterfaceName<ffmpeg_cpp::Frame>}};
  }

  OnReadReturn<input_options::Read<ffmpeg_cpp::Frame>>
  on_read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
          input_options::Read<ffmpeg_cpp::Frame>) override {
    if (const auto input_frame = Base::template get_state_as_pointer<ffmpeg_cpp::Frame>(
            input_name_, HardwareInterfaceName<ffmpeg_cpp::Frame>);
        input_frame && (*input_frame)->pkt_dts > prev_dts_) {
      // Return the input frame if it is new
      prev_dts_ = (*input_frame)->pkt_dts;
      return {Base::ControllerReturn::OK, std::cref(*input_frame)};
    } else if (input_frame) {
      // Return OK with no frame if the frame is not new
      return {Base::ControllerReturn::OK, std::nullopt};
    } else {
      // It is still OK if the input frame is not available
      // RCLCPP_WARN(Base::get_logger(), "Failed to get input frame. Will skip this update.");
      return {Base::ControllerReturn::OK, std::nullopt};
    }
  }

private:
  std::string input_name_;
  std::int64_t prev_dts_;
};

template <class ControllerIface>
class InputMixin<input_options::Read<ffmpeg_cpp::Packet>, ControllerIface>
    : public virtual ControllerInterfaceAdapter<ControllerIface>,
      public OnReadContract<input_options::Read<ffmpeg_cpp::Packet>> {
private:
  using Base = ControllerInterfaceAdapter<ControllerIface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller from which the state interface is loaned
      input_name_ = Base::template get_user_parameter<std::string>("input_name");
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
            {input_name_ + "/" + HardwareInterfaceName<ffmpeg_cpp::Packet>}};
  }

  OnReadReturn<input_options::Read<ffmpeg_cpp::Packet>>
  on_read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
          input_options::Read<ffmpeg_cpp::Packet>) override {
    if (const auto input_packet = Base::template get_state_as_pointer<ffmpeg_cpp::Packet>(
            input_name_, HardwareInterfaceName<ffmpeg_cpp::Packet>);
        input_packet && (*input_packet)->dts > prev_dts_) {
      // Return the input packet if it is new
      prev_dts_ = (*input_packet)->dts;
      return {Base::ControllerReturn::OK, std::cref(*input_packet)};
    } else if (input_packet) {
      // Return OK with no packet if the packet is not new
      return {Base::ControllerReturn::OK, std::nullopt};
    } else {
      // It is still OK if the input packet is not available
      // RCLCPP_WARN(Base::get_logger(), "Failed to get input packet. Will skip this update.");
      return {Base::ControllerReturn::OK, std::nullopt};
    }
  }

private:
  std::string input_name_;
  std::int64_t prev_dts_;
};

template <typename Object, class ControllerIface>
class InputMixin<input_options::Read<Object>, ControllerIface>
    : public virtual ControllerInterfaceAdapter<ControllerIface>,
      public OnReadContract<input_options::Read<Object>> {
private:
  using Base = ControllerInterfaceAdapter<ControllerIface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller from which the state interface is loaned
      input_name_ = Base::template get_user_parameter<std::string>("input_name");
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {input_name_ + "/" + HardwareInterfaceName<Object>}};
  }

  OnReadReturn<input_options::Read<Object>> on_read(const rclcpp::Time & /*time*/,
                                                    const rclcpp::Duration & /*period*/,
                                                    input_options::Read<Object>) override {
    if (const auto object =
            Base::template get_state_as_pointer<Object>(input_name_, HardwareInterfaceName<Object>);
        object) {
      return {Base::ControllerReturn::OK, std::cref(*object)};
    } else {
      // RCLCPP_WARN(Base::get_logger(), "Failed to get input object. Will skip this update.");
      return {Base::ControllerReturn::OK, std::nullopt};
    }
  }

private:
  std::string input_name_;
};

template <typename Message, class ControllerIface>
class InputMixin<input_options::Subscribe<Message>, ControllerIface>
    : public virtual ControllerInterfaceAdapter<ControllerIface>,
      public OnReadContract<input_options::Subscribe<Message>> {
private:
  using Base = ControllerInterfaceAdapter<ControllerIface>;
  using StampedMessage = message_filters::MessageEvent<Message>;

protected:
  using InputMessage = Message;

  typename Base::NodeReturn
  on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      rclcpp::SubscriptionOptions options;
      options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
      subscription_ = Base::get_node()->template create_subscription<InputMessage>(
          TopicName<Message>, rclcpp::SystemDefaultsQoS(),
          [this](const typename InputMessage::ConstSharedPtr msg) {
            msg_buffer_.writeFromNonRT(StampedMessage(msg, Base::get_node()->now()));
          },
          options);
      RCLCPP_INFO(Base::get_logger(), "Created subscription to %s",
                  subscription_->get_topic_name());
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while creating subscription: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  typename Base::NodeReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    prev_stamp_ = Base::get_node()->now();
    msg_buffer_.writeFromNonRT(StampedMessage(
        // as of jazzy, passing nullptr leads a segmentation fault
        std::make_shared<InputMessage>(), prev_stamp_));
    return Base::NodeReturn::SUCCESS;
  }

  OnReadReturn<input_options::Subscribe<Message>>
  on_read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
          input_options::Subscribe<Message>) override {
    if (const StampedMessage &msg = *msg_buffer_.readFromRT(); msg.getReceiptTime() > prev_stamp_) {
      // Return the input message if it is new
      prev_stamp_ = msg.getReceiptTime();
      return {Base::ControllerReturn::OK, std::cref(*msg.getConstMessage())};
    } else {
      // It is still OK if the input message is not new
      return {Base::ControllerReturn::OK, std::nullopt};
    }
  }

private:
  typename rclcpp::Subscription<InputMessage>::SharedPtr subscription_;
  realtime_tools::RealtimeBuffer<StampedMessage> msg_buffer_;
  rclcpp::Time prev_stamp_;
};

template <typename... InputOptions, class ControllerIface>
class InputMixin<std::tuple<InputOptions...>, ControllerIface>
    : public InputMixin<InputOptions, ControllerIface>...,
      public OnReadContract<std::tuple<InputOptions...>> {
private:
  using BaseCommon = ControllerInterfaceAdapter<ControllerIface>;
  template <typename InputOption> using BaseInput = InputMixin<InputOption, ControllerIface>;

protected:
  typename BaseCommon::NodeReturn on_init() override {
    return BaseCommon::merge({BaseInput<InputOptions>::on_init()...});
  }

  typename BaseCommon::NodeReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override {
    return BaseCommon::merge({BaseInput<InputOptions>::on_activate(previous_state)...});
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return BaseCommon::merge({BaseInput<InputOptions>::state_interface_configuration()...});
  }

  OnReadReturn<std::tuple<InputOptions...>> on_read(const rclcpp::Time &time,
                                                    const rclcpp::Duration &period,
                                                    std::tuple<InputOptions...>) override {
    // Call on_read() for each InputOption and defer to the implementation
    return on_read_impl(BaseInput<InputOptions>::on_read(time, period, InputOptions())...);
  }

  static OnReadReturn<std::tuple<InputOptions...>>
  on_read_impl(OnReadReturn<InputOptions> &&...results) {
    return {// Overall success - OK if all of on_read() returned OK
            BaseCommon::merge({std::move(results.first)...}),
            // Overall outputs - not a nullopt if any of on_read() did not return nullopt
            (results.second && ...)
                ? std::make_optional(std::make_tuple(std::move(*results.second)...))
                : std::nullopt};
  }
};

} // namespace ffmpeg_controllers

#endif