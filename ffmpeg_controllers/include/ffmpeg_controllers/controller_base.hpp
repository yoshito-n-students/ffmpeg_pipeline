#ifndef FFMPEG_CONTROLLERS_CONTROLLER_BASE_HPP
#define FFMPEG_CONTROLLERS_CONTROLLER_BASE_HPP

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <controller_interface/chainable_controller_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace ffmpeg_controllers {

// ============================
// Tag types for ControllerBase
// ============================

namespace input_options {

// - Read state interface loaned from other hardware or controller
struct ReadState {};
// - Subscribe a topic
template <typename Message> struct Subscribe {};

} // namespace input_options

namespace output_options {

// - Export state interface to other hardware or controller
struct ExportState {};
// - Write to command interface loaned from other hardware or controller
struct WriteCommand {};
// - Publish messages to a topic
template <typename Message> struct Publish {};

} // namespace output_options

// ===================================
// Interface adapter for mixin classes
// ===================================

template <class Interface> class InterfaceAdapter;

// Common base class for all variations of InterfaceAdapter<>
template <class Interface> class InterfaceAdapterBase : public virtual Interface {
protected:
  // Provide consistent aliases for the return values of virtual interface functions
  using NodeReturn = typename Interface::CallbackReturn;      // for node-related functions
  using ControllerReturn = controller_interface::return_type; // for controller-related functions

  // Provide default implementations for virtual functions
  // defined but not implemented in controller_interface::[Chainable]ControllerInterface

  NodeReturn on_init() override { return NodeReturn::SUCCESS; }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  // Virtual function to be called during update regardless of the value of InheritanceOption
  virtual ControllerReturn on_update(const rclcpp::Time & /*time*/,
                                     const rclcpp::Duration & /*period*/) {
    return ControllerReturn::OK;
  }

  // Provide some utility functions for derived classes

  rclcpp::Logger get_logger() const { return Interface::get_node()->get_logger(); }

  // Get the pointer value stored in the state interface loaned from other classes
  template <typename T>
  const T *get_state_as_pointer(const std::string &prefix_name,
                                const std::string &iface_name) const {
    // Find the state interface specified by iface_name
    // exported from the hardware or chained controller specified by prefix_name
    for (const auto &iface : Interface::state_interfaces_) {
      if (iface.get_prefix_name() != prefix_name || iface.get_interface_name() != iface_name) {
        continue;
      }
      // Convert the double value held by the state interface to a pointer type and return it
      if (const auto double_value = iface.template get_optional<double>(); double_value) {
        return reinterpret_cast<const T *>(static_cast<std::uintptr_t>(*double_value));
      } else {
        return nullptr;
      }
    }
    return nullptr;
  }

  // Get the pointer value stored in the command interface loaned from other classes
  template <typename T>
  T *get_command_as_pointer(const std::string &prefix_name, const std::string &iface_name) const {
    // Find the state interface specified by iface_name
    // exported from the hardware or chained controller specified by prefix_name
    for (const auto &iface : Interface::command_interfaces_) {
      if (iface.get_prefix_name() != prefix_name || iface.get_interface_name() != iface_name) {
        continue;
      }
      // Convert the double value held by the state interface to a pointer type and return it
      if (const auto double_value = iface.template get_optional<double>(); double_value) {
        return reinterpret_cast<T *>(static_cast<std::uintptr_t>(*double_value));
      } else {
        return nullptr;
      }
    }
    return nullptr;
  }
};

template <>
class InterfaceAdapter<controller_interface::ControllerInterface>
    : public virtual InterfaceAdapterBase<controller_interface::ControllerInterface> {
private:
  using Base = InterfaceAdapterBase<controller_interface::ControllerInterface>;

protected:
  Base::ControllerReturn update(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    return on_update(time, period);
  }
};

template <>
class InterfaceAdapter<controller_interface::ChainableControllerInterface>
    : public virtual InterfaceAdapterBase<controller_interface::ChainableControllerInterface> {
private:
  using Base = InterfaceAdapterBase<controller_interface::ChainableControllerInterface>;

protected:
  Base::ControllerReturn
  update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                    const rclcpp::Duration & /*period*/) override {
    return Base::ControllerReturn::OK;
  }

  Base::ControllerReturn update_and_write_commands(const rclcpp::Time &time,
                                                   const rclcpp::Duration &period) override {
    return on_update(time, period);
  }

  // Set the pointer value to the state interface exported from the derived class
  template <typename T>
  std::enable_if_t<std::is_pointer_v<T> || std::is_same_v<T, std::nullptr_t>, bool>
  set_state_from_pointer(const std::string &iface_name, const T ptr_value) {
    // Find the state interface specified by iface_name owned by this controller
    for (const auto &iface : Base::ordered_exported_state_interfaces_) {
      if (iface && iface->get_interface_name() != iface_name) {
        continue;
      }
      // Convert the pointer to a double value and set it to the state interface
      return iface->set_value(static_cast<double>(reinterpret_cast<std::uintptr_t>(ptr_value)));
    }
    return false;
  }
};

// ====================================
// Mixin class depending on InputOption
// ====================================

template <typename InputOption, class Interface> class InputMixin;

template <class Interface>
class InputMixin<input_options::ReadState, Interface> : public virtual InterfaceAdapter<Interface> {
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

  template <typename T> const T *get_state_as_pointer(const std::string &iface_name) const {
    return Base::template get_state_as_pointer<T>(input_name_, iface_name);
  }

protected:
  std::string input_name_;
};

template <typename Message, class Interface>
class InputMixin<input_options::Subscribe<Message>, Interface>
    : public virtual InterfaceAdapter<Interface> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  using InputMessage = Message;

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

  typename Base::NodeReturn
  on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    msg_buffer_ = realtime_tools::RealtimeBuffer<typename InputMessage::ConstSharedPtr>();
    return Base::NodeReturn::SUCCESS;
  }

  typename Base::ControllerReturn on_update(const rclcpp::Time &time,
                                            const rclcpp::Duration &period) override {
    if (const typename InputMessage::ConstSharedPtr msg = *msg_buffer_.readFromRT(); msg) {
      // If the message is available, process it by the derived class
      return on_process(time, period, *msg);
    } else {
      // If no message is available, just return OK
      return Base::ControllerReturn::OK;
    }
  }

  virtual typename Base::ControllerReturn
  on_process(const rclcpp::Time &time, const rclcpp::Duration &period, const InputMessage &msg) = 0;

protected:
  std::string input_topic_;
  realtime_tools::RealtimeBuffer<typename InputMessage::ConstSharedPtr> msg_buffer_;
  typename rclcpp::Subscription<InputMessage>::SharedPtr subscription_;
};

// =====================================
// Mixin class depending on OutputOption
// =====================================

template <typename OutputOption, class Interface> class OutputMixin;

template <class Interface>
class OutputMixin<output_options::ExportState, Interface>
    : public virtual InterfaceAdapter<Interface> {};

template <class Interface>
class OutputMixin<output_options::WriteCommand, Interface>
    : public virtual InterfaceAdapter<Interface> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller to which the command interface is written
      output_name_ = Base::get_node()->template declare_parameter<std::string>("output_name");
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  template <typename T> T *get_command_as_pointer(const std::string &iface_name) const {
    return Base::template get_command_as_pointer<T>(output_name_, iface_name);
  }

protected:
  std::string output_name_;
};

template <typename Message, class Interface>
class OutputMixin<output_options::Publish<Message>, Interface>
    : public virtual InterfaceAdapter<Interface> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  using OutputMessage = Message;

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

  typename Base::ControllerReturn on_update(const rclcpp::Time &time,
                                            const rclcpp::Duration &period) override {
    // Publish a message if the message is generated and the publisher can be locked
    if (std::optional<OutputMessage> msg = on_generate(time, period); msg) {
      if (async_publisher_->trylock()) {
        async_publisher_->msg_ = std::move(*msg);
        async_publisher_->unlockAndPublish();
      }
    }
    return Base::ControllerReturn::OK;
  }

  virtual std::optional<OutputMessage> on_generate(const rclcpp::Time &time,
                                                   const rclcpp::Duration &period) = 0;

protected:
  std::string output_topic_;
  typename rclcpp::Publisher<OutputMessage>::SharedPtr underlying_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<OutputMessage>> async_publisher_;
};

// =====================================================
// Mixin class depending on InputOption and OutputOption
// =====================================================

// Mapping from options to interface types
template <typename InputOption, typename OutputOption>
using InterfaceFor = std::conditional_t<std::is_same_v<OutputOption, output_options::ExportState>,
                                        controller_interface::ChainableControllerInterface,
                                        controller_interface::ControllerInterface>;

template <typename InputOption, typename OutputOption>
class InputOutputMixin
    : public virtual InputMixin<InputOption, InterfaceFor<InputOption, OutputOption>>,
      public virtual OutputMixin<OutputOption, InterfaceFor<InputOption, OutputOption>> {
private:
  using BaseCommon = InterfaceAdapter<InterfaceFor<InputOption, OutputOption>>;
  using BaseInput = InputMixin<InputOption, InterfaceFor<InputOption, OutputOption>>;
  using BaseOutput = OutputMixin<OutputOption, InterfaceFor<InputOption, OutputOption>>;

protected:
  typename BaseCommon::NodeReturn on_init() override {
    if (const auto ret = BaseInput::on_init(); ret != BaseCommon::NodeReturn::SUCCESS) {
      return ret;
    }
    if (const auto ret = BaseOutput::on_init(); ret != BaseCommon::NodeReturn::SUCCESS) {
      return ret;
    }
    return BaseCommon::NodeReturn::SUCCESS;
  }
};

// =====================================================
// Base classes built with mixin classes for controllers
// =====================================================

template <typename InputOption, typename OutputOption>
using ControllerBase = InputOutputMixin<InputOption, OutputOption>;

using FilterBase = ControllerBase<input_options::ReadState, output_options::ExportState>;
using WriterBase = ControllerBase<input_options::ReadState, output_options::WriteCommand>;
template <typename Message>
using BroadcasterBase = ControllerBase<input_options::ReadState, output_options::Publish<Message>>;
template <typename Message>
using ReceiverBase = ControllerBase<input_options::Subscribe<Message>, output_options::ExportState>;
// ControllerBase<Susbcribe, WriteCommand> is unnecessary
// because it can be realized by a combination of Receiver and Writer.
// ControllerBase<Susbcribe, Publish> is also not defined here
// as it is not a controller but a normal node.

} // namespace ffmpeg_controllers

#endif