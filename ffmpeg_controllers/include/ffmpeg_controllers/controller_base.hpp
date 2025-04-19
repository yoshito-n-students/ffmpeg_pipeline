#ifndef FFMPEG_CONTROLLERS_CONTROLLER_BASE_HPP
#define FFMPEG_CONTROLLERS_CONTROLLER_BASE_HPP

#include <algorithm>
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

namespace controller_base_mixin {

// ===========================
// Tag types for mixin classes
// ===========================

// InputOptions
// - Read state interface loaned from other hardware or controller
struct ReadState {};
// - Subscribe a topic
template <typename Message> struct Subscribe {};

// OutputOptions
// - Export state interface to other hardware or controller
struct ExportState {};
// - Write to command interface loaned from other hardware or controller
struct WriteCommand {};
// - Publish messages to a topic
template <typename Message> struct Publish {};

// InheritanceOptions (automatically determined from OutputOptions)
// - Inherit controller_interface::ControllerInterface
struct InheritControllerInterface {};
// - Inherit controller_interface::ChainableControllerInterface
struct InheritChainableControllerInterface {};

// Mapping from OutputOption to InheritanceOption
template <typename OutputOption> struct GetInheritanceOptionFor {};
template <> struct GetInheritanceOptionFor<ExportState> {
  // ChainableControllerInterface must be inherited to export state interfaces
  using Result = InheritChainableControllerInterface;
};
template <> struct GetInheritanceOptionFor<WriteCommand> {
  using Result = InheritControllerInterface;
};
template <typename Message> struct GetInheritanceOptionFor<Publish<Message>> {
  using Result = InheritControllerInterface;
};
template <typename OutputOption>
using InheritanceOptionFor = typename GetInheritanceOptionFor<OutputOption>::Result;

// ================================
// Mixin class for empty base class
// ================================

class EmptyMixin {};

// ====================================
// Mixin class depending on InputOption
// ====================================

template <typename InputOption, class InnerMixin = EmptyMixin> class InputMixin;

template <class InnerMixin> class InputMixin<ReadState, InnerMixin> : public virtual InnerMixin {
protected:
  // The name of the hardware or controller from which the state interface is loaned
  std::string input_name_;
};

template <typename Message, class InnerMixin>
class InputMixin<Subscribe<Message>, InnerMixin> : public virtual InnerMixin {
protected:
  using InputMessage = Message;

  std::string input_topic_;
  realtime_tools::RealtimeBuffer<typename Message::ConstSharedPtr> msg_buffer_;
  typename rclcpp::Subscription<Message>::SharedPtr subscription_;
};

// =====================================
// Mixin class depending on OutputOption
// =====================================

template <typename OutputOption, class InnerMixin = EmptyMixin> class OutputMixin;

template <class InnerMixin>
class OutputMixin<ExportState, InnerMixin> : public virtual InnerMixin {};

template <class InnerMixin>
class OutputMixin<WriteCommand, InnerMixin> : public virtual InnerMixin {
protected:
  // The name of the hardware or controller to which the command interface is written
  std::string output_name_;
};

template <typename Message, class InnerMixin>
class OutputMixin<Publish<Message>, InnerMixin> : public virtual InnerMixin {
protected:
  using OutputMessage = Message;

  std::string output_topic_;
  typename rclcpp::Publisher<Message>::SharedPtr underlying_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<Message>> async_publisher_;
};

// ==========================================
// Mixin class depending on InheritanceOption
// ==========================================

template <typename InheritanceOption, class InnerMixin = EmptyMixin> class InheritanceMixin;

// Common base class for all variations of InheritanceMixin<>
template <class Interface> struct InterfaceAdapter : public virtual Interface {
protected:
  // Return types of virtual interface functions
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

  // Set the pointer value to the state interface exported from the derived class
  template <typename T>
  bool set_state_from_pointer(const std::string &iface_name, const T *const ptr_value) {
    // Find the state interface specified by iface_name owned by this controller
    for (const auto &iface : Interface::ordered_exported_state_interfaces_) {
      if (iface && iface->get_interface_name() != iface_name) {
        continue;
      }
      // Convert the pointer to a double value and set it to the state interface
      return iface->set_value(static_cast<double>(reinterpret_cast<std::uintptr_t>(ptr_value)));
    }
    return false;
  }
};

template <class InnerMixin>
class InheritanceMixin<InheritControllerInterface, InnerMixin>
    : public virtual InterfaceAdapter<controller_interface::ControllerInterface>,
      public virtual InnerMixin {
private:
  using Interface = InterfaceAdapter<controller_interface::ControllerInterface>;

protected:
  Interface::ControllerReturn update(const rclcpp::Time &time,
                                     const rclcpp::Duration &period) override {
    return on_update(time, period);
  }
};

template <class InnerMixin>
class InheritanceMixin<InheritChainableControllerInterface, InnerMixin>
    : public virtual InterfaceAdapter<controller_interface::ChainableControllerInterface>,
      public virtual InnerMixin {
private:
  using Interface = InterfaceAdapter<controller_interface::ChainableControllerInterface>;

protected:
  Interface::ControllerReturn
  update_reference_from_subscribers(const rclcpp::Time & /*time*/,
                                    const rclcpp::Duration & /*period*/) override {
    return Interface::ControllerReturn::OK;
  }

  Interface::ControllerReturn update_and_write_commands(const rclcpp::Time &time,
                                                        const rclcpp::Duration &period) override {
    return on_update(time, period);
  }
};

// ==========================================================
// Mixin class depending on InputOption and InheritanceOption
// ==========================================================

template <typename InputOption, typename InheritanceOption, class InnerMixin = EmptyMixin>
class InputInheritanceMixin;

template <typename InheritanceOption, class InnerMixin>
class InputInheritanceMixin<ReadState, InheritanceOption, InnerMixin>
    : public virtual InputMixin<ReadState, InheritanceMixin<InheritanceOption, InnerMixin>> {
private:
  using Base = InputMixin<ReadState, InheritanceMixin<InheritanceOption, InnerMixin>>;

protected:
  typename Base::NodeReturn on_init() override {
    if (const auto ret = Base::on_init(); ret != Base::NodeReturn::SUCCESS) {
      return ret;
    }

    try {
      Base::input_name_ = Base::get_node()->template declare_parameter<std::string>("input_name");
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return Base::NodeReturn::ERROR;
    }

    return Base::NodeReturn::SUCCESS;
  }

  template <typename T> const T *get_state_as_pointer(const std::string &iface_name) const {
    return Base::template get_state_as_pointer<T>(Base::input_name_, iface_name);
  }
};

template <typename Message, typename InheritanceOption, class InnerMixin>
class InputInheritanceMixin<Subscribe<Message>, InheritanceOption, InnerMixin>
    : public virtual InputMixin<Subscribe<Message>,
                                InheritanceMixin<InheritanceOption, InnerMixin>> {
private:
  using Base = InputMixin<Subscribe<Message>, InheritanceMixin<InheritanceOption, InnerMixin>>;

protected:
  typename Base::NodeReturn on_configure(const rclcpp_lifecycle::State &previous_state) override {
    if (const auto ret = Base::on_configure(previous_state); ret != Base::NodeReturn::SUCCESS) {
      return ret;
    }

    try {
      Base::subscription_ = Base::get_node()->template create_subscription<Message>(
          Base::input_topic_, rclcpp::SystemDefaultsQoS(),
          [this](const typename Message::ConstSharedPtr msg) {
            Base::msg_buffer_.writeFromNonRT(msg);
          });
    } catch (const std::exception &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while creating subscription: %s", error.what());
      return Base::NodeReturn::ERROR;
    }

    return Base::NodeReturn::SUCCESS;
  }

  typename Base::NodeReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
    if (const auto ret = Base::on_activate(previous_state); ret != Base::NodeReturn::SUCCESS) {
      return ret;
    }

    Base::msg_buffer_ = realtime_tools::RealtimeBuffer<typename Message::ConstSharedPtr>();

    return Base::NodeReturn::SUCCESS;
  }

  typename Base::NodeReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
    Base::msg_buffer_ = realtime_tools::RealtimeBuffer<typename Message::ConstSharedPtr>();

    return Base::on_deactivate(previous_state);
  }

  typename Base::ControllerReturn on_update(const rclcpp::Time &time,
                                            const rclcpp::Duration &period) override {
    if (const auto ret = Base::on_update(time, period); ret != Base::ControllerReturn::OK) {
      return ret;
    }

    if (const typename Message::ConstSharedPtr msg = *Base::msg_buffer_.readFromRT(); msg) {
      // If the message is available, process it by the derived class
      return on_process(time, period, *msg);
    } else {
      // If no message is available, just return OK
      return Base::ControllerReturn::OK;
    }
  }

  virtual typename Base::ControllerReturn
  on_process(const rclcpp::Time &time, const rclcpp::Duration &period, const Message &msg) = 0;
};

// ===========================================================
// Mixin class depending on OutputOption and InheritanceOption
// ===========================================================

template <typename OutputOption, class InnerMixin = EmptyMixin> class OutputInheritanceMixin;

template <class InnerMixin>
class OutputInheritanceMixin<ExportState, InnerMixin>
    : public virtual OutputMixin<ExportState,
                                 InheritanceMixin<InheritanceOptionFor<ExportState>, InnerMixin>> {
};

template <class InnerMixin>
class OutputInheritanceMixin<WriteCommand, InnerMixin>
    : public virtual OutputMixin<WriteCommand,
                                 InheritanceMixin<InheritanceOptionFor<WriteCommand>, InnerMixin>> {
private:
  using Base =
      OutputMixin<WriteCommand, InheritanceMixin<InheritanceOptionFor<WriteCommand>, InnerMixin>>;

protected:
  typename Base::NodeReturn on_init() override {
    if (const auto ret = Base::on_init(); ret != Base::NodeReturn::SUCCESS) {
      return ret;
    }

    try {
      Base::output_name_ = Base::get_node()->template declare_parameter<std::string>("output_name");
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return Base::NodeReturn::ERROR;
    }

    return Base::NodeReturn::SUCCESS;
  }

  template <typename T> T *get_command_as_pointer(const std::string &iface_name) const {
    return Base::template get_command_as_pointer<T>(Base::output_name_, iface_name);
  }
};

template <class Message, class InnerMixin>
class OutputInheritanceMixin<Publish<Message>, InnerMixin>
    : public virtual OutputMixin<
          Publish<Message>, InheritanceMixin<InheritanceOptionFor<Publish<Message>>, InnerMixin>> {
private:
  using Base = OutputMixin<Publish<Message>,
                           InheritanceMixin<InheritanceOptionFor<Publish<Message>>, InnerMixin>>;

protected:
  typename Base::NodeReturn on_configure(const rclcpp_lifecycle::State &previous_state) override {
    if (const auto ret = Base::on_configure(previous_state); ret != Base::NodeReturn::SUCCESS) {
      return ret;
    }

    try {
      Base::underlying_publisher_ = Base::get_node()->template create_publisher<Message>(
          Base::output_topic_, rclcpp::SystemDefaultsQoS());
      Base::async_publisher_ =
          std::make_unique<realtime_tools::RealtimePublisher<Message>>(Base::underlying_publisher_);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(Base ::get_logger(), "Error while creating publishers: %s", error.what());
      return Base::NodeReturn::ERROR;
    }

    return Base::NodeReturn::SUCCESS;
  }

  typename Base::ControllerReturn on_update(const rclcpp::Time &time,
                                            const rclcpp::Duration &period) override {
    if (const auto ret = Base::on_update(time, period); ret != Base::ControllerReturn::OK) {
      return ret;
    }

    // Publish a message if the message is generated and the publisher can be locked
    if (std::optional<Message> msg = on_generate(time, period); msg) {
      if (Base::async_publisher_->trylock()) {
        Base::async_publisher_->msg_ = std::move(*msg);
        Base::async_publisher_->unlockAndPublish();
      }
    }

    return Base::ControllerReturn::OK;
  }

  virtual std::optional<Message> on_generate(const rclcpp::Time &time,
                                             const rclcpp::Duration &period) = 0;
};

// ====================================
// Mixin class depending on all options
// ====================================

template <typename InputOption, typename OutputOption>
class AllMixin
    : public virtual InputInheritanceMixin<InputOption, InheritanceOptionFor<OutputOption>,
                                           OutputInheritanceMixin<OutputOption>> {};

} // namespace controller_base_mixin

// =====================================================
// Base classes for controllers built with mixin classes
// =====================================================

template <typename InputOption, typename OutputOption>
using ControllerBase = controller_base_mixin::AllMixin<InputOption, OutputOption>;

using FilterBase =
    ControllerBase<controller_base_mixin::ReadState, controller_base_mixin::ExportState>;
using WriterBase =
    ControllerBase<controller_base_mixin::ReadState, controller_base_mixin::WriteCommand>;
template <typename Message>
using BroadcasterBase =
    ControllerBase<controller_base_mixin::ReadState, controller_base_mixin::Publish<Message>>;
template <typename Message>
using ReceiverBase =
    ControllerBase<controller_base_mixin::Subscribe<Message>, controller_base_mixin::ExportState>;
// ControllerBase<Susbcribe, WriteCommand> is unnecessary
// because it can be realized by a combination of Receiver and Writer.
// ControllerBase<Susbcribe, Publish> is also not defined here
// as it is not a controller but a normal node.

} // namespace ffmpeg_controllers

#endif