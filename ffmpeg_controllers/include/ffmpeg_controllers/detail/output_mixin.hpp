#ifndef FFMPEG_CONTROLLERS_DETAIL_OUTPUT_MIXIN_HPP
#define FFMPEG_CONTROLLERS_DETAIL_OUTPUT_MIXIN_HPP

#include <stdexcept>
#include <string>
#include <tuple>

#include <controller_interface/controller_interface_base.hpp> // for controller_interface::return_type
#include <ffmpeg_controllers/controller_options.hpp>
#include <ffmpeg_controllers/detail/controller_interface_adapter.hpp>
#include <ffmpeg_controllers/detail/controller_traits.hpp>
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

template <typename OutputOption, class ControllerIface> class OutputMixin;

template <typename Object, class ControllerIface>
class OutputMixin<output_options::Export<Object>, ControllerIface>
    : public virtual ControllerInterfaceAdapter<ControllerIface>,
      public OnWriteContract<output_options::Export<Object>> {
private:
  using Base = ControllerInterfaceAdapter<ControllerIface>;

protected:
  typename Base::NodeReturn on_init() override {
    // Names of intraprocess read-only variables to be exported
    Base::exported_state_interface_names_.emplace_back(HardwareInterfaceName<Object>);
    return Base::NodeReturn::SUCCESS;
  }

  typename Base::NodeReturn
  on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    // Unregister the object from state interface owned by this controller
    return Base::set_state_from_pointer(HardwareInterfaceName<Object>, nullptr)
               ? Base::NodeReturn::SUCCESS
               : Base::NodeReturn::ERROR;
  }

  typename Base::ControllerReturn on_write(const rclcpp::Time & /*time*/,
                                           const rclcpp::Duration & /*period*/,
                                           Object &&input_object) override {
    // Export the given object to the state interface owned by this controller
    output_object_ = std::move(input_object);
    return Base::set_state_from_pointer(HardwareInterfaceName<Object>, &output_object_)
               ? Base::ControllerReturn::OK
               : Base::ControllerReturn::ERROR;
  }

private:
  Object output_object_ = Object::null();
};

template <typename Object, class ControllerIface>
class OutputMixin<output_options::Write<Object>, ControllerIface>
    : public virtual ControllerInterfaceAdapter<ControllerIface>,
      public OnWriteContract<output_options::Write<Object>> {
private:
  using Base = ControllerInterfaceAdapter<ControllerIface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller to which the command interface is written
      output_name_ = Base::template get_user_parameter<std::string>("output_name");
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {output_name_ + "/" + HardwareInterfaceName<Object>}};
  }

  typename Base::ControllerReturn on_write(const rclcpp::Time & /*time*/,
                                           const rclcpp::Duration & /*period*/,
                                           Object &&input_object) override {
    if (const auto output_object = Base::template get_command_as_pointer<Object>(
            output_name_, HardwareInterfaceName<Object>);
        output_object) {
      // Update the command variable with the given object
      *output_object = std::move(input_object);
      return Base::ControllerReturn::OK;
    } else {
      // It is still OK if the command variable is not available
      RCLCPP_WARN(Base::get_logger(), "Failed to get output object. Will skip this update.");
      return Base::ControllerReturn::OK;
    }
  }

private:
  std::string output_name_;
};

template <typename Message, class ControllerIface>
class OutputMixin<output_options::Publish<Message>, ControllerIface>
    : public virtual ControllerInterfaceAdapter<ControllerIface>,
      public OnWriteContract<output_options::Publish<Message>> {
private:
  using Base = ControllerInterfaceAdapter<ControllerIface>;

protected:
  using OutputMessage = Message;

  typename Base::NodeReturn on_init() override {
    output_topic_ = TopicName<Message>;
    return Base::NodeReturn::SUCCESS;
  }

  typename Base::NodeReturn
  on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override {
    try {
      rclcpp::PublisherOptions options;
      options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
      underlying_publisher_ = Base::get_node()->template create_publisher<OutputMessage>(
          output_topic_, rclcpp::SystemDefaultsQoS(), options);
      async_publisher_ =
          std::make_unique<realtime_tools::RealtimePublisher<OutputMessage>>(underlying_publisher_);
      RCLCPP_INFO(Base::get_logger(), "Created publisher on %s",
                  underlying_publisher_->get_topic_name());
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

template <typename... OutputOptions, class ControllerIface>
class OutputMixin<std::tuple<OutputOptions...>, ControllerIface>
    : public OutputMixin<OutputOptions, ControllerIface>...,
      public OnWriteContract<std::tuple<OutputOptions...>> {
private:
  using BaseCommon = ControllerInterfaceAdapter<ControllerIface>;
  template <typename OutputOption> using BaseOutput = OutputMixin<OutputOption, ControllerIface>;

protected:
  typename BaseCommon::NodeReturn on_init() override {
    const std::array<typename BaseCommon::NodeReturn, sizeof...(OutputOptions)> results = {
        BaseOutput<OutputOptions>::on_init()...};
    return BaseCommon::merge(results);
  }

  typename BaseCommon::NodeReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override {
    const std::array<typename BaseCommon::NodeReturn, sizeof...(OutputOptions)> results = {
        BaseOutput<OutputOptions>::on_deactivate(previous_state)...};
    return BaseCommon::merge(results);
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    const std::array<controller_interface::InterfaceConfiguration, sizeof...(OutputOptions)>
        results = {BaseOutput<OutputOptions>::command_interface_configuration()...};
    return BaseCommon::merge(results);
  }

  typename BaseCommon::ControllerReturn on_write(const rclcpp::Time &time,
                                                 const rclcpp::Duration &period,
                                                 OutputFor<OutputOptions> &&...outputs) override {
    // Call on_read() for each input option and collect the results
    const std::array<typename BaseCommon::ControllerReturn, sizeof...(OutputOptions)> results = {
        BaseOutput<OutputOptions>::on_write(time, period,
                                            std::forward<decltype(outputs)>(outputs))...};
    return BaseCommon::merge(results);
  }
};

} // namespace ffmpeg_controllers
#endif