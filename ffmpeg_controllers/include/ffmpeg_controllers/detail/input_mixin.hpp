#ifndef FFMPEG_CONTROLLERS_DETAIL_INPUT_MIXIN_HPP
#define FFMPEG_CONTROLLERS_DETAIL_INPUT_MIXIN_HPP

#include <array>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility> // for std::pair<>

#include <ffmpeg_controllers/controller_options.hpp>
#include <ffmpeg_controllers/detail/controller_traits.hpp>
#include <ffmpeg_controllers/detail/interface_adapter.hpp>
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
  // The final argument is used to change the signature of the function for each InputOption type,
  // and not supposed to be used. This allows multiple OnReadContract to be inherited.
  virtual OnReadReturn<InputOption> on_read(const rclcpp::Time &time,
                                            const rclcpp::Duration &period, InputOption) = 0;
};

// ====================================
// Mixin class depending on InputOption
// ====================================

template <typename InputOption, class Interface> class InputMixin;

template <class Interface>
class InputMixin<input_options::Read<ffmpeg_cpp::Frame>, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnReadContract<input_options::Read<ffmpeg_cpp::Frame>> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller from which the state interface is loaned
      input_name_ = Base::template declare_or_get_parameter<std::string>("input_name");
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

  OnReadReturn<input_options::Read<ffmpeg_cpp::Frame>>
  on_read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
          input_options::Read<ffmpeg_cpp::Frame>) override {
    if (const auto input_frame =
            Base::template get_state_as_pointer<ffmpeg_cpp::Frame>(input_name_, "frame");
        input_frame && (*input_frame)->pkt_dts > prev_dts_) {
      // Return the input frame if it is new
      prev_dts_ = (*input_frame)->pkt_dts;
      return {Base::ControllerReturn::OK, std::cref(*input_frame)};
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
class InputMixin<input_options::Read<ffmpeg_cpp::Packet>, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnReadContract<input_options::Read<ffmpeg_cpp::Packet>> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller from which the state interface is loaned
      input_name_ = Base::template declare_or_get_parameter<std::string>("input_name");
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

  OnReadReturn<input_options::Read<ffmpeg_cpp::Packet>>
  on_read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
          input_options::Read<ffmpeg_cpp::Packet>) override {
    if (const auto input_packet =
            Base::template get_state_as_pointer<ffmpeg_cpp::Packet>(input_name_, "packet");
        input_packet && (*input_packet)->dts > prev_dts_) {
      // Return the input packet if it is new
      prev_dts_ = (*input_packet)->dts;
      return {Base::ControllerReturn::OK, std::cref(*input_packet)};
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
class InputMixin<input_options::Read<ffmpeg_cpp::CodecParameters>, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnReadContract<input_options::Read<ffmpeg_cpp::CodecParameters>> {
private:
  using Base = InterfaceAdapter<Interface>;

protected:
  typename Base::NodeReturn on_init() override {
    try {
      // The name of the hardware or controller from which the state interface is loaned
      input_name_ = Base::template declare_or_get_parameter<std::string>("input_name");
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while getting parameter value: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return {controller_interface::interface_configuration_type::INDIVIDUAL,
            {input_name_ + "/codec_parameters"}};
  }

  OnReadReturn<input_options::Read<ffmpeg_cpp::CodecParameters>>
  on_read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/,
          input_options::Read<ffmpeg_cpp::CodecParameters>) override {
    if (const auto codec_params = Base::template get_state_as_pointer<ffmpeg_cpp::CodecParameters>(
            input_name_, "codec_parameters");
        codec_params) {
      return {Base::ControllerReturn::OK, std::cref(*codec_params)};
    } else {
      RCLCPP_WARN(Base::get_logger(), "Failed to get codec parameters. Will skip this update.");
      return {Base::ControllerReturn::OK, std::nullopt};
    }
  }

private:
  std::string input_name_;
};

template <typename Message, class Interface>
class InputMixin<input_options::Subscribe<Message>, Interface>
    : public virtual InterfaceAdapter<Interface>,
      public OnReadContract<input_options::Subscribe<Message>> {
private:
  using Base = InterfaceAdapter<Interface>;
  using StampedMessage = message_filters::MessageEvent<Message>;

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
            msg_buffer_.writeFromNonRT(StampedMessage(msg, Base::get_node()->now()));
          });
      return Base::NodeReturn::SUCCESS;
    } catch (const std::runtime_error &error) {
      RCLCPP_ERROR(Base::get_logger(), "Error while creating subscription: %s", error.what());
      return Base::NodeReturn::ERROR;
    }
  }

  typename Base::NodeReturn
  on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override {
    prev_stamp_ = Base::get_node()->now();
    msg_buffer_.writeFromNonRT(StampedMessage(nullptr, prev_stamp_));
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

protected:
  std::string input_topic_;

private:
  typename rclcpp::Subscription<InputMessage>::SharedPtr subscription_;
  realtime_tools::RealtimeBuffer<StampedMessage> msg_buffer_;
  rclcpp::Time prev_stamp_;
};

template <typename... InputOptions, class Interface>
class InputMixin<std::tuple<InputOptions...>, Interface>
    : public InputMixin<InputOptions, Interface>...,
      public OnReadContract<std::tuple<InputOptions...>> {
private:
  using BaseCommon = InterfaceAdapter<Interface>;
  template <typename InputOption> using BaseInput = InputMixin<InputOption, Interface>;

protected:
  typename BaseCommon::NodeReturn on_init() override {
    const std::array<typename BaseCommon::NodeReturn, sizeof...(InputOptions)> results = {
        BaseInput<InputOptions>::on_init()...};
    return std::all_of(results.begin(), results.end(),
                       [](const auto result) { return result == BaseCommon::NodeReturn::SUCCESS; })
               ? BaseCommon::NodeReturn::SUCCESS
               : BaseCommon::NodeReturn::ERROR;
  }

  typename BaseCommon::NodeReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override {
    const std::array<typename BaseCommon::NodeReturn, sizeof...(InputOptions)> results = {
        BaseInput<InputOptions>::on_activate(previous_state)...};
    return std::all_of(results.begin(), results.end(),
                       [](const auto result) { return result == BaseCommon::NodeReturn::SUCCESS; })
               ? BaseCommon::NodeReturn::SUCCESS
               : BaseCommon::NodeReturn::ERROR;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    using ConfigType = controller_interface::interface_configuration_type;
    const std::array<controller_interface::InterfaceConfiguration, sizeof...(InputOptions)>
        results = {BaseInput<InputOptions>::state_interface_configuration()...};
    if (std::any_of(results.begin(), results.end(),
                    [](const auto &config) { return config.type == ConfigType::ALL; })) {
      return {ConfigType::ALL, {}};
    } else if (std::all_of(results.begin(), results.end(),
                           [](const auto &config) { return config.type == ConfigType::NONE; })) {
      return {ConfigType::NONE, {}};
    } else {
      std::set<std::string> names;
      for (const auto &result : results) {
        if (result.type == ConfigType::INDIVIDUAL) {
          names.insert(result.names.begin(), result.names.end());
        }
      }
      return {ConfigType::INDIVIDUAL, std::vector<std::string>(names.begin(), names.end())};
    }
  }

  OnReadReturn<std::tuple<InputOptions...>> on_read(const rclcpp::Time &time,
                                                    const rclcpp::Duration &period,
                                                    std::tuple<InputOptions...>) override {
    // Call on_read() for each input option and collect the results
    const auto results =
        std::make_tuple(BaseInput<InputOptions>::on_read(time, period, InputOptions())...);

    // Reshape the results into a list of success and a list of outputs
    const auto [success, outputs] = transpose(results);

    // Return overall result
    return {// Overall success - OK if all of on_read() returned OK
            std::all_of(success.begin(), success.end(),
                        [](const auto s) { return s == BaseCommon::ControllerReturn::OK; })
                ? BaseCommon::ControllerReturn::OK
                : BaseCommon::ControllerReturn::ERROR,
            // Overall outputs - not a nullopt if any of on_read() did not return nullopt
            flip(outputs)};
  }

  // Make pair<array<T, N>, tuple<U0, U1, ...>> by transposing tuple<pair<T, U0>, pair<T, U1>, ...>
  template <typename First, typename... Second>
  static std::pair<std::array<First, sizeof...(Second)>, std::tuple<Second...>>
  transpose(const std::tuple<std::pair<First, Second>...> &src) {
    return transpose_impl(src, std::index_sequence_for<InputOptions...>());
  }

  template <typename First, typename... Second, std::size_t... Ids>
  static std::pair<std::array<First, sizeof...(Second)>, std::tuple<Second...>>
  transpose_impl(const std::tuple<std::pair<First, Second>...> &src, std::index_sequence<Ids...>) {
    return {{std::get<Ids>(src).first...}, {std::get<Ids>(src).second...}};
  }

  // Make optional<tuple<T0, T1, ...>> by flipping tuple<optional<T0>, optional<T1>, ...>
  template <typename... Ts>
  static std::optional<std::tuple<Ts...>> flip(const std::tuple<std::optional<Ts>...> &src) {
    return flip_impl(src, std::index_sequence_for<Ts...>());
  }

  template <typename... Ts, std::size_t... Ids>
  static std::optional<std::tuple<Ts...>> flip_impl(const std::tuple<std::optional<Ts>...> &src,
                                                    std::index_sequence<Ids...>) {
    return (std::get<Ids>(src) && ...) ? std::make_optional(std::make_tuple(*std::get<Ids>(src)...))
                                       : std::nullopt;
  }
};

} // namespace ffmpeg_controllers

#endif