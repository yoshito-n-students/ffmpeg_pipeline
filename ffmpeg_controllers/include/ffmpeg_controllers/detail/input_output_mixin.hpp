#ifndef FFMPEG_CONTROLLERS_DETAIL_INPUT_OUTPUT_MIXIN_HPP
#define FFMPEG_CONTROLLERS_DETAIL_INPUT_OUTPUT_MIXIN_HPP

#include <optional>
#include <tuple>

#include <controller_interface/controller_interface_base.hpp> // for controller_interface::return_type
#include <ffmpeg_controllers/detail/input_mixin.hpp>
#include <ffmpeg_controllers/detail/controller_traits.hpp>
#include <ffmpeg_controllers/detail/interface_adapter.hpp>
#include <ffmpeg_controllers/detail/output_mixin.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace ffmpeg_controllers {

// ================================================================================
// Helper class to define the pure virtual function InputOutputMixin::on_generate()
// which is used to generate the output from the input
// ================================================================================

// Return type of InputOutputMixin::on_generate()
template <typename OutputOption> struct GetOnGenerateReturn {
  using Result =
      std::pair<controller_interface::return_type, std::optional<OutputFor<OutputOption>>>;
};
template <typename... OutputOptions> struct GetOnGenerateReturn<std::tuple<OutputOptions...>> {
  using Result = std::pair<controller_interface::return_type,
                           std::optional<std::tuple<OutputFor<OutputOptions>...>>>;
};
template <typename OutputOption>
using OnGenerateReturn = typename GetOnGenerateReturn<OutputOption>::Result;

// Definition of InputOutputMixin::on_generate()
template <typename InputOption, typename OutputOption> class OnGenerateDefinition {
protected:
  // Default version
  virtual OnGenerateReturn<OutputOption> on_generate(const rclcpp::Time &time,
                                                     const rclcpp::Duration &period,
                                                     const InputFor<InputOption> &input) = 0;
};
template <typename... InputOptions, typename OutputOption>
class OnGenerateDefinition<std::tuple<InputOptions...>, OutputOption> {
protected:
  // Tuple version
  virtual OnGenerateReturn<OutputOption> on_generate(const rclcpp::Time &time,
                                                     const rclcpp::Duration &period,
                                                     const InputFor<InputOptions> &...inputs) = 0;
};

// =====================================================
// Mixin class depending on InputOption and OutputOption
// =====================================================

template <typename InputOption, typename OutputOption>
class InputOutputMixin : public InputMixin<InputOption, InterfaceFor<InputOption, OutputOption>>,
                         public OutputMixin<OutputOption, InterfaceFor<InputOption, OutputOption>>,
                         public OnGenerateDefinition<InputOption, OutputOption> {
private:
  using Interface = InterfaceFor<InputOption, OutputOption>;
  using BaseCommon = InterfaceAdapter<Interface>;
  using BaseInput = InputMixin<InputOption, Interface>;
  using BaseOutput = OutputMixin<OutputOption, Interface>;

protected:
  typename BaseCommon::NodeReturn on_init() override {
    return (BaseInput::on_init() == BaseCommon::NodeReturn::SUCCESS &&
            BaseOutput::on_init() == BaseCommon::NodeReturn::SUCCESS)
               ? BaseCommon::NodeReturn::SUCCESS
               : BaseCommon::NodeReturn::ERROR;
  }

  typename BaseCommon::ControllerReturn on_update(const rclcpp::Time &time,
                                                  const rclcpp::Duration &period) override {
    const auto [read_ret, generate_args] = this->on_read(time, period, InputOption());
    if (read_ret != BaseCommon::ControllerReturn::OK || !generate_args) {
      return read_ret;
    }

    auto [generate_ret, write_args] = invoke_on_generate(time, period, *generate_args);
    if (generate_ret != BaseCommon::ControllerReturn::OK || !write_args) {
      return generate_ret;
    }

    const auto write_ret = invoke_on_write(time, period, std::move(*write_args));
    if (write_ret != BaseCommon::ControllerReturn::OK) {
      return write_ret;
    }

    return BaseCommon::ControllerReturn::OK;
  }

  // Call on_generate(time, period, input) for the given single input
  template <typename Input>
  std::enable_if_t<ElementCount<Input> == 1, OnGenerateReturn<OutputOption>>
  invoke_on_generate(const rclcpp::Time &time, const rclcpp::Duration &period, const Input &input) {
    return this->on_generate(time, period, input);
  }

  // Call on_generate(time, period, input0, input1, ...) for the given tuple of inputs
  template <typename Input>
  std::enable_if_t<ElementCount<Input> >= 2, OnGenerateReturn<OutputOption>>
  invoke_on_generate(const rclcpp::Time &time, const rclcpp::Duration &period, const Input &input) {
    return std::apply(
        [&, this](auto &&...args) { return this->on_generate(time, period, args.get()...); },
        input);
  }

  // Call on_write(time, period, input) for the given single input
  template <typename Input>
  std::enable_if_t<ElementCount<Input> == 1, typename BaseCommon::ControllerReturn>
  invoke_on_write(const rclcpp::Time &time, const rclcpp::Duration &period, Input &&input) {
    return this->on_write(time, period, std::forward<decltype(input)>(input));
  }

  // Call on_write(time, period, input0, input1, ...) for the given tuple of inputs
  template <typename Input>
  std::enable_if_t<ElementCount<Input> >= 2, typename BaseCommon::ControllerReturn>
  invoke_on_write(const rclcpp::Time &time, const rclcpp::Duration &period, Input &&input) {
    return std::apply(
        [&, this](auto &&...args) {
          return this->on_write(time, period, std::forward<decltype(args)>(args)...);
        },
        std::forward<decltype(input)>(input));
  }
};

} // namespace ffmpeg_controllers

#endif