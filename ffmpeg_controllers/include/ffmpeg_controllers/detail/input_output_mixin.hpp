#ifndef FFMPEG_CONTROLLERS_DETAIL_INPUT_OUTPUT_MIXIN_HPP
#define FFMPEG_CONTROLLERS_DETAIL_INPUT_OUTPUT_MIXIN_HPP

#include <optional>
#include <tuple>
#include <type_traits>

#include <ffmpeg_controllers/detail/controller_traits.hpp>
#include <ffmpeg_controllers/detail/input_mixin.hpp>
#include <ffmpeg_controllers/detail/output_mixin.hpp>
#include <ffmpeg_controllers/detail/utility.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace ffmpeg_controllers {

// ================================================================================
// Helper class to define the pure virtual function InputOutputMixin::on_generate()
// which is used to generate the output from the input
// ================================================================================

// Return type of InputOutputMixin::on_generate()
template <typename OutputOption> struct GetOnGenerateReturn {
  using Result = std::pair<ControllerReturn, std::optional<OutputFor<OutputOption>>>;
};
template <typename... OutputOptions> struct GetOnGenerateReturn<std::tuple<OutputOptions...>> {
  using Result =
      std::pair<ControllerReturn, std::optional<std::tuple<OutputFor<OutputOptions>...>>>;
};
template <typename OutputOption>
using OnGenerateReturn = typename GetOnGenerateReturn<OutputOption>::Result;

// Definition of InputOutputMixin::on_generate()
template <typename InputOption, typename OutputOption> class OnGenerateContract {
protected:
  // Default version
  virtual OnGenerateReturn<OutputOption> on_generate(const rclcpp::Time &time,
                                                     const rclcpp::Duration &period,
                                                     const InputFor<InputOption> &input) = 0;
};
template <typename... InputOptions, typename OutputOption>
class OnGenerateContract<std::tuple<InputOptions...>, OutputOption> {
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
class InputOutputMixin
    : public InputMixin<InputOption, ControllerInterfaceFor<InputOption, OutputOption>>,
      public OutputMixin<OutputOption, ControllerInterfaceFor<InputOption, OutputOption>>,
      public OnGenerateContract<InputOption, OutputOption> {
private:
  using ControllerIface = ControllerInterfaceFor<InputOption, OutputOption>;
  using BaseInput = InputMixin<InputOption, ControllerIface>;
  using BaseOutput = OutputMixin<OutputOption, ControllerIface>;

protected:
  NodeReturn on_init() override { return merge({BaseInput::on_init(), BaseOutput::on_init()}); }

  NodeReturn on_activate(const rclcpp_lifecycle::State &previous_state) override {
    return merge({BaseInput::on_activate(previous_state), BaseOutput::on_activate(previous_state)});
  }

  ControllerReturn on_update(const rclcpp::Time &time, const rclcpp::Duration &period) override {
    const auto [read_ret, generate_args] = this->on_read(time, period, InputOption());
    if (read_ret != ControllerReturn::OK || !generate_args) {
      return read_ret;
    }

    auto [generate_ret, write_args] =
        invoke(&InputOutputMixin::on_generate, time, period, *generate_args);
    if (generate_ret != ControllerReturn::OK || !write_args) {
      return generate_ret;
    }

    const auto write_ret =
        invoke(&InputOutputMixin::on_write, time, period, std::move(*write_args));
    if (write_ret != ControllerReturn::OK) {
      return write_ret;
    }

    return ControllerReturn::OK;
  }

  // Invoke the given method of this class with the given arguments.
  // If the given arguments are not compatible with the method,
  // it will be unpacked and passed to the method by std::apply().
  template <class Method, typename Arg>
  auto invoke(Method &&method, const rclcpp::Time &time, const rclcpp::Duration &period,
              Arg &&arg) {
    if constexpr (std::is_invocable_v<Method, decltype(this), decltype(time), decltype(period),
                                      Arg>) {
      return (this->*method)(time, period, std::forward<Arg>(arg));
    } else {
      return std::apply(
          [&, this](auto &&...args) {
            return (this->*method)(time, period, std::forward<decltype(args)>(args)...);
          },
          std::forward<Arg>(arg));
    }
  }
};

} // namespace ffmpeg_controllers

#endif