#ifndef FFMPEG_CONTROLLERS_DETAIL_INPUT_OUTPUT_MIXIN_HPP
#define FFMPEG_CONTROLLERS_DETAIL_INPUT_OUTPUT_MIXIN_HPP

#include <optional>
#include <tuple>

#include <controller_interface/controller_interface_base.hpp> // for controller_interface::return_type
#include <ffmpeg_controllers/detail/input_mixin.hpp>
#include <ffmpeg_controllers/detail/input_output_options.hpp>
#include <ffmpeg_controllers/detail/interface_adapter.hpp>
#include <ffmpeg_controllers/detail/output_mixin.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

namespace ffmpeg_controllers {

// =====================================================
// Mixin class depending on InputOption and OutputOption
// =====================================================

template <typename Inputs, typename Outputs> class OnGenerateDefinition;
template <typename... InputElements, typename... OutputElements>
class OnGenerateDefinition<std::tuple<InputElements...>, std::tuple<OutputElements...>> {
protected:
  using Outputs = std::tuple<OutputElements...>;

  virtual std::pair<controller_interface::return_type, std::optional<Outputs>>
  on_generate(const rclcpp::Time &time, const rclcpp::Duration &period,
              const InputElements &...inputs) = 0;
};

template <typename InputOption, typename OutputOption>
class InputOutputMixin
    : public InputMixin<InputOption, CommonInterfaceFor<InputOption, OutputOption>>,
      public OutputMixin<OutputOption, CommonInterfaceFor<InputOption, OutputOption>>,
      public OnGenerateDefinition<InputsFor<InputOption>, OutputsFor<OutputOption>> {
private:
  using Interface = CommonInterfaceFor<InputOption, OutputOption>;
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
    const auto [read_ret, generate_args] = BaseInput::on_read(time, period);
    if (read_ret != BaseCommon::ControllerReturn::OK || !generate_args) {
      return read_ret;
    }

    const auto on_generate_f = [&, this](auto &&...args) {
      return this->on_generate(time, period, args.get()...);
    };
    auto [generate_ret, write_args] = std::apply(on_generate_f, *generate_args);
    if (generate_ret != BaseCommon::ControllerReturn::OK || !write_args) {
      return generate_ret;
    }

    const auto on_write_f = [&, this](auto &&...args) {
      return BaseOutput::on_write(time, period, std::forward<decltype(args)>(args)...);
    };
    const auto write_ret = std::apply(on_write_f, std::move(*write_args));
    if (write_ret != BaseCommon::ControllerReturn::OK) {
      return write_ret;
    }

    return BaseCommon::ControllerReturn::OK;
  }
};

} // namespace ffmpeg_controllers

#endif