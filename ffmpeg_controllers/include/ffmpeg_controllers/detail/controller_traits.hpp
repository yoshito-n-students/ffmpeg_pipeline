#ifndef FFMPEG_CONTROLLERS_DETAIL_CONTROLLER_TRAITS_HPP
#define FFMPEG_CONTROLLERS_DETAIL_CONTROLLER_TRAITS_HPP

#include <tuple>
#include <type_traits>

#include <controller_interface/chainable_controller_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_controllers/controller_options.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_controllers {

// =============================
// Traits for controller options
// =============================

// Object type to be read or subscribed for the given input option
template <typename InputOption> struct GetInputFor;
template <typename Object> struct GetInputFor<input_options::Read<Object>> {
  using Result = Object;
};
template <typename Message> struct GetInputFor<input_options::Subscribe<Message>> {
  using Result = Message;
};
template <typename InputOption> using InputFor = typename GetInputFor<InputOption>::Result;

// Object type to be exported, written or published for the given output option
template <typename OutputOption> struct GetOutputFor;
template <typename Object> struct GetOutputFor<output_options::Export<Object>> {
  using Result = Object;
};
template <typename Object> struct GetOutputFor<output_options::Write<Object>> {
  using Result = Object;
};
template <typename Message> struct GetOutputFor<output_options::Publish<Message>> {
  using Result = Message;
};
template <typename OutputOption> using OutputFor = typename GetOutputFor<OutputOption>::Result;

// Hardware interface name for the given object type
template <typename Object> struct GetHardwareInterfaceName;
template <> struct GetHardwareInterfaceName<ffmpeg_cpp::Frame> {
  static constexpr const char *Value = "frame";
};
template <> struct GetHardwareInterfaceName<ffmpeg_cpp::Packet> {
  static constexpr const char *Value = "packet";
};
template <> struct GetHardwareInterfaceName<ffmpeg_cpp::CodecParameters> {
  static constexpr const char *Value = "codec_parameters";
};
template <typename Object>
inline constexpr const char *HardwareInterfaceName = GetHardwareInterfaceName<Object>::Value;

// Supported controller interface type for the given input and output options
template <typename InputOption, typename OutputOption> struct GetControllerInterfaceFor {
  using Result = controller_interface::ControllerInterface;
};
template <typename InputOption, typename Object>
struct GetControllerInterfaceFor<InputOption, output_options::Export<Object>> {
  using Result = controller_interface::ChainableControllerInterface;
};
template <typename InputOption, typename... OutputOptions>
struct GetControllerInterfaceFor<InputOption, std::tuple<OutputOptions...>> {
  using Result =
      std::conditional_t<std::conjunction_v<std::is_same<
                             typename GetControllerInterfaceFor<InputOption, OutputOptions>::Result,
                             controller_interface::ControllerInterface>...>,
                         controller_interface::ControllerInterface,
                         controller_interface::ChainableControllerInterface>;
};
template <typename InputOption, typename OutputOption>
using ControllerInterfaceFor =
    typename GetControllerInterfaceFor<InputOption, OutputOption>::Result;

} // namespace ffmpeg_controllers

#endif