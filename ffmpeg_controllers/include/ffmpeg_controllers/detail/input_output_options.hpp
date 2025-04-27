#ifndef FFMPEG_CONTROLLERS_DETAIL_INPUT_OUTPUT_OPTIONS_HPP
#define FFMPEG_CONTROLLERS_DETAIL_INPUT_OUTPUT_OPTIONS_HPP

#include <functional> // for std::reference_wrapper<>
#include <tuple>
#include <type_traits>

#include <controller_interface/chainable_controller_interface.hpp>
#include <controller_interface/controller_interface.hpp>
#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_controllers {

// =========================================
// Input/Output options for ControllerBase<>
// =========================================

namespace input_options {

// Read state interface loaned from other hardware or controller
template <typename Object> struct Read {};
// Subscribe a topic
template <typename Message> struct Subscribe {};

} // namespace input_options

namespace output_options {

// Export state interface to other hardware or controller
template <typename Object> struct Export {};
// Write to command interface loaned from other hardware or controller
template <typename Object> struct Write {};
// Publish messages to a topic
template <typename Message> struct Publish {};

} // namespace output_options

// ===============================
// Traits for input/output options
// ===============================

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

// Supported interface type for the given input and output options
template <typename InputOption, typename OutputOption> struct GetInterfaceFor {
  using Result = controller_interface::ControllerInterface;
};
template <typename InputOption, typename Object>
struct GetInterfaceFor<InputOption, output_options::Export<Object>> {
  using Result = controller_interface::ChainableControllerInterface;
};
template <typename InputOption, typename... OutputOptions>
struct GetInterfaceFor<InputOption, std::tuple<OutputOptions...>> {
  using Result = std::conditional_t<
      std::disjunction_v<std::is_same<typename GetInterfaceFor<InputOption, OutputOptions>::Result,
                                      controller_interface::ChainableControllerInterface>...>,
      controller_interface::ChainableControllerInterface,
      controller_interface::ControllerInterface>;
};
template <typename InputOption, typename OutputOption>
using InterfaceFor = typename GetInterfaceFor<InputOption, OutputOption>::Result;

} // namespace ffmpeg_controllers

#endif