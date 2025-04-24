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

struct ReadFrame {}; // Read state interface loaned from other hardware or controller
struct ReadPacket {};
struct ReadPacketWithParams {};
template <typename Message> struct Subscribe {}; // Subscribe a topic

} // namespace input_options

namespace output_options {

struct ExportFrame {}; // Export state interface to other hardware or controller
struct ExportPacket {};
struct ExportPacketWithParams {};
struct WritePacket {}; // Write to command interface loaned from other hardware or controller
template <typename Message> struct Publish {}; // Publish messages to a topic

} // namespace output_options

// =====================================
// Helper metafunctions for std::tuple<>
// =====================================

// Add std::reference_wrapper<const T> to each element of std::tuple<T0, T1, ...>
template <typename T> struct AddCRefToElements;
template <typename... Ts> struct AddCRefToElements<std::tuple<Ts...>> {
  using Result = std::tuple<std::reference_wrapper<const Ts>...>;
};
template <typename T> using ElementwiseCRef = typename AddCRefToElements<T>::Result;

// Find the first type in std::tuple<T0, T1, ...> that is also in std::tuple<U0, U1, ...>
template <typename T, typename U> struct FindFirstCommonElement;
template <typename... Us> struct FindFirstCommonElement<std::tuple<>, std::tuple<Us...>> {
  using Result = void;
};
template <typename T0, typename... Ts, typename... Us>
struct FindFirstCommonElement<std::tuple<T0, Ts...>, std::tuple<Us...>> {
  using Result = typename std::conditional_t<
      std::disjunction_v<std::is_same<T0, Us>...>, T0,
      typename FindFirstCommonElement<std::tuple<Ts...>, std::tuple<Us...>>::Result>;
};
template <typename T, typename U>
using FirstCommonElement = typename FindFirstCommonElement<T, U>::Result;

// ===============================
// Traits for input/output options
// ===============================

// Tuple of input types for the given input option
template <typename InputOption> struct GetInputsFor;
template <> struct GetInputsFor<input_options::ReadFrame> {
  using Result = std::tuple<ffmpeg_cpp::Frame>;
};
template <> struct GetInputsFor<input_options::ReadPacket> {
  using Result = std::tuple<ffmpeg_cpp::Packet>;
};
template <> struct GetInputsFor<input_options::ReadPacketWithParams> {
  using Result = std::tuple<ffmpeg_cpp::Packet, ffmpeg_cpp::CodecParameters>;
};
template <typename Message> struct GetInputsFor<input_options::Subscribe<Message>> {
  using Result = std::tuple<Message>;
};
template <typename InputOption> using InputsFor = typename GetInputsFor<InputOption>::Result;

// Add std::reference_wrapper<const T> to each element of InputsFor<InputOption>
template <typename InputOption> using CRefInputsFor = ElementwiseCRef<InputsFor<InputOption>>;

// Tuple of output types for the given output option
template <typename OutputOption> struct GetOutputsFor;
template <> struct GetOutputsFor<output_options::ExportFrame> {
  using Result = std::tuple<ffmpeg_cpp::Frame>;
};
template <> struct GetOutputsFor<output_options::ExportPacket> {
  using Result = std::tuple<ffmpeg_cpp::Packet>;
};
template <> struct GetOutputsFor<output_options::ExportPacketWithParams> {
  using Result = std::tuple<ffmpeg_cpp::Packet, ffmpeg_cpp::CodecParameters>;
};
template <> struct GetOutputsFor<output_options::WritePacket> {
  using Result = std::tuple<ffmpeg_cpp::Packet>;
};
template <typename Message> struct GetOutputsFor<output_options::Publish<Message>> {
  using Result = std::tuple<Message>;
};
template <typename OutputOption> using OutputsFor = typename GetOutputsFor<OutputOption>::Result;

// Supported interface types for the given input/output option
template <typename Option>
using InterfacesFor =
    std::conditional_t<(std::is_same_v<Option, output_options::ExportFrame> ||
                        std::is_same_v<Option, output_options::ExportPacket> ||
                        std::is_same_v<Option, output_options::ExportPacketWithParams>),
                       std::tuple<controller_interface::ChainableControllerInterface>,
                       std::tuple<controller_interface::ControllerInterface,
                                  controller_interface::ChainableControllerInterface>>;

// First common interface types that the given input and output options support
template <typename InputOption, typename OutputOption>
using CommonInterfaceFor =
    FirstCommonElement<InterfacesFor<InputOption>, InterfacesFor<OutputOption>>;

} // namespace ffmpeg_controllers

#endif