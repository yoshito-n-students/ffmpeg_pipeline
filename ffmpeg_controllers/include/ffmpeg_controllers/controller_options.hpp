#ifndef FFMPEG_CONTROLLERS_CONTROLLER_OPTIONS_HPP
#define FFMPEG_CONTROLLERS_CONTROLLER_OPTIONS_HPP

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

} // namespace ffmpeg_controllers

#endif