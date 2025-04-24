#ifndef FFMPEG_CONTROLLERS_CONTROLLER_BASE_HPP
#define FFMPEG_CONTROLLERS_CONTROLLER_BASE_HPP

#include <ffmpeg_controllers/detail/input_output_mixin.hpp>
// This makes options available in files that include this header
#include <ffmpeg_controllers/detail/input_output_options.hpp>

namespace ffmpeg_controllers {

// Base type for controllers built with mixin classes
template <typename InputOption, typename OutputOption>
using ControllerBase = InputOutputMixin<InputOption, OutputOption>;

} // namespace ffmpeg_controllers

#endif