extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

#include "internal.hpp"

namespace ffmpeg_cpp {

// ========================================
// Input - RAII wrapper for AVFormatContext
// ========================================

Input Input::null() { return Input(nullptr); }

Input Input::create(const std::string &url, const std::string &format_name,
                    const Dictionary &options, const std::string &media_type_name) {
  // Register all the input format types
  avdevice_register_all();

  // Allocate the input format context and set the non-blocking flag
  Input input(avformat_alloc_context());
  if (!input) {
    throw Error("Input::Input(): Failed to allocate AVFormatContext");
  }
  input->flags |= AVFMT_FLAG_NONBLOCK;

  // Find the input format by name
  const AVInputFormat *iformat =
      (format_name.empty() ? nullptr : av_find_input_format(format_name.c_str()));
  if (!format_name.empty() && iformat == nullptr) {
    throw Error("Input::Input(): " + format_name + " was not recognized as an input format");
  }

  // Open the input with the URL, format and options.
  // avformat_open_input() may free the context and options,
  // so we release the ownership of them from unique_ptr during calling it.
  {
    AVFormatContext *iformat_ctx = input.release();
    Dictionary writable_options = options;
    AVDictionary *writable_options_ptr = writable_options.release();
    const int ret = avformat_open_input(&iformat_ctx, url.c_str(), iformat, &writable_options_ptr);
    input.reset(iformat_ctx);
    writable_options.reset(writable_options_ptr);
    if (ret < 0) {
      throw Error("Input::Input(): Failed to open input " + url, ret);
    }
    if (writable_options) {
      throw Error("Input::Input(): Input " + url + " does not accept option [" +
                  writable_options.to_flow_style_yaml() + "]");
    }
  }

  // Retrieve stream information on the input
  if (const int ret = avformat_find_stream_info(input.get(), nullptr); ret < 0) {
    throw Error("Input::Input(): Failed to find stream information", ret);
  }

  // Identify the media type from the name
  const AVMediaType media_type = to_media_type(media_type_name);
  if (media_type == AVMEDIA_TYPE_UNKNOWN) {
    throw Error("Input::Input(): " + media_type_name + " was not recognized as a media type");
  }

  // Find the best stream of the given media type
  input.istream_id_ = av_find_best_stream(input.get(), media_type, -1, -1, nullptr, 0);
  if (input.istream_id_ < 0) {
    throw Error("Input::Input(): Failed to find the best stream of media type " + media_type_name,
                input.istream_id_);
  }

  return input;
}

CodecParameters Input::codec_parameters() const {
  CodecParameters params = CodecParameters::create();
  if (const int ret = avcodec_parameters_copy(params.get(), get()->streams[istream_id_]->codecpar);
      ret < 0) {
    throw Error("Input::codec_parameters(): Failed to copy codec parameters", ret);
  }
  return params;
}

Packet Input::read_frame() {
  while (true) {
    Packet packet = Packet::create();
    if (const int ret = av_read_frame(get(), packet.get());
        (ret >= 0 || ret == AVERROR(EAGAIN)) && packet->stream_index == istream_id_) {
      return packet;
    } else if (ret < 0) {
      throw Error("Input::read_frame(): Failed to read frame", ret);
    }
  }
}

} // namespace ffmpeg_cpp