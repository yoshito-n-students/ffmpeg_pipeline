#include <set>

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
    throw Error("Input::create(): Failed to allocate AVFormatContext");
  }
  input->flags |= AVFMT_FLAG_NONBLOCK;

  // Find the input format by name
  const AVInputFormat *iformat =
      (format_name.empty() ? nullptr : av_find_input_format(format_name.c_str()));
  if (!format_name.empty() && iformat == nullptr) {
    throw Error("Input::create(): " + format_name + " was not recognized as an input format");
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
      throw Error("Input::create(): Failed to open input " + url, ret);
    }
    if (writable_options) {
      throw Error("Input::create(): Input " + url + " does not accept option [" +
                  writable_options.to_flow_style_yaml() + "]");
    }
  }

  // Retrieve stream information on the input
  if (const int ret = avformat_find_stream_info(input.get(), nullptr); ret < 0) {
    throw Error("Input::create(): Failed to find stream information", ret);
  }

  // Determine the media type to pass to av_find_best_stream()
  AVMediaType media_type;
  if (!media_type_name.empty()) {
    // Use the given media type name if provided
    media_type = to_media_type(media_type_name);
    if (media_type == AVMEDIA_TYPE_UNKNOWN) {
      throw Error("Input::create(): " + media_type_name + " was not recognized as a media type");
    }
  } else {
    // Otherwise, use the common media type of all streams
    std::set<AVMediaType> media_types;
    for (unsigned int i = 0; i < input->nb_streams; ++i) {
      if (input->streams[i]->codecpar) {
        media_types.insert(input->streams[i]->codecpar->codec_type);
      }
    }
    if (media_types.size() != 1) {
      throw Error("Input::create(): No common media type among all streams in the input. "
                  "media_type_name must be specified to select a stream.");
    }
    media_type = *media_types.begin();
  }

  // Find the best stream of the determined media type
  const int istream_id = av_find_best_stream(input.get(), media_type, -1, -1, nullptr, 0);
  if (istream_id < 0) {
    throw Error("Input::create(): Failed to find the best stream of media type " + media_type_name,
                istream_id);
  }
  input.istream_ = input->streams[istream_id];

  return input;
}

CodecParameters Input::codec_parameters() const {
  if (istream_ && istream_->codecpar) {
    CodecParameters params = CodecParameters::create();
    if (const int ret = avcodec_parameters_copy(params.get(), istream_->codecpar); ret < 0) {
      throw Error("Input::codec_parameters(): Failed to copy codec parameters", ret);
    }
    return params;
  }
  return CodecParameters::null();
}

Packet Input::read_frame() {
  while (true) {
    Packet packet = Packet::create();
    if (const int ret = av_read_frame(get(), packet.get());
        ret >= 0 && packet->stream_index == istream_->index) {
      // Successfully read a frame from the stream of interest
      return packet;
    } else if (ret == AVERROR(EAGAIN) && packet->stream_index == istream_->index) {
      // No frame is temporarily available for the stream of interest
      return Packet::null();
    } else if (ret < 0) {
      // Error during reading
      throw Error("Input::read_frame(): Failed to read frame", ret);
    }
    // Otherwise, continue reading until we find a frame from the stream of interest
  }
}

} // namespace ffmpeg_cpp