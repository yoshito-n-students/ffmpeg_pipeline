extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_cpp {

// ========================================
// Input - RAII wrapper for AVFormatContext
// ========================================

Input::Input(const std::string &url, const std::string &format_name, Dictionary *const options,
             const std::string &media_type_name)
    : iformat_ctx_(nullptr, &close_input), istream_id_(-1) {
  // Register all the input format types
  avdevice_register_all();

  // Allocate the input format context and set the non-blocking flag
  if (AVFormatContext *const iformat_ctx = avformat_alloc_context(); iformat_ctx) {
    iformat_ctx->flags |= AVFMT_FLAG_NONBLOCK;
    iformat_ctx_.reset(iformat_ctx);
  } else {
    throw Error("Input::Input(): Failed to allocate AVFormatContext");
  }

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
    AVFormatContext *iformat_ctx = iformat_ctx_.release();
    AVDictionary *options_ptr = options->release();
    const int ret = avformat_open_input(&iformat_ctx, url.c_str(), iformat, &options_ptr);
    iformat_ctx_.reset(iformat_ctx);
    *options = Dictionary(options_ptr);
    if (ret < 0) {
      throw Error("Input::Input(): Failed to open input " + url, ret);
    }
  }

  // Check if the input accepts all the options
  if (!options->empty()) {
    throw Error("Input::Input(): Input " + url + " does not accept option [" +
                options->to_flow_style_yaml() + "]");
  }

  // Retrieve stream information on the input
  if (const int ret = avformat_find_stream_info(iformat_ctx_.get(), nullptr); ret < 0) {
    throw Error("Input::Input(): Failed to find stream information", ret);
  }

  // Identify the media type from the name
  const AVMediaType media_type = [](const std::string &name) {
    if (name == av_get_media_type_string(AVMEDIA_TYPE_VIDEO)) {
      return AVMEDIA_TYPE_VIDEO;
    } else if (name == av_get_media_type_string(AVMEDIA_TYPE_AUDIO)) {
      return AVMEDIA_TYPE_AUDIO;
    } else if (name == av_get_media_type_string(AVMEDIA_TYPE_DATA)) {
      return AVMEDIA_TYPE_DATA;
    } else if (name == av_get_media_type_string(AVMEDIA_TYPE_SUBTITLE)) {
      return AVMEDIA_TYPE_SUBTITLE;
    } else if (name == av_get_media_type_string(AVMEDIA_TYPE_ATTACHMENT)) {
      return AVMEDIA_TYPE_ATTACHMENT;
    } else if (name == av_get_media_type_string(AVMEDIA_TYPE_NB)) {
      return AVMEDIA_TYPE_NB;
    } else {
      return AVMEDIA_TYPE_UNKNOWN;
    }
  }(media_type_name);
  if (media_type == AVMEDIA_TYPE_UNKNOWN) {
    throw Error("Input::Input(): " + media_type_name + " was not recognized as a media type");
  }

  // Find the best stream of the given media type
  istream_id_ = av_find_best_stream(iformat_ctx_.get(), media_type, -1, -1, nullptr, 0);
  if (istream_id_ < 0) {
    throw Error("Input::Input(): Failed to find the best stream of media type " + media_type_name,
                istream_id_);
  }
}

std::string Input::format_name() const {
  return (iformat_ctx_ && iformat_ctx_->iformat && iformat_ctx_->iformat->name)
             ? iformat_ctx_->iformat->name
             : "";
}

std::string Input::url() const {
  return (iformat_ctx_ && iformat_ctx_->url) ? iformat_ctx_->url : "";
}

CodecParameters Input::codec_parameters() const {
  CodecParameters params;
  if (const int ret =
          avcodec_parameters_copy(params.get(), iformat_ctx_->streams[istream_id_]->codecpar);
      ret < 0) {
    throw Error("Input::codec_parameters(): Failed to copy codec parameters", ret);
  }
  return params;
}

Packet Input::read_frame() {
  while (true) {
    Packet packet;
    if (const int ret = av_read_frame(iformat_ctx_.get(), packet.get());
        (ret >= 0 || ret == AVERROR(EAGAIN)) && packet->stream_index == istream_id_) {
      return packet;
    } else if (ret < 0) {
      throw Error("Input::read_frame(): Failed to read frame", ret);
    }
  }
}

void Input::close_input(AVFormatContext *iformat_ctx) { avformat_close_input(&iformat_ctx); }

} // namespace ffmpeg_cpp