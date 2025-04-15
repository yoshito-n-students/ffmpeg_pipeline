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

Input::Input(const std::string &url, const std::string &iformat_name,
             const std::map<std::string, std::string> &option_map,
             const std::string &media_type_name)
    : iformat_ctx_(nullptr, &close_input), istream_id_(-1) {
  // Register all the input format types
  avdevice_register_all();

  // Allocate the input format context and set the non-blocking flag
  AVFormatContext *iformat_ctx = avformat_alloc_context();
  if (!iformat_ctx) {
    throw Error("Input::Input(): Failed to allocate AVFormatContext");
  }
  iformat_ctx->flags |= AVFMT_FLAG_NONBLOCK;

  // Find the input format by name
  const AVInputFormat *iformat =
      (iformat_name.empty() ? nullptr : av_find_input_format(iformat_name.c_str()));
  if (!iformat_name.empty() && iformat == nullptr) {
    avformat_free_context(iformat_ctx);
    throw Error("Input::Input(): " + iformat_name + " was not recognized as an input format");
  }

  // Build the option dictionary from the map
  AVDictionary *option_dict = nullptr;
  for (const auto &[key, value] : option_map) {
    if (const int ret = av_dict_set(&option_dict, key.c_str(), value.c_str(), 0); ret < 0) {
      avformat_free_context(iformat_ctx);
      av_dict_free(&option_dict);
      throw Error("Input::Input(): Failed to pack option [" + key + ", " + value + "]", ret);
    }
  }

  // Open the input with the URL, format and options.
  // If avformat_open_input() fails, it frees format_ctx.
  // So, we put format_ctx into unique_ptr after open succeeds.
  if (const int ret = avformat_open_input(&iformat_ctx, url.c_str(), iformat, &option_dict);
      ret < 0) {
    av_dict_free(&option_dict);
    throw Error("Input::Input(): Failed to open input " + url, ret);
  }
  iformat_ctx_.reset(iformat_ctx);

  // Check if the input accepts all the options
  if (option_dict) {
    AVDictionaryEntry *remaining_option = nullptr;
    av_dict_get(option_dict, "", remaining_option, AV_DICT_IGNORE_SUFFIX);
    const std::string msg = "Input::Input(): Input " + url + " does not accept option [" +
                            remaining_option->key + ", " + remaining_option->value + "]";
    av_dict_free(&option_dict);
    throw Error(msg);
  }

  // Retrieve stream information on the input
  // (TODO: check if this is necessary to find the stream)
  if (const int ret = avformat_find_stream_info(iformat_ctx, nullptr); ret < 0) {
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
  istream_id_ = av_find_best_stream(iformat_ctx, media_type, -1, -1, nullptr, 0);
  if (istream_id_ < 0) {
    throw Error("Input::Input(): Failed to find the best stream of media type " + media_type_name,
                istream_id_);
  }
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