extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_cpp {

// =========================================
// Output - RAII wrapper for AVFormatContext
// =========================================

Output::Output(const std::string &format_name, const std::string &filename,
               const CodecParameters &codec_params,
               const std::map<std::string, std::string> &option_map)
    : format_ctx_(nullptr, &close_output) {
  // Register all the input format types
  avdevice_register_all();

  // Allocate the format context and set the non-blocking flag
  AVFormatContext *format_ctx = nullptr;
  if (const int ret = avformat_alloc_output_context2(&format_ctx, nullptr, format_name.c_str(),
                                                     filename.c_str());
      ret < 0) {
    throw Error("Output::Output(): Failed to allocate AVFormatContext (format: " + format_name +
                    ", filename: " + filename + ")",
                ret);
  }
  format_ctx_.reset(format_ctx);

  // Create a new stream in the format context
  AVStream *const stream = avformat_new_stream(format_ctx, nullptr);
  if (!stream) {
    throw Error("Output::Output(): Failed to create new stream on AVFormatContext");
  }

  // Set the parameters for the stream
  switch (codec_params->codec_type) {
  case AVMEDIA_TYPE_AUDIO:
    stream->time_base = AVRational{1, codec_params->sample_rate};
    break;
  case AVMEDIA_TYPE_VIDEO:
    // TODO: set required parameters for video streams
    break;
  default:
    break;
  }
  if (const int ret = avcodec_parameters_copy(stream->codecpar, codec_params.get()); ret < 0) {
    throw Error("Output::Output(): Failed to set codec parameters for output stream", ret);
  }

  // Open the file for writing if the format requires it
  if (!(format_ctx->oformat->flags & AVFMT_NOFILE)) {
    if (const int ret = avio_open(&format_ctx->pb, filename.c_str(), AVIO_FLAG_WRITE); ret < 0) {
      throw Error("Output::Output(): Failed to open output file " + filename, ret);
    }
  }

  // Build the option dictionary from the map
  AVDictionary *option_dict = nullptr;
  for (const auto &[key, value] : option_map) {
    if (const int ret = av_dict_set(&option_dict, key.c_str(), value.c_str(), 0); ret < 0) {
      av_dict_free(&option_dict);
      throw Error("Output::Output(): Failed to pack option [" + key + ", " + value + "]", ret);
    }
  }

  // Write the header for the output stream
  if (const int ret = avformat_write_header(format_ctx, &option_dict); ret < 0) {
    throw Error("Output::Output(): Failed to write header", ret);
  }

  // Check if the input accepts all the options
  if (option_dict) {
    AVDictionaryEntry *remaining_option = nullptr;
    av_dict_get(option_dict, "", remaining_option, AV_DICT_IGNORE_SUFFIX);
    const std::string msg = "Output::Ourput(): Output " + filename + " does not accept option [" +
                            remaining_option->key + ", " + remaining_option->value + "]";
    av_dict_free(&option_dict);
    throw Error(msg);
  }
}

void Output::write_frame(Packet *const packet) {
  if (const int ret = av_write_frame(format_ctx_.get(), packet->get()); ret < 0) {
    throw Error("Output::write_frame(): Failed to write frame", ret);
  }
}

void Output::close_output(AVFormatContext *format_ctx) {
  av_write_trailer(format_ctx);
  if (!(format_ctx->oformat->flags & AVFMT_NOFILE)) {
    avio_closep(&format_ctx->pb);
  }
  avformat_free_context(format_ctx);
}

} // namespace ffmpeg_cpp