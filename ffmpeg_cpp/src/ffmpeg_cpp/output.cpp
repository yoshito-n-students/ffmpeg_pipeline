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

Output::Output(const std::string &oformat_name, const std::string &filename,
               const CodecParameters &codec_params,
               const std::map<std::string, std::string> &option_map)
    : oformat_ctx_(nullptr, &close_output), ostream_(nullptr), increasing_dts_(0) {
  // Register all the input format types
  avdevice_register_all();

  // Allocate the output format context and set the non-blocking flag
  AVFormatContext *oformat_ctx = nullptr;
  if (const int ret = avformat_alloc_output_context2(&oformat_ctx, nullptr, oformat_name.c_str(),
                                                     filename.c_str());
      ret < 0) {
    throw Error("Output::Output(): Failed to allocate AVFormatContext (format: " + oformat_name +
                    ", filename: " + filename + ")",
                ret);
  }
  oformat_ctx->flags |= AVFMT_FLAG_NONBLOCK;
  oformat_ctx_.reset(oformat_ctx);

  // Create a new stream in the output format context
  ostream_ = avformat_new_stream(oformat_ctx, nullptr);
  if (!ostream_) {
    throw Error("Output::Output(): Failed to create new stream on AVFormatContext");
  }

  // Set the parameters for the output stream
  // TODO: set required parameters for video streams
  ostream_->time_base = AVRational{1, codec_params->sample_rate};
  if (const int ret = avcodec_parameters_copy(ostream_->codecpar, codec_params.get()); ret < 0) {
    throw Error("Output::Output(): Failed to set codec parameters for output stream", ret);
  }

  // Open the file for writing if the format requires it
  if (!(oformat_ctx->oformat->flags & AVFMT_NOFILE)) {
    if (const int ret = avio_open(&oformat_ctx->pb, filename.c_str(), AVIO_FLAG_WRITE); ret < 0) {
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
  if (const int ret = avformat_write_header(oformat_ctx, &option_dict); ret < 0) {
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

bool Output::write_frame(const Packet &packet) {
  // Create a copy (shallow copy if possible) of the packet
  // and modify the properties for the output stream
  Packet output_packet(packet);
  // stream_index must point to the output stream
  output_packet->stream_index = ostream_->index;
  // The packet given to av_write_frame() must have monotonically increasing dts and pts.
  // Even if the packet is temporarily failed to be sent to av_write_frame() and is resent,
  // dts and pts must continue to increase.
  // To guarantee this, overwrite the dts and pts of the copied packet.
  output_packet->pts = output_packet->dts = increasing_dts_++;

  // Write the packet to the output stream
  if (const int ret = av_write_frame(oformat_ctx_.get(), output_packet.get()); ret >= 0) {
    return true; // Successfully written
  } else if (ret == AVERROR(EAGAIN)) {
    return false; // The output device is not ready to accept more data
  } else {
    throw Error("Output::write_frame(): Failed to write frame", ret);
  }
}

void Output::close_output(AVFormatContext *oformat_ctx) {
  av_write_trailer(oformat_ctx);
  if (!(oformat_ctx->oformat->flags & AVFMT_NOFILE)) {
    avio_closep(&oformat_ctx->pb);
  }
  avformat_free_context(oformat_ctx);
}

} // namespace ffmpeg_cpp