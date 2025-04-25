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

Output::Output(const std::string &format_name, const std::string &url,
               const CodecParameters &codec_params, Dictionary *const options)
    : oformat_ctx_(nullptr, &close_output), ostream_(nullptr), increasing_dts_(0) {
  // Register all the input format types
  avdevice_register_all();

  // Allocate the output format context and set the non-blocking flag
  {
    AVFormatContext *oformat_ctx = nullptr;
    if (const int ret =
            avformat_alloc_output_context2(&oformat_ctx, nullptr, format_name.c_str(), url.c_str());
        ret < 0) {
      throw Error("Output::Output(): Failed to allocate AVFormatContext ([" + format_name + "] " +
                      url + ")",
                  ret);
    }
    oformat_ctx->flags |= AVFMT_FLAG_NONBLOCK;
    oformat_ctx_.reset(oformat_ctx);
  }

  // Create a new stream in the output format context
  ostream_ = avformat_new_stream(oformat_ctx_.get(), nullptr);
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
  if (!(oformat_ctx_->oformat->flags & AVFMT_NOFILE)) {
    if (const int ret = avio_open(&oformat_ctx_->pb, url.c_str(), AVIO_FLAG_WRITE); ret < 0) {
      throw Error("Output::Output(): Failed to open output file " + url, ret);
    }
  }

  // Write the header for the output stream.
  // avformat_write_header() may free the options,
  // so we release the ownership of it from unique_ptr during calling the function.
  {
    AVDictionary *options_ptr = options->release();
    const int ret = avformat_write_header(oformat_ctx_.get(), &options_ptr);
    *options = Dictionary(options_ptr);
    if (ret < 0) {
      throw Error("Output::Output(): Failed to write header", ret);
    }
  }

  // Check if the output accepts all the options
  if (!options->empty()) {
    throw Error("Output::Output(): Output " + url + " does not accept options [" +
                options->to_flow_style_yaml() + "]");
  }
}

std::string Output::format_name() const {
  return (oformat_ctx_ && oformat_ctx_->oformat && oformat_ctx_->oformat->name)
             ? oformat_ctx_->oformat->name
             : "";
}

std::string Output::url() const {
  return (oformat_ctx_ && oformat_ctx_->url) ? oformat_ctx_->url : "";
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

bool Output::write_uncoded_frame(const Frame &frame) {
  // Set the timestamps of the copied frame
  Frame output_frame(frame);
  output_frame->pts = output_frame->pkt_dts = increasing_dts_++;

  // Write the uncoded frame to the output stream
  if (const int ret =
          av_write_uncoded_frame(oformat_ctx_.get(), ostream_->index, output_frame.get());
      ret >= 0) {
    return true; // Successfully written
  } else if (ret == AVERROR(EAGAIN)) {
    return false; // The output device is not ready to accept more data
  } else {
    throw Error("Output::write_uncoded_frame(): Failed to write uncoded frame", ret);
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