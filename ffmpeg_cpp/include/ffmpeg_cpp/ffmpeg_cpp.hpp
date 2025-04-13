#ifndef FFMPEG_CPP_FFMPEG_CPP_HPP
#define FFMPEG_CPP_FFMPEG_CPP_HPP

#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}

namespace ffmpeg_cpp {

// =================
// Utility functions
// =================

// Convert a ffmpeg's error number to a string
std::string err2str(const int errnum);
// Convert a ffmpeg pixel format name to a ROS image encoding, or an empty string if not found
std::string to_ros_image_encoding(const std::string &ffmpeg_format_name);
// Convert a ROS image encoding to a ffmpeg pixel format name, or an empty string if not found
std::string to_ffmpeg_format_name(const std::string &ros_image_encoding);

// ==================================================
// Error class with an optional ffmpeg's error number
// ==================================================

class Error : public std::runtime_error {
public:
  Error(const std::string &msg) : std::runtime_error(msg) {}
  Error(const std::string &msg, const int errnum)
      : std::runtime_error(msg + ": " + err2str(errnum)) {}
};

// ============================
// RAII wrapper for AVBufferRef
// ============================

class BufferRef {
public:
  // Construct a BufferRef by copying the given data and adding padding
  BufferRef(const std::uint8_t *const data, const std::size_t unpadded_size);
  // Construct a BufferRef by referencing an existing buffer
  BufferRef(const BufferRef &buf);

  // Get the size of the buffer, with or without padding
  std::size_t padded_size() const { return buf_->size; }
  std::size_t unpadded_size() const { return buf_->size - AV_INPUT_BUFFER_PADDING_SIZE; }

  // Access to the underlying AVBufferRef
  AVBufferRef *get() { return buf_.get(); }
  const AVBufferRef *get() const { return buf_.get(); }
  AVBufferRef *operator->() { return buf_.operator->(); }
  const AVBufferRef *operator->() const { return buf_.operator->(); }

private:
  static void unref_buffer(AVBufferRef *buf);

private:
  std::unique_ptr<AVBufferRef, decltype(&unref_buffer)> buf_;
};

// =========================
// RAII wrapper for AVPacket
// =========================

class Packet {
public:
  // Allocate the packet and default the fields
  Packet();
  // Create a packet by referencing the given buffer
  Packet(const BufferRef &buf);

  // True if the packet data is empty or invalid
  bool empty() const { return !packet_ || !packet_->data || packet_->size == 0; }

  // Access to the underlying AVPacket
  AVPacket *get() { return packet_.get(); }
  const AVPacket *get() const { return packet_.get(); }
  AVPacket *operator->() { return packet_.operator->(); }
  const AVPacket *operator->() const { return packet_.operator->(); }

private:
  static void free_packet(AVPacket *packet);

private:
  std::unique_ptr<AVPacket, decltype(&free_packet)> packet_;
};

// ========================
// RAII wrapper for AVFrame
// ========================

class Frame {
public:
  // Allocate the frame and default the fields
  Frame();

  // True if the packet data is empty or invalid
  bool empty() const { return !frame_ || !frame_->data[0]; }

  std::string format_name() const;

  // True if the frame is on a hardware memory, which is not accessible by the CPU
  bool is_hw_frame() const { return frame_->hw_frames_ctx != nullptr; }

  // Copy the frame to the CPU-accessible memory
  Frame transfer_data() const;

  // Access to the underlying AVFrame
  AVFrame *get() { return frame_.get(); }
  const AVFrame *get() const { return frame_.get(); }
  AVFrame *operator->() { return frame_.operator->(); }
  const AVFrame *operator->() const { return frame_.operator->(); }

private:
  static void free_frame(AVFrame *frame);

private:
  std::unique_ptr<AVFrame, decltype(&free_frame)> frame_;
};

// ==================================
// RAII wrapper for AVCodecParameters
// ==================================

class CodecParameters {
public:
  // Allocate the codec parameters and default the fields
  CodecParameters();

  std::string codec_type_name() const;
  std::string codec_name() const;
  std::string format_name() const;

  // Access to the underlying AVPacket
  AVCodecParameters *get() { return params_.get(); }
  const AVCodecParameters *get() const { return params_.get(); }
  AVCodecParameters *operator->() { return params_.operator->(); }
  const AVCodecParameters *operator->() const { return params_.operator->(); }

private:
  static void free_parameters(AVCodecParameters *params);

private:
  std::unique_ptr<AVCodecParameters, decltype(&free_parameters)> params_;
};

// ======================================================
// RAII wrapper for input device (a.k.a. AVFormatContext)
// ======================================================

class Input {
public:
  // Construct without underlying AVFormatContext
  Input() : format_ctx_(nullptr, &close_input), stream_id_(-1) {}
  // Open the input device by avformat_open_input() with the given URL, format name, and options,
  // and find the best stream of the given media type
  Input(const std::string &url, const std::string &format_name,
        const std::map<std::string, std::string> &option_map, const std::string &media_type_name);

  std::string codec_name() const;
  CodecParameters codec_parameters() const;

  // Get a frame from the stream of interest in a NON-BLOCKING way.
  // If the next frame is not available for some temporary reason, return an empty frame.
  Packet read_frame();

  // Access to the underlying AVFormatContext
  bool valid() const { return format_ctx_.get(); }
  AVFormatContext *get() { return format_ctx_.get(); }
  const AVFormatContext *get() const { return format_ctx_.get(); }
  AVFormatContext *operator->() { return format_ctx_.operator->(); }
  const AVFormatContext *operator->() const { return format_ctx_.operator->(); }

private:
  static void close_input(AVFormatContext *format_ctx);

private:
  std::unique_ptr<AVFormatContext, decltype(&close_input)> format_ctx_;
  int stream_id_;
};

// ===============================================
// RAII wrapper for decoder (a.k.a AVCodecContext)
// ===============================================

class Decoder {
public:
  // Construct without underlying AVCodecContext
  Decoder() : codec_ctx_(nullptr, &free_context) {}
  // Allocate the codec context for the given codec name.
  // Parameters can be filled with Parser::parse().
  Decoder(const std::string &codec_name);
  // Allocate the codec context for the given codec parameters
  Decoder(const CodecParameters &params);

  std::string codec_name() const;
  std::string hw_device_type() const;

  // Send a cmpressed packet to the decoder
  void send_packet(const Packet &packet);

  // Receive a decoded frame from the decoder.
  // The frame may be empty if no frame is available.
  Frame receive_frame();

  // Access to the underlying AVCodecContext
  bool valid() const { return codec_ctx_.get(); }
  AVCodecContext *get() { return codec_ctx_.get(); }
  const AVCodecContext *get() const { return codec_ctx_.get(); }
  AVCodecContext *operator->() { return codec_ctx_.operator->(); }
  const AVCodecContext *operator->() const { return codec_ctx_.operator->(); }

private:
  static void free_context(AVCodecContext *codec_ctx);

private:
  std::unique_ptr<AVCodecContext, decltype(&free_context)> codec_ctx_;
};

// ================================
// RAII wrapper for AVParserContext
// ================================

class Parser {
public:
  // Construct without underlying AVCodecParserContext
  Parser() : parser_ctx_(nullptr, &av_parser_close) {}
  // Allocate the parser context for the given codec name
  Parser(const std::string &codec_name);

  std::vector<std::string> codec_names() const;

  // Parse the given buffer and advance the data pointer by the number of bytes parsed.
  // If a packet is found, return it; otherwise, return an empty packet.
  // The decoder may be tuned based on the parsing result.
  Packet parse(BufferRef *const buffer, Decoder *const decoder);

  // Access to the underlying AVCodecParserContext
  bool valid() const { return parser_ctx_.get(); }
  AVCodecParserContext *get() { return parser_ctx_.get(); }
  const AVCodecParserContext *get() const { return parser_ctx_.get(); }
  AVCodecParserContext *operator->() { return parser_ctx_.operator->(); }
  const AVCodecParserContext *operator->() const { return parser_ctx_.operator->(); }

private:
  std::unique_ptr<AVCodecParserContext, decltype(&av_parser_close)> parser_ctx_;
};

// ===========================
// RAII wrapper for SwsContext
// ===========================

class Converter {
public:
  // Construct without underlying SwsContext
  Converter()
      : sws_ctx_(nullptr, &sws_freeContext), width_(0), height_(0), //
        src_format_(AV_PIX_FMT_NONE), dst_format_(AV_PIX_FMT_NONE) {};
  // Initialize the converter with the given source and destination formats
  Converter(const std::size_t width, const std::size_t height, const std::string &src_format_name,
            const std::string &dst_format_name);

  // Convert the source frame to the destination pixel format
  void convert(const Frame &src_frame, std::vector<std::uint8_t> *const dst_data);

  std::size_t width() const { return width_; }
  std::size_t height() const { return height_; }
  std::string src_format_name() const;
  std::string dst_format_name() const;

  // Access to the underlying SwsContext
  bool valid() const { return sws_ctx_.get(); }
  SwsContext *get() { return sws_ctx_.get(); }
  const SwsContext *get() const { return sws_ctx_.get(); }
  SwsContext *operator->() { return sws_ctx_.operator->(); }
  const SwsContext *operator->() const { return sws_ctx_.operator->(); }

private:
  std::unique_ptr<SwsContext, decltype(&sws_freeContext)> sws_ctx_;
  std::size_t width_, height_;
  AVPixelFormat src_format_, dst_format_;
};

} // namespace ffmpeg_cpp

#endif