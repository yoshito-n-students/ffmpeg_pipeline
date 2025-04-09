#ifndef FFMPEG_CPP_FFMPEG_CPP_HPP
#define FFMPEG_CPP_FFMPEG_CPP_HPP

#include <algorithm>
#include <chrono>
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

// Utility functions
std::string err2str(const int errnum);
std::string to_ros_image_encoding(const std::string &ffmpeg_format_name);
std::string to_ffmpeg_format_name(const std::string &ros_image_encoding);

// Error with an optional error number from libav
class Error : public std::runtime_error {
public:
  Error(const std::string &msg) : std::runtime_error(msg) {}
  Error(const std::string &msg, const int errnum)
      : std::runtime_error(msg + ": " + err2str(errnum)) {}
};

// RAII wrapper for AVBufferRef
class BufferRef {
public:
  BufferRef(const std::size_t unpadded_size);
  BufferRef(const std::uint8_t *const data, const std::size_t unpadded_size)
      : BufferRef(unpadded_size) {
    std::copy(data, data + unpadded_size, buf_->data);
  }
  BufferRef(const BufferRef &buf);

  AVBufferRef *get() { return buf_.get(); }
  const AVBufferRef *get() const { return buf_.get(); }
  AVBufferRef *operator->() { return buf_.operator->(); }
  const AVBufferRef *operator->() const { return buf_.operator->(); }

  std::size_t padded_size() const { return buf_->size; }
  std::size_t unpadded_size() const { return buf_->size - AV_INPUT_BUFFER_PADDING_SIZE; }

private:
  static void unref_buffer(AVBufferRef *buf);

private:
  std::unique_ptr<AVBufferRef, decltype(&unref_buffer)> buf_;
};

// RAII wrapper for AVPacket
class Packet {
public:
  // Allocate the packet and default the fields
  Packet();

  void ref(const BufferRef &buf);

  // Detach the packet from the data buffer if any, and default the rest of the fields
  void unref();

  AVPacket *get() { return packet_.get(); }
  const AVPacket *get() const { return packet_.get(); }
  AVPacket *operator->() { return packet_.operator->(); }
  const AVPacket *operator->() const { return packet_.operator->(); }

private:
  static void free_packet(AVPacket *packet);

private:
  std::unique_ptr<AVPacket, decltype(&free_packet)> packet_;
};

// RAII wrapper for AVFrame
class Frame {
public:
  // Allocate the frame and default the fields
  Frame();

  // Detach the frame from the data buffers if any, and default the rest of the fields
  void unref();

  // True if the frame is on a hardware memory, which is not accessible by the CPU
  bool is_hw_frame() const { return frame_->hw_frames_ctx != nullptr; }

  // Copy the frame to the CPU-accessible memory
  void transfer_data(Frame *const dst) const;

  AVFrame *get() { return frame_.get(); }
  const AVFrame *get() const { return frame_.get(); }
  AVFrame *operator->() { return frame_.operator->(); }
  const AVFrame *operator->() const { return frame_.operator->(); }

  std::string format_name() const;

private:
  static void free_frame(AVFrame *frame);

private:
  std::unique_ptr<AVFrame, decltype(&free_frame)> frame_;
};

// RAII wrapper for input decice (a.k.a. AVFormatContext)
class Input {
private:
  using Clock = std::chrono::steady_clock;

public:
  // Open the input device by avformat_open_input() with the given URL, format and options
  // and find the best video stream
  void reconfigure(const std::string &url, const std::string &format_name,
                   const std::map<std::string, std::string> &option_map,
                   const std::string &media_type_name);

  // Read the next frame from the video stream of interest
  template <typename Rep, typename Period>
  void read_frame(Packet *const packet, const std::chrono::duration<Rep, Period> &timeout) {
    read_frame_impl(packet, std::chrono::duration_cast<Clock::duration>(timeout));
  }

  AVFormatContext *get() { return format_ctx_.get(); }
  const AVFormatContext *get() const { return format_ctx_.get(); }
  AVFormatContext *operator->() { return format_ctx_.operator->(); }
  const AVFormatContext *operator->() const { return format_ctx_.operator->(); }

  int stream_id() const { return stream_id_; }
  std::string codec_name() const;

private:
  void read_frame_impl(Packet *const packet, const Clock::duration &timeout);

  static void close_input(AVFormatContext *format_ctx);

private:
  std::unique_ptr<AVFormatContext, decltype(&close_input)> format_ctx_{nullptr, &close_input};
  int stream_id_ = -1;
  Clock::time_point deadline_ = Clock::time_point::max();
};

// RAII wrapper for decoder (a.k.a AVCodecContext)
class Decoder {
public:
  // Check if the codec is supported by the current decoder context
  bool is_supported(const std::string &codec_name) const;

  // (Re)initialize the decoder with the given codec name
  void reconfigure(const std::string &codec_name);

  // Send a cmpressed packet to the decoder
  void send_packet(const Packet &packet);

  // True if a decoded frame is received, false if no more frames are available
  bool receive_frame(Frame *const frame);

  AVCodecContext *get() { return codec_ctx_.get(); }
  const AVCodecContext *get() const { return codec_ctx_.get(); }
  AVCodecContext *operator->() { return codec_ctx_.operator->(); }
  const AVCodecContext *operator->() const { return codec_ctx_.operator->(); }

  std::string codec_name() const;
  std::string hw_device_type() const;

private:
  static void free_context(AVCodecContext *codec_ctx);

private:
  std::unique_ptr<AVCodecContext, decltype(&free_context)> codec_ctx_{nullptr, &free_context};
};

// RAII wrapper for AVParserContext
class Parser {
public:
  // Check if the codec is supported by the current parser context
  bool is_supported(const std::string &codec_name) const;

  // (Re)initialize the parser with the given codec name
  void reconfigure(const std::string &codec_name);

  // Parse the buffer starting from the pos-th byte, and store the data of one frame in the packet.
  // The data in the packet is managed by reference counting, so its lifetime is guaranteed.
  // Return the number of bytes successfully parsed.
  int parse(const BufferRef &buffer, Decoder *const decoder, Packet *const packet,
            const std::int64_t pos);

  AVCodecParserContext *get() { return parser_ctx_.get(); }
  const AVCodecParserContext *get() const { return parser_ctx_.get(); }
  AVCodecParserContext *operator->() { return parser_ctx_.operator->(); }
  const AVCodecParserContext *operator->() const { return parser_ctx_.operator->(); }

  std::vector<std::string> codec_names() const;

private:
  std::unique_ptr<AVCodecParserContext, decltype(&av_parser_close)> parser_ctx_{nullptr,
                                                                                &av_parser_close};
};

// RAII wrapper for SwsContext
class Converter {
public:
  // Check if the conversion from the given source format to the destination format is supported
  // by the current converter context
  bool is_supported(const std::size_t width, const std::size_t height,
                    const std::string &src_format_name, const std::string &dst_format_name) const;

  // (Re)initialize the converter with the given source and destination formats
  void reconfigure(const std::size_t width, const std::size_t height,
                   const std::string &src_format_name, const std::string &dst_format_name);

  // Convert the source frame to the destination pixel format
  void convert(const Frame &src_frame, std::vector<std::uint8_t> *const dst_data);

  std::size_t width() const { return width_; }
  std::size_t height() const { return height_; }
  std::string src_format_name() const;
  std::string dst_format_name() const;

private:
  std::unique_ptr<SwsContext, decltype(&sws_freeContext)> sws_ctx_{nullptr, &sws_freeContext};
  std::size_t width_ = 0, height_ = 0;
  AVPixelFormat src_format_ = AV_PIX_FMT_NONE;
  AVPixelFormat dst_format_ = AV_PIX_FMT_NONE;
};

} // namespace ffmpeg_cpp

#endif