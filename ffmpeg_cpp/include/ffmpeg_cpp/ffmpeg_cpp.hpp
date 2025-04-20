#ifndef FFMPEG_CPP_FFMPEG_CPP_HPP
#define FFMPEG_CPP_FFMPEG_CPP_HPP

#include <cstdint>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ffmpeg_pipeline_msgs/msg/packet.hpp>
#include <rclcpp/time.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}

#include <yaml-cpp/yaml.h>

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
  BufferRef(const BufferRef &other);
  BufferRef &operator=(const BufferRef &other);
  // We need to define the move constructor and operator explicitly
  // because they are not automatically defined if the copy constructor is manually defined.
  BufferRef(BufferRef &&other) = default;
  BufferRef &operator=(BufferRef &&other) = default;

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
  // Create a packet by copying the given message
  Packet(const ffmpeg_pipeline_msgs::msg::Packet &msg);
  // Create a packet by referencing the data of the given packet.
  // If the data is not reference-counted, copy the data to a new packet.
  Packet(const Packet &other);
  Packet &operator=(const Packet &other);
  // We need to define the move constructor and operator explicitly
  // because they are not automatically defined if the copy constructor is manually defined.
  Packet(Packet &&other) = default;
  Packet &operator=(Packet &&other) = default;

  // True if the packet data is empty or invalid
  bool empty() const { return !packet_ || !packet_->data || packet_->size == 0; }

  // Convert the packet to a message
  ffmpeg_pipeline_msgs::msg::Packet to_msg(const rclcpp::Time &stamp,
                                           const std::string &codec_name) const;

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
  // Create a frame by referencing the data of the given frame.
  // If the data is not reference-counted, copy the data to a new frame.
  Frame(const Frame &other);
  Frame &operator=(const Frame &other);
  // We need to define the move constructor and operator explicitly
  // because they are not automatically defined if the copy constructor is manually defined.
  Frame(Frame &&other) = default;
  Frame &operator=(Frame &&other) = default;

  // True if the packet data is empty or invalid
  bool empty() const { return !frame_ || !frame_->data[0]; }

  std::string format_name() const;

  // True if the frame is on a hardware memory, which is not accessible by the CPU
  bool is_hw_frame() const { return frame_ && frame_->hw_frames_ctx; }

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

// =============================
// RAII wrapper for AVDictionary
// =============================

class Dictionary {
public:
  // Construct without underlying AVDictionary
  Dictionary() : dict_(nullptr, &free_dict) {};
  // Take ownership of the given AVDictionary
  Dictionary(AVDictionary *const dict) : dict_(dict, &free_dict) {}
  // Create a dictionary by parsing the given yaml string
  Dictionary(const std::string &yaml);
  // Create a dictionary by copying the entries from the given map
  Dictionary(const std::map<std::string, std::string> &map);
  // Create a dictionary by copying the given dictionary
  Dictionary(const Dictionary &other);
  Dictionary &operator=(const Dictionary &other) {
    *this = Dictionary(other);
    return *this;
  }
  // We need to define the move constructor and operator explicitly
  // because they are not automatically defined if the copy constructor is manually defined.
  Dictionary(Dictionary &&other) = default;
  Dictionary &operator=(Dictionary &&other) = default;

  std::map<std::string, std::string> to_map() const;

  // Access to the underlying AVDictionary
  AVDictionary *get() { return dict_.get(); }
  const AVDictionary *get() const { return dict_.get(); }
  AVDictionary *operator->() { return dict_.operator->(); }
  const AVDictionary *operator->() const { return dict_.operator->(); }
  AVDictionary *release() { return dict_.release(); }

private:
  static void free_dict(AVDictionary *dict);

private:
  std::unique_ptr<AVDictionary, decltype(&free_dict)> dict_;
};

// ==================================
// RAII wrapper for AVCodecParameters
// ==================================

class CodecParameters {
public:
  // Allocate the codec parameters and default the fields
  CodecParameters();
  // Allocate the codec parameters and set fields from the given yaml string
  CodecParameters(const std::string &yaml);
  // Create a codec parameters by copying the given codec parameters
  CodecParameters(const CodecParameters &other);
  CodecParameters &operator=(const CodecParameters &other) {
    *this = CodecParameters(other);
    return *this;
  }
  // We need to define the move constructor and operator explicitly
  // because they are not automatically defined if the copy constructor is manually defined.
  CodecParameters(CodecParameters &&other) = default;
  CodecParameters &operator=(CodecParameters &&other) = default;

  std::string codec_type_name() const;
  std::string codec_name() const;
  std::string format_name() const;
  std::string ch_layout_str() const;

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
  Input() : iformat_ctx_(nullptr, &close_input), istream_id_(-1) {}
  // Open the input device by avformat_open_input() with the given URL, format name, and options,
  // and find the best stream of the given media type
  Input(const std::string &url, const std::string &iformat_name,
        const std::map<std::string, std::string> &option_map, const std::string &media_type_name);

  CodecParameters codec_parameters() const;

  // Get a frame from the stream of interest in a NON-BLOCKING way.
  // If the next frame is not available for some temporary reason, return an empty frame.
  Packet read_frame();

  // Access to the underlying AVFormatContext
  bool valid() const { return iformat_ctx_.get(); }
  AVFormatContext *get() { return iformat_ctx_.get(); }
  const AVFormatContext *get() const { return iformat_ctx_.get(); }
  AVFormatContext *operator->() { return iformat_ctx_.operator->(); }
  const AVFormatContext *operator->() const { return iformat_ctx_.operator->(); }

private:
  static void close_input(AVFormatContext *iformat_ctx);

private:
  std::unique_ptr<AVFormatContext, decltype(&close_input)> iformat_ctx_;
  int istream_id_;
};

// ===============================================
// RAII wrapper for decoder (a.k.a AVCodecContext)
// ===============================================

class Decoder {
public:
  // Construct without underlying AVCodecContext
  Decoder() : decoder_ctx_(nullptr, &free_context) {}
  // Allocate the codec context for the given codec name.
  // Parameters can be filled with Parser::parse().
  Decoder(const std::string &codec_name);
  // Allocate the codec context for the given codec parameters
  Decoder(const CodecParameters &params);

  std::string codec_name() const;
  std::string hw_type_name() const;

  // Send a compressed packet to the decoder
  void send_packet(const Packet &packet);

  // Receive a decoded frame from the decoder.
  // The frame may be empty if no frame is available.
  Frame receive_frame();

  // Access to the underlying AVCodecContext
  bool valid() const { return decoder_ctx_.get(); }
  AVCodecContext *get() { return decoder_ctx_.get(); }
  const AVCodecContext *get() const { return decoder_ctx_.get(); }
  AVCodecContext *operator->() { return decoder_ctx_.operator->(); }
  const AVCodecContext *operator->() const { return decoder_ctx_.operator->(); }

private:
  static void free_context(AVCodecContext *decoder_ctx);

private:
  std::unique_ptr<AVCodecContext, decltype(&free_context)> decoder_ctx_;
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

// ===============================================
// RAII wrapper for encoder (a.k.a AVCodecContext)
// ===============================================

class Encoder {
public:
  // Construct without underlying AVCodecContext
  Encoder() : encoder_ctx_(nullptr, &free_context) {}
  // Allocate the codec context for the given codec name.
  // Parameters can be filled with Parser::parse().
  Encoder(const std::string &codec_name);
  // Allocate the codec context for the given codec parameters
  Encoder(const CodecParameters &params);

  std::string codec_name() const;
  std::string hw_type_name() const;

  // Send a raw frame to the encoder
  void send_frame(const Frame &frame);

  // Receive a encoded packet from the encoder.
  // The packet may be empty if no packet is available.
  Packet receive_packet();

  // Access to the underlying AVCodecContext
  bool valid() const { return encoder_ctx_.get(); }
  AVCodecContext *get() { return encoder_ctx_.get(); }
  const AVCodecContext *get() const { return encoder_ctx_.get(); }
  AVCodecContext *operator->() { return encoder_ctx_.operator->(); }
  const AVCodecContext *operator->() const { return encoder_ctx_.operator->(); }

private:
  static void free_context(AVCodecContext *encoder_ctx);

private:
  std::unique_ptr<AVCodecContext, decltype(&free_context)> encoder_ctx_;
};

// ===========================
// RAII wrapper for SwsContext
// ===========================

class VideoConverter {
public:
  // Construct without underlying SwsContext
  VideoConverter() : sws_ctx_(nullptr, &sws_freeContext) {};
  // Initialize the converter for changing only the pixel format
  VideoConverter(const std::size_t width, const std::size_t height,
                 const std::string &src_format_name, const std::string &dst_format_name)
      : VideoConverter(width, height, src_format_name, width, height, dst_format_name) {};
  // Initialize the converter for changing the pixel format and the size
  VideoConverter(const std::size_t src_width, const std::size_t src_height,
                 const std::string &src_format_name, const std::size_t dst_width,
                 const std::size_t dst_height, const std::string &dst_format_name);

  std::size_t src_width() const;
  std::size_t src_height() const;
  std::string src_format_name() const;
  std::size_t dst_width() const;
  std::size_t dst_height() const;
  std::string dst_format_name() const;

  // Convert the source frame to the destination format
  Frame convert(const Frame &src_frame);
  std::vector<std::uint8_t> convert_to_vector(const Frame &src_frame);

  // Access to the underlying SwsContext
  bool valid() const { return sws_ctx_.get(); }
  SwsContext *get() { return sws_ctx_.get(); }
  const SwsContext *get() const { return sws_ctx_.get(); }
  SwsContext *operator->() { return sws_ctx_.operator->(); }
  const SwsContext *operator->() const { return sws_ctx_.operator->(); }

private:
  std::unique_ptr<SwsContext, decltype(&sws_freeContext)> sws_ctx_;
};

// ===========================
// RAII wrapper for SwrContext
// ===========================

class AudioConverter {
public:
  AudioConverter() : swr_ctx_(nullptr, &free_context) {}
  AudioConverter(const std::string &in_ch_layout_str, const std::string &in_format_name,
                 const int in_sample_rate, //
                 const std::string &out_ch_layout_str, const std::string &out_format_name,
                 const int out_sample_rate);

  std::string in_ch_layout_str() const;
  std::string in_format_name() const;
  std::string out_ch_layout_str() const;
  std::string out_format_name() const;

  // Convert the source frame to the destination format
  Frame convert(const Frame &src_frame);

  // Access to the underlying SwrContext
  bool valid() const { return swr_ctx_.get(); }
  SwrContext *get() { return swr_ctx_.get(); }
  const SwrContext *get() const { return swr_ctx_.get(); }
  SwrContext *operator->() { return swr_ctx_.operator->(); }
  const SwrContext *operator->() const { return swr_ctx_.operator->(); }

private:
  static void free_context(SwrContext *swr_ctx);

private:
  std::unique_ptr<SwrContext, decltype(&free_context)> swr_ctx_;
};

// =======================================================
// RAII wrapper for output device (a.k.a. AVFormatContext)
// =======================================================

class Output {
public:
  // Construct without underlying AVFormatContext
  Output() : oformat_ctx_(nullptr, &close_output), ostream_(nullptr), increasing_dts_(0) {}
  // Open the output device with the given format name and filename,
  // and set the codec parameters and options to the stream
  Output(const std::string &oformat_name, const std::string &filename,
         const CodecParameters &codec_params, const std::map<std::string, std::string> &option_map);

  // Write a frame to the stream of interest in a NON-BLOCKING way.
  // False if the output stream is not ready to accept the data for some temporary reason.
  bool write_frame(const Packet &packet);

  // Access to the underlying AVFormatContext
  bool valid() const { return oformat_ctx_.get(); }
  AVFormatContext *get() { return oformat_ctx_.get(); }
  const AVFormatContext *get() const { return oformat_ctx_.get(); }
  AVFormatContext *operator->() { return oformat_ctx_.operator->(); }
  const AVFormatContext *operator->() const { return oformat_ctx_.operator->(); }

private:
  static void close_output(AVFormatContext *oformat_ctx);

private:
  std::unique_ptr<AVFormatContext, decltype(&close_output)> oformat_ctx_;
  AVStream *ostream_;
  std::uint64_t increasing_dts_;
};

} // namespace ffmpeg_cpp

// =====================================
// Conversion between datatypes and yaml
// =====================================

namespace YAML {

template <> struct convert<ffmpeg_cpp::Dictionary> {
  static bool decode(const Node &yaml, ffmpeg_cpp::Dictionary &dict);
};

template <> struct convert<ffmpeg_cpp::CodecParameters> {
  static bool decode(const Node &yaml, ffmpeg_cpp::CodecParameters &params);
};

} // namespace YAML

#endif