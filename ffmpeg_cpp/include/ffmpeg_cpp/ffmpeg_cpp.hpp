#ifndef FFMPEG_CPP_FFMPEG_CPP_HPP
#define FFMPEG_CPP_FFMPEG_CPP_HPP

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ffmpeg_pipeline_msgs/msg/frame.hpp>
#include <ffmpeg_pipeline_msgs/msg/packet.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/audio_fifo.h>
#include <libavutil/avutil.h>
#include <libswresample/swresample.h>
#include <libswscale/swscale.h>
}

#include <yaml-cpp/yaml.h>

namespace ffmpeg_cpp {

// =================
// Utility functions
// =================

// Set the log level ("quiet", "panic", "fatal", ..., "verbose", "debug", "trace") for ffmpeg
void set_log_level(const std::string &log_level);
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

// =====================================
// Smart pointer for RAII wrappers below
// =====================================

template <class T> struct Deleter {
  void operator()(T *ptr) const;
};
template <class T> using UniquePtr = std::unique_ptr<T, Deleter<T>>;

// =========================
// RAII wrapper for AVPacket
// =========================

class Packet : public UniquePtr<AVPacket> {
private:
  using UniquePtr<AVPacket>::UniquePtr;
  Packet() = delete;

public:
  // Construct without underlying AVPacket
  static Packet null();
  // Allocate the packet and default the fields
  static Packet create();
  // Create a packet by copying the given data
  static Packet create(const std::uint8_t *const data, const std::size_t size);
  // Create a packet by copying the given message
  static Packet create(const ffmpeg_pipeline_msgs::msg::Packet &msg);
  // Create a packet by referencing the data of the given packet.
  // If the data is not reference-counted, copy the data to a new packet.
  Packet(const Packet &other);
  Packet &operator=(const Packet &other) {
    *this = Packet(other);
    return *this;
  }
  // We need to define the move constructor and operator explicitly
  // because they are not automatically defined if the copy constructor is manually defined.
  Packet(Packet &&other) = default;
  Packet &operator=(Packet &&other) = default;

  // True if the packet data is empty or invalid
  bool empty() const {
    return !get()               // no AVPacket instance
           || !get()->data      // OR no data
           || get()->size <= 0; // OR no size
  }

  // Convert the packet to a message
  ffmpeg_pipeline_msgs::msg::Packet to_packet_msg(const rclcpp::Time &stamp,
                                                  const std::string &codec_name) const;
  sensor_msgs::msg::CompressedImage to_compressed_image_msg(const rclcpp::Time &stamp,
                                                            const std::string &format) const;
};

// ========================
// RAII wrapper for AVFrame
// ========================

class Frame : public UniquePtr<AVFrame> {
private:
  using UniquePtr<AVFrame>::UniquePtr;
  Frame() = delete;

public:
  // Construct without underlying AVFrame
  static Frame null();
  // Allocate the frame and default the fields
  static Frame create();
  // Create a frame by copying the given data
  static Frame create(const std::uint8_t *const data, const std::size_t size);
  // Create a frame by copying the given message
  static Frame create(const ffmpeg_pipeline_msgs::msg::Frame &msg);
  // Create a frame by referencing the data of the given frame.
  // If the data is not reference-counted, copy the data to a new frame.
  Frame(const Frame &other);
  Frame &operator=(const Frame &other) {
    *this = Frame(other);
    return *this;
  }
  // We need to define the move constructor and operator explicitly
  // because they are not automatically defined if the copy constructor is manually defined.
  Frame(Frame &&other) = default;
  Frame &operator=(Frame &&other) = default;

  // True if the frame data is empty or invalid
  bool empty() const {
    return !get()                                        // no AVFrame instance
           || (!get()->data[0] && !get()->hw_frames_ctx) // OR no data on CPU nor GPU
           || ((get()->width <= 0 || get()->height <= 0) // OR no pixels and audio samples
               && get()->nb_samples <= 0);
  }

  // Convert this->{format, ch_layout} using FFmpeg's utility functions.
  // If error or this is null, return an empty string.
  std::string format_name() const;
  std::string ch_layout_str() const;

  // True if the frame is on a hardware memory, which is not accessible by the CPU
  bool is_hw_frame() const { return get() && get()->hw_frames_ctx; }

  // Copy the frame to the CPU-accessible memory
  Frame transfer_data() const;

  // Convert the frame to a message
  ffmpeg_pipeline_msgs::msg::Frame to_msg(const rclcpp::Time &stamp) const;
};

// =============================
// RAII wrapper for AVDictionary
// =============================

class Dictionary : public UniquePtr<AVDictionary> {
private:
  using UniquePtr<AVDictionary>::UniquePtr;
  // yaml-cpp needs a default constructor to create the object
  Dictionary() = default;
  friend struct YAML::as_if<Dictionary, void>;
  friend struct YAML::as_if<Dictionary, Dictionary>;

public:
  // Construct without underlying AVDictionary
  static Dictionary null();
  // Create a dictionary by parsing the given yaml string
  static Dictionary create(const std::string &yaml);
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

  std::string to_yaml() const;
  std::string to_flow_style_yaml() const;
};

// ==================================
// RAII wrapper for AVCodecParameters
// ==================================

class CodecParameters : public UniquePtr<AVCodecParameters> {
private:
  using UniquePtr<AVCodecParameters>::UniquePtr;
  // yaml-cpp needs a default constructor to create the object
  CodecParameters() = default;
  friend struct YAML::as_if<CodecParameters, void>;
  friend struct YAML::as_if<CodecParameters, CodecParameters>;

public:
  // Construct without underlying AVCodecParameters
  static CodecParameters null();
  // Allocate the codec parameters and default the fields
  static CodecParameters create();
  // Allocate the codec parameters and set fields from the given yaml string
  static CodecParameters create(const std::string &yaml);
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

  // Convert this->{codec_type, codec_id, format, ch_layout} using FFmpeg's utility functions.
  // If error or this is null, return an empty string.
  std::string codec_type_name() const;
  std::string codec_name() const;
  std::string format_name() const;
  std::string ch_layout_str() const;
};

// ======================================================
// RAII wrapper for input device (a.k.a. AVFormatContext)
// ======================================================

class Input : public UniquePtr<AVFormatContext> {
private:
  using UniquePtr<AVFormatContext>::UniquePtr;
  Input() = delete;

public:
  // Construct without underlying AVFormatContext
  static Input null();
  // Open the input device by avformat_open_input() with the given URL, format name, and options,
  // and find the best stream of the given media type
  static Input create(const std::string &url, const std::string &format_name,
                      const Dictionary &options, const std::string &media_type_name);

  // Return a copy of istream_->codecpar.
  // If istream_ or istream_->codecpar is null, return CodecParameters::null().
  CodecParameters codec_parameters() const;

  // Get a frame from the stream of interest in a NON-BLOCKING way.
  // If the next frame is not available for some temporary reason, return a null frame.
  Packet read_frame();

private:
  AVStream *istream_ = nullptr;
};

// ================================
// RAII wrapper for AVParserContext
// ================================

class Parser : public UniquePtr<AVCodecParserContext> {
private:
  using UniquePtr<AVCodecParserContext>::UniquePtr;
  Parser() = delete;

public:
  // Construct without underlying AVCodecParserContext
  static Parser null();
  // Allocate the parser context for the given decoder name
  static Parser create(const std::string &decoder_name);

  // Convert this->parser->codec_ids[] to a vector of codec names.
  // If this is null, return an empty vector.
  std::vector<std::string> codec_names() const;

  // Parse the given buffer and return the found packet and parameters.
  // The parameters contain enough information to decode the packet.
  // If no packet is found or the found packet is not a keyframe,
  // return an empty packet and default parameters.
  std::pair<Packet, CodecParameters> parse_initial_packet(const Packet &buffer,
                                                          std::int64_t *const pos);

  // Parse the given buffer and return the found packet.
  // If no packet is found, return an empty packet.
  Packet parse_next_packet(const Packet &buffer, std::int64_t *const pos);

private:
  UniquePtr<AVCodecContext> decoder_ctx_ = nullptr;
};

// ===============================================
// RAII wrapper for decoder (a.k.a AVCodecContext)
// ===============================================

class Decoder : public UniquePtr<AVCodecContext> {
private:
  using UniquePtr<AVCodecContext>::UniquePtr;
  Decoder() = delete;

public:
  // Construct without underlying AVCodecContext
  static Decoder null();
  // Allocate the AVCodecContext for the given arguments. All of them are optional.
  // The underlying decoder is determined by decoder_name if not empty,
  // otherwise by codec_params->codec_id. If multiple decoders support the codec_id,
  // ffmpeg selects the default one.
  // The hardware acceleration is enabled if hw_type_name is not empty.
  // It can be a valid hardware type name or "auto" to select the first available one.
  static Decoder create(const std::string &decoder_name,
                        const CodecParameters &codec_params = CodecParameters::null(),
                        const std::string &hw_type_name = "",
                        const std::string &request_format_name = "",
                        const Dictionary &decoder_options = Dictionary::null());

  // Convert this->{codec_id, hw_device_ctx->data->type} using FFmpeg's utility functions.
  // If error or this is null, return an empty string.
  std::string codec_name() const;
  std::string hw_type_name() const;

  // Send a compressed packet to the decoder
  void send_packet(const Packet &packet);

  // Receive a decoded frame from the decoder.
  // The frame may be null if no frame is available.
  Frame receive_frame();
};

// ===============================================
// RAII wrapper for encoder (a.k.a AVCodecContext)
// ===============================================

class Encoder : public UniquePtr<AVCodecContext> {
private:
  using UniquePtr<AVCodecContext>::UniquePtr;
  Encoder() = delete;

public:
  // Construct without underlying AVCodecContext
  static Encoder null();
  // Allocate the AVCodecContext for the given arguments. All of them are optional.
  // The underlying encoder is determined by encoder_name if not empty,
  // otherwise by codec_params->codec_id. If multiple encoders support the codec_id,
  // ffmpeg selects the default one.
  // The hardware acceleration is enabled if hw_type_name is not empty.
  // It can be a valid hardware type name or "auto" to select the first available one.
  static Encoder create(const std::string &encoder_name,
                        const CodecParameters &codec_params = CodecParameters::null(),
                        const std::string &hw_type_name = "",
                        const Dictionary &encoder_options = Dictionary::null());

  // Convert this->{codec_id, hw_device_ctx->data->type} using FFmpeg's utility functions.
  // If error or this is null, return an empty string.
  std::string codec_name() const;
  std::string hw_type_name() const;

  // Send a raw frame to the encoder
  void send_frame(const Frame &frame);

  // Receive a encoded packet from the encoder.
  // The packet may be null if no packet is available.
  Packet receive_packet();
};

// ============================
// RAII wrapper for AVAudioFifo
// ============================

class AudioFifo : public UniquePtr<AVAudioFifo> {
private:
  using UniquePtr<AVAudioFifo>::UniquePtr;
  AudioFifo() = delete;

public:
  static AudioFifo null();
  static AudioFifo create(const std::string &ch_layout_str, const std::string &format_name,
                          const int sample_rate);

  // Return properties of the underlying AVAudioFifo.
  // If it has not been created, return empty strings or 0.
  std::string ch_layout_str() const;
  std::string format_name() const;
  int sample_rate() const;

  // Push a frame to the FIFO
  void write(const Frame &frame);

  // Pop nb_samples samples if available in the FIFO, or return a null frame
  Frame read(const int nb_samples);

private:
  Frame template_frame_ = Frame::null();
};

// ===========================
// RAII wrapper for SwsContext
// ===========================

class VideoConverter : public UniquePtr<SwsContext> {
private:
  using UniquePtr<SwsContext>::UniquePtr;
  VideoConverter() = delete;

public:
  // Construct without underlying SwsContext
  static VideoConverter null();
  // Initialize the converter for changing only the pixel format
  static VideoConverter create(const int width, const int height, //
                               const std::string &src_format_name,
                               const std::string &dst_format_name) {
    return create(width, height, src_format_name, width, height, dst_format_name);
  }
  // Initialize the converter for changing the pixel format and the size
  static VideoConverter create(const int src_width, const int src_height,
                               const std::string &src_format_name, const int dst_width,
                               const int dst_height, const std::string &dst_format_name);

  // Return properties of the underlying SwsContext.
  // If it has not been created, return empty strings or 0.
  int src_width() const;
  int src_height() const;
  std::string src_format_name() const;
  int dst_width() const;
  int dst_height() const;
  std::string dst_format_name() const;

  // Convert the source frame to the destination format
  Frame convert(const Frame &src_frame);
  std::vector<std::uint8_t> convert_to_vector(const Frame &src_frame);
};

// ===========================
// RAII wrapper for SwrContext
// ===========================

class AudioConverter : public UniquePtr<SwrContext> {
private:
  using UniquePtr<SwrContext>::UniquePtr;
  AudioConverter() = delete;

public:
  static AudioConverter null();
  static AudioConverter create(const std::string &in_ch_layout_str,
                               const std::string &in_format_name, const int in_sample_rate,
                               const std::string &out_ch_layout_str,
                               const std::string &out_format_name, const int out_sample_rate);

  // Return properties of the underlying SwrContext.
  // If it has not been created, return empty strings or 0.
  std::string in_ch_layout_str() const;
  std::string in_format_name() const;
  int in_sample_rate() const;
  std::string out_ch_layout_str() const;
  std::string out_format_name() const;
  int out_sample_rate() const;

  // Convert the source frame to the destination format
  Frame convert(const Frame &src_frame);
};

// =======================================================
// RAII wrapper for output device (a.k.a. AVFormatContext)
// =======================================================

class Output : public UniquePtr<AVFormatContext> {
private:
  using UniquePtr<AVFormatContext>::UniquePtr;
  Output() = delete;

public:
  // Construct without underlying AVFormatContext
  static Output null();
  // Open the output device with the given format name and url,
  // and set the codec parameters and options to the stream
  static Output create(const std::string &format_name, const std::string &url,
                       const CodecParameters &codec_params, const Dictionary &options);

  // Write a frame to the stream of interest in a NON-BLOCKING way.
  // False if the output stream is not ready to accept the data for some temporary reason.
  bool write_frame(const Packet &packet);
  bool write_uncoded_frame(const Frame &frame);

private:
  AVStream *ostream_ = nullptr;
  std::uint64_t increasing_dts_ = 0;
};

} // namespace ffmpeg_cpp

// =====================================
// Conversion between datatypes and yaml
// =====================================

namespace YAML {

template <> struct convert<ffmpeg_cpp::Dictionary> {
  static bool decode(const Node &yaml, ffmpeg_cpp::Dictionary &dict);
  static Node encode(const ffmpeg_cpp::Dictionary &dict);
};

template <> struct convert<ffmpeg_cpp::CodecParameters> {
  static bool decode(const Node &yaml, ffmpeg_cpp::CodecParameters &params);
};

} // namespace YAML

#endif