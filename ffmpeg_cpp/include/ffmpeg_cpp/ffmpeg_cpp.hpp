#ifndef FFMPEG_CPP_FFMPEG_CPP_HPP
#define FFMPEG_CPP_FFMPEG_CPP_HPP

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ffmpeg_pipeline_msgs/msg/packet.hpp>
#include <rclcpp/time.hpp>

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

// ===============================
// Deleter for RAII wrappers below
// ===============================

template <typename T> struct Deleter {
  void operator()(T *ptr) const;
};

// =========================
// RAII wrapper for AVPacket
// =========================

class Packet : public std::unique_ptr<AVPacket, Deleter<AVPacket>> {
private:
  using std::unique_ptr<AVPacket, Deleter<AVPacket>>::unique_ptr;
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
  bool empty() const { return !get() || !get()->data || get()->size == 0; }

  // Convert the packet to a message
  ffmpeg_pipeline_msgs::msg::Packet to_msg(const rclcpp::Time &stamp,
                                           const std::string &codec_name) const;
};

// ========================
// RAII wrapper for AVFrame
// ========================

class Frame : public std::unique_ptr<AVFrame, Deleter<AVFrame>> {
public:
  // Allocate the frame and default the fields
  Frame();
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

  // True if the packet data is empty or invalid
  bool empty() const { return !get() || !get()->data[0]; }

  std::string format_name() const;
  std::string ch_layout_str() const;

  // True if the frame is on a hardware memory, which is not accessible by the CPU
  bool is_hw_frame() const { return get() && get()->hw_frames_ctx; }

  // Copy the frame to the CPU-accessible memory
  Frame transfer_data() const;
};

// =============================
// RAII wrapper for AVDictionary
// =============================

class Dictionary : public std::unique_ptr<AVDictionary, Deleter<AVDictionary>> {
public:
  // Construct without underlying AVDictionary
  static Dictionary null();
  Dictionary();
  // Create a dictionary by parsing the given yaml string
  Dictionary(const std::string &yaml);
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

class CodecParameters : public std::unique_ptr<AVCodecParameters, Deleter<AVCodecParameters>> {
public:
  static CodecParameters null();
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
};

// ======================================================
// RAII wrapper for input device (a.k.a. AVFormatContext)
// ======================================================

class Input : public std::unique_ptr<AVFormatContext, Deleter<AVFormatContext>> {
public:
  // Construct without underlying AVFormatContext
  Input();
  // Open the input device by avformat_open_input() with the given URL, format name, and options,
  // and find the best stream of the given media type
  Input(const std::string &url, const std::string &format_name, Dictionary *const options,
        const std::string &media_type_name);

  std::string format_name() const;
  std::string url() const;
  CodecParameters codec_parameters() const;

  // Get a frame from the stream of interest in a NON-BLOCKING way.
  // If the next frame is not available for some temporary reason, return an empty frame.
  Packet read_frame();

private:
  int istream_id_ = -1;
};

// ================================
// RAII wrapper for AVParserContext
// ================================

class Parser : public std::unique_ptr<AVCodecParserContext, Deleter<AVCodecParserContext>> {
public:
  // Construct without underlying AVCodecParserContext
  Parser();
  // Allocate the parser context for the given codec name
  Parser(const std::string &codec_name);

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
  std::unique_ptr<AVCodecContext, Deleter<AVCodecContext>> codec_ctx_ = nullptr;
};

// ===============================================
// RAII wrapper for decoder (a.k.a AVCodecContext)
// ===============================================

class Decoder : public std::unique_ptr<AVCodecContext, Deleter<AVCodecContext>> {
public:
  // Construct without underlying AVCodecContext
  Decoder();
  // Allocate the codec context for the given arguments. All of them are optional.
  // The underlying decoder is determined by decoder_name if not empty,
  // otherwise by codec_params->codec_id. If multiple decoders support the codec_id,
  // ffmpeg selects the default one.
  Decoder(const std::string &decoder_name,
          const CodecParameters &codec_params = CodecParameters::null(),
          const Dictionary &decoder_options = Dictionary::null());

  std::string codec_name() const;
  std::string hw_type_name() const;

  // Send a compressed packet to the decoder
  void send_packet(const Packet &packet);

  // Receive a decoded frame from the decoder.
  // The frame may be empty if no frame is available.
  Frame receive_frame();
};

// ===============================================
// RAII wrapper for encoder (a.k.a AVCodecContext)
// ===============================================

class Encoder : public std::unique_ptr<AVCodecContext, Deleter<AVCodecContext>> {
public:
  // Construct without underlying AVCodecContext
  Encoder();
  // Allocate the codec context for the given codec parameters and options
  Encoder(const CodecParameters &codec_params, Dictionary *const codec_options);

  std::string codec_name() const;
  std::string hw_type_name() const;

  // Send a raw frame to the encoder
  void send_frame(const Frame &frame);

  // Receive a encoded packet from the encoder.
  // The packet may be empty if no packet is available.
  Packet receive_packet();
};

// ============================
// RAII wrapper for AVAudioFifo
// ============================

class AudioFifo : public std::unique_ptr<AVAudioFifo, Deleter<AVAudioFifo>> {
public:
  AudioFifo();
  AudioFifo(const std::string &ch_layout_str, const std::string &format_name,
            const int sample_rate);

  std::string ch_layout_str() const;
  std::string format_name() const;
  int sample_rate() const;

  // Push a frame to the FIFO
  void write(const Frame &frame);

  // Pop nb_samples samples if available in the FIFO, or return an empty frame
  Frame read(const int nb_samples);

private:
  Frame template_frame_ = Frame();
};

// ===========================
// RAII wrapper for SwsContext
// ===========================

class VideoConverter : public std::unique_ptr<SwsContext, Deleter<SwsContext>> {
public:
  // Construct without underlying SwsContext
  VideoConverter();
  // Initialize the converter for changing only the pixel format
  VideoConverter(const int width, const int height, //
                 const std::string &src_format_name, const std::string &dst_format_name)
      : VideoConverter(width, height, src_format_name, width, height, dst_format_name) {};
  // Initialize the converter for changing the pixel format and the size
  VideoConverter(const int src_width, const int src_height, const std::string &src_format_name,
                 const int dst_width, const int dst_height, const std::string &dst_format_name);

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

class AudioConverter : public std::unique_ptr<SwrContext, Deleter<SwrContext>> {
public:
  AudioConverter();
  AudioConverter(const std::string &in_ch_layout_str, const std::string &in_format_name,
                 const int in_sample_rate, //
                 const std::string &out_ch_layout_str, const std::string &out_format_name,
                 const int out_sample_rate);

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

class Output : public std::unique_ptr<AVFormatContext, Deleter<AVFormatContext>> {
public:
  // Construct without underlying AVFormatContext
  Output();
  // Open the output device with the given format name and url,
  // and set the codec parameters and options to the stream
  Output(const std::string &format_name, const std::string &url,
         const CodecParameters &codec_params, Dictionary *const options);

  std::string format_name() const;
  std::string url() const;

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