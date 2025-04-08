#include <algorithm>
#include <array>
#include <chrono>
#include <cstring> // for std::memset()
#include <numeric> // for std::partial_sum()

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace ffmpeg_cpp {

// =======
// Utility
// =======

std::string err2str(const int errnum) {
  char buf[AV_ERROR_MAX_STRING_SIZE];
  av_strerror(errnum, buf, sizeof(buf));
  return buf;
}

constexpr std::pair<AVPixelFormat, const char *> format_encoding_pairs[] = {
    // RGB formats
    {AV_PIX_FMT_RGB24, sensor_msgs::image_encodings::RGB8},
    {AV_PIX_FMT_RGBA, sensor_msgs::image_encodings::RGBA8},
    {AV_PIX_FMT_RGB48, sensor_msgs::image_encodings::RGB16},
    {AV_PIX_FMT_RGBA64, sensor_msgs::image_encodings::RGBA16},
    // BGR formats
    {AV_PIX_FMT_BGR24, sensor_msgs::image_encodings::BGR8},
    {AV_PIX_FMT_BGRA, sensor_msgs::image_encodings::BGRA8},
    {AV_PIX_FMT_BGR48, sensor_msgs::image_encodings::BGR16},
    {AV_PIX_FMT_BGRA64, sensor_msgs::image_encodings::BGRA16},
    // Grayscale formats
    {AV_PIX_FMT_GRAY8, sensor_msgs::image_encodings::MONO8},
    {AV_PIX_FMT_GRAY16, sensor_msgs::image_encodings::MONO16},
    // bayer formats
    {AV_PIX_FMT_BAYER_RGGB8, sensor_msgs::image_encodings::BAYER_RGGB8},
    {AV_PIX_FMT_BAYER_RGGB16, sensor_msgs::image_encodings::BAYER_RGGB16},
    {AV_PIX_FMT_BAYER_BGGR8, sensor_msgs::image_encodings::BAYER_BGGR8},
    {AV_PIX_FMT_BAYER_BGGR16, sensor_msgs::image_encodings::BAYER_BGGR16},
    {AV_PIX_FMT_BAYER_GRBG8, sensor_msgs::image_encodings::BAYER_GRBG8},
    {AV_PIX_FMT_BAYER_GRBG16, sensor_msgs::image_encodings::BAYER_GRBG16},
    {AV_PIX_FMT_BAYER_GBRG8, sensor_msgs::image_encodings::BAYER_GBRG8},
    {AV_PIX_FMT_BAYER_GBRG16, sensor_msgs::image_encodings::BAYER_GBRG16},
    // YUV formats
    {AV_PIX_FMT_UYVY422, sensor_msgs::image_encodings::UYVY},
    {AV_PIX_FMT_YUYV422, sensor_msgs::image_encodings::YUYV},
    {AV_PIX_FMT_NV21, sensor_msgs::image_encodings::NV21},
    {AV_PIX_FMT_NV24, sensor_msgs::image_encodings::NV24}};

std::string to_ros_image_encoding(const std::string &ffmpeg_format_name) {
  const AVPixelFormat ffmpeg_format = av_get_pix_fmt(ffmpeg_format_name.c_str());
  const auto found_it =
      std::find_if(std::begin(format_encoding_pairs), std::end(format_encoding_pairs),
                   [ffmpeg_format](const auto &pair) { return pair.first == ffmpeg_format; });
  return found_it != std::end(format_encoding_pairs) ? found_it->second : "";
}

std::string to_ffmpeg_format_name(const std::string &ros_image_encoding) {
  const auto found_it = std::find_if(
      std::begin(format_encoding_pairs), std::end(format_encoding_pairs),
      [&ros_image_encoding](const auto &pair) { return pair.second == ros_image_encoding; });
  return found_it != std::end(format_encoding_pairs) ? av_get_pix_fmt_name(found_it->first) : "";
}

// ========================================
// BufferRef - RAII wrapper for AVBufferRef
// ========================================

BufferRef::BufferRef(const std::size_t unpadded_size)
    : buf_(av_buffer_alloc(unpadded_size + AV_INPUT_BUFFER_PADDING_SIZE), &unref_buffer) {
  if (!buf_) {
    throw Error("BufferRef::BufferRef(): Failed to allocate AVBufferRef");
  }
  // Zero the padding area
  std::memset(buf_->data + unpadded_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);
}

BufferRef::BufferRef(const BufferRef &buf) : buf_(av_buffer_ref(buf.get()), &unref_buffer) {
  if (!buf_) {
    throw Error("BufferRef::BufferRef(): Failed to create a reference to buffer");
  }
}

void BufferRef::unref_buffer(AVBufferRef *buf) { av_buffer_unref(&buf); }

// ==================================
// Packet - RAII wrapper for AVPacket
// ==================================

Packet::Packet() : packet_(av_packet_alloc(), free_packet) {
  if (!packet_) {
    throw Error("Packet::Packet(): Failed to allocate AVPacket");
  }
}

void Packet::ref(const BufferRef &buf) {
  unref();
  packet_->buf = av_buffer_ref(buf.get());
  if (!packet_->buf) {
    throw Error("Packet::ref(): Failed to create a reference to buffer");
  }
  packet_->data = buf->data;
  packet_->size = buf.unpadded_size();
}

void Packet::unref() { av_packet_unref(packet_.get()); }

void Packet::free_packet(AVPacket *packet) { av_packet_free(&packet); }

// ================================
// Frame - RAII wrapper for AVFrame
// ================================

Frame::Frame() : frame_(av_frame_alloc(), free_frame) {
  if (!frame_) {
    throw Error("Frame::Frame(): Failed to allocate AVFrame");
  }
}

void Frame::unref() { av_frame_unref(frame_.get()); }

void Frame::transfer_data(Frame *const dst) const {
  dst->unref();
  if (const int ret = av_hwframe_transfer_data(dst->get(), frame_.get(), 0); ret < 0) {
    throw Error("Frame::transfer_data(): Error transferring data", ret);
  }
}

std::string Frame::format_name() const {
  return av_get_pix_fmt_name(static_cast<AVPixelFormat>(frame_->format));
}

void Frame::free_frame(AVFrame *frame) { av_frame_free(&frame); }

// ========================================
// Input - RAII wrapper for AVFormatContext
// ========================================

void Input::reconfigure(const std::string &url, const std::string &format_name,
                        const std::map<std::string, std::string> &option_map) {
  // Register all the input format types
  avdevice_register_all();

  // Allocate the format context
  AVFormatContext *format_ctx = avformat_alloc_context();
  if (!format_ctx) {
    throw Error("Input::Input(): Failed to allocate AVFormatContext");
  }

  // Set a callback function to return 1 if the deadline time is exceeded
  // to limit the blocking time of read_frame() and other functions.
  format_ctx->interrupt_callback.callback = [](void *deadline) {
    using Clock = std::chrono::steady_clock;
    return Clock::now() < *static_cast<Clock::time_point *>(deadline) ? 0 : 1;
  };
  format_ctx->interrupt_callback.opaque = &deadline_;

  // Find the input format by name
  const AVInputFormat *format =
      (format_name.empty() ? nullptr : av_find_input_format(format_name.c_str()));
  if (!format_name.empty() && format == nullptr) {
    avformat_free_context(format_ctx);
    throw Error("Input::Input(): " + format_name + " was not recognized as an input format");
  }

  // Build the option dictionary from the map
  AVDictionary *option_dict = nullptr;
  for (const auto &[key, value] : option_map) {
    if (const int ret = av_dict_set(&option_dict, key.c_str(), value.c_str(), 0); ret < 0) {
      avformat_free_context(format_ctx);
      av_dict_free(&option_dict);
      throw Error("Input::Input(): Failed to pack option [" + key + ", " + value + "]", ret);
    }
  }

  // Open the input with the URL, format and options
  if (const int ret = avformat_open_input(&format_ctx, url.c_str(), format, &option_dict);
      ret < 0) {
    avformat_free_context(format_ctx);
    av_dict_free(&option_dict);
    throw Error("Input::Input(): Failed to open input " + url, ret);
  }
  format_ctx_.reset(format_ctx);

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
  // (TODO: check if this is necessary to find the video stream)
  if (const int ret = avformat_find_stream_info(format_ctx, nullptr); ret < 0) {
    throw Error("Input::Input(): Failed to find stream information", ret);
  }

  // Find the best video stream
  stream_id_ = av_find_best_stream(format_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
  if (stream_id_ < 0) {
    throw Error("Input::Input(): Failed to find video stream", stream_id_);
  }
}

std::string Input::codec_name() const {
  return avcodec_get_name(format_ctx_ && 0 <= stream_id_ &&
                                  static_cast<unsigned int>(stream_id_) < format_ctx_->nb_streams
                              ? format_ctx_->streams[stream_id_]->codecpar->codec_id
                              : AV_CODEC_ID_NONE);
}

void Input::read_frame_impl(Packet *const packet, const Clock::duration &timeout) {
  if (!format_ctx_) {
    throw Error("Input::read_frame(): Input context is not configured");
  }

  packet->unref();
  deadline_ = Clock::now() + timeout;
  do {
    if (const int ret = av_read_frame(format_ctx_.get(), packet->get()); ret < 0) {
      throw Error("Input::read_frame(): Failed to read frame", ret);
    }
  } while ((*packet)->stream_index != stream_id_);
}

void Input::close_input(AVFormatContext *format_ctx) { avformat_close_input(&format_ctx); }

// =========================================
// Decoder - RAII wrapper for AVCodecContext
// =========================================

bool Decoder::is_supported(const std::string &codec_name) const {
  const AVCodec *const codec = avcodec_find_decoder_by_name(codec_name.c_str());
  return codec && codec_ctx_ && (codec->id == codec_ctx_->codec_id);
}

void Decoder::reconfigure(const std::string &codec_name) {
  // Find the decoder by name
  const AVCodec *const codec = avcodec_find_decoder_by_name(codec_name.c_str());
  if (!codec) {
    throw Error("Decoder::Decoder(): " + codec_name + " was not recognized as a decoder name");
  }

  // Allocate the decoder context and set the options to enable
  // error concealment and format preference
  codec_ctx_.reset(avcodec_alloc_context3(codec));
  if (!codec_ctx_) {
    throw Error("Decoder::Decoder(): Failed to allocate codec context");
  }
  codec_ctx_->workaround_bugs = FF_BUG_AUTODETECT;
  codec_ctx_->err_recognition = AV_EF_CRCCHECK;
  codec_ctx_->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
  codec_ctx_->get_format = [](AVCodecContext *codec_ctx, const AVPixelFormat *formats) {
    // Prefer the first pixel formats compatible with ROS image encodings
    // to avoid unnecessary conversions after decoding
    for (const AVPixelFormat *format = formats; *format != AV_PIX_FMT_NONE; ++format) {
      if (std::any_of(std::begin(format_encoding_pairs), std::end(format_encoding_pairs),
                      [format](const auto &pair) { return pair.first == *format; })) {
        return *format;
      }
    }
    // If no compatible pixel format is found, defer to the default behavior
    return avcodec_default_get_format(codec_ctx, formats);
  };

  // Create a hardware acceleration context supported by the decoder.
  // If multiple hardware devices are supported, the first one is used.
  if (!codec_ctx_->hw_device_ctx) {
    for (int i = 0;; ++i) {
      const AVCodecHWConfig *const hw_config = avcodec_get_hw_config(codec, i);
      if (hw_config                                                         // HW config exists
          && (hw_config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) // HW context supported
          && av_hwdevice_ctx_create(&codec_ctx_->hw_device_ctx, hw_config->device_type, nullptr,
                                    nullptr, 0) == 0 // HW context created
      ) {
        break; // exit the loop if the HW context is created successfully
      } else if (!hw_config) {
        break; // exit the loop if no more HW config is available
      }
    }
  }

  // Open the decoder
  if (const int ret = avcodec_open2(codec_ctx_.get(), codec, nullptr); ret < 0) {
    throw Error("Decoder::Decoder(): Failed to open codec", ret);
  }
}

void Decoder::send_packet(const Packet &packet) {
  if (!codec_ctx_) {
    throw Error("Decoder::send_packet(): Decoder context is not configured");
  }

  if (const int ret = avcodec_send_packet(codec_ctx_.get(), packet.get()); ret < 0) {
    throw Error("Decoder::send_packet(): Error sending packet for decoding", ret);
  }
}

bool Decoder::receive_frame(Frame *const frame) {
  if (!codec_ctx_) {
    throw Error("Decoder::receive_frame(): Decoder context is not configured");
  }

  frame->unref();
  if (const int ret = avcodec_receive_frame(codec_ctx_.get(), frame->get()); ret == 0) {
    return true;
  } else if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
    // The decoder is in an OK state, but no frame is currently available
    return false;
  } else {
    throw Error("Decoder::receive_frame(): Error during decoding", ret);
  }
}

std::string Decoder::codec_name() const {
  // avcodec_get_name(AV_CODEC_ID_NONE) returns "none",
  // so std::string can be constructed without any problem.
  return avcodec_get_name(codec_ctx_ ? codec_ctx_->codec_id : AV_CODEC_ID_NONE);
}

std::string Decoder::hw_device_type() const {
  // av_hwdevice_get_type_name(AV_HWDEVICE_TYPE_NONE) returns nullptr,
  // so std::string CANNOT be constructed and std::logic_error is thrown.
  // To avoid this, return "none" in the case of no hardware.
  return (codec_ctx_ && codec_ctx_->hw_device_ctx)
             ? av_hwdevice_get_type_name(
                   reinterpret_cast<AVHWDeviceContext *>(codec_ctx_->hw_device_ctx->data)->type)
             : "none";
}

void Decoder::free_context(AVCodecContext *codec_ctx) { avcodec_free_context(&codec_ctx); }

// ==============================================
// Parser - RAII wrapper for AVCodecParserContext
// ==============================================

bool Parser::is_supported(const std::string &codec_name) const {
  const AVCodec *const codec = avcodec_find_decoder_by_name(codec_name.c_str());
  return codec && parser_ctx_ &&
         std::any_of(parser_ctx_->parser->codec_ids, parser_ctx_->parser->codec_ids + 7,
                     [codec](const int parser_id) { return parser_id == codec->id; });
}

void Parser::reconfigure(const std::string &codec_name) {
  // Find the codec by name
  const AVCodec *const codec = avcodec_find_decoder_by_name(codec_name.c_str());
  if (!codec) {
    throw Error("Parser::Parser(): " + codec_name + " was not recognized as a codec name");
  }

  // Initialize the parser with the codec
  parser_ctx_.reset(av_parser_init(codec->id));
  if (!parser_ctx_) {
    throw Error("Parser::Parser(): Failed to initialize parser for codec (" + codec_name + ")");
  }
}

int Parser::parse(const BufferRef &buffer, Decoder *const decoder, Packet *const packet,
                  const std::int64_t pos) {
  if (!parser_ctx_) {
    throw Error("Parser::parse(): Parser context is not configured");
  }

  // Parse the buffer starting from the pos-th byte
  std::uint8_t *packet_data;
  int packet_size;
  const int len =
      av_parser_parse2(parser_ctx_.get(), decoder->get(), &packet_data, &packet_size, buffer->data,
                       buffer.unpadded_size(), AV_NOPTS_VALUE, AV_NOPTS_VALUE, pos);

  // Set the packet content based on the parser's output
  packet->unref();
  if (packet_data && buffer->data + pos <= packet_data &&
      packet_data + packet_size <= buffer->data + buffer.unpadded_size()) {
    // If a packet is available and the data is within the input buffer,
    // refer to the entire buffer from the packet, and set the data to the packet
    packet->ref(buffer);
    (*packet)->data = packet_data;
    (*packet)->size = packet_size;
  } else if (packet_data) {
    // If a packet is available but the data is within the parser's internal buffer,
    // copy the data from the parser's internal buffer to a new buffer
    // to guarantee the lifetime of the data, and refer to it from the packet
    packet->ref(BufferRef(packet_data, packet_size));
  }

  return len;
}

std::vector<std::string> Parser::codec_names() const {
  std::vector<std::string> names;
  if (parser_ctx_) {
    for (const auto id : parser_ctx_->parser->codec_ids) {
      if (id != AV_CODEC_ID_NONE) {
        names.push_back(avcodec_get_name(static_cast<AVCodecID>(id)));
      }
    }
  }
  return names;
}

// =======================================
// Converter - RAII wrapper for SwsContext
// =======================================

bool Converter::is_supported(const std::size_t width, const std::size_t height,
                             const std::string &src_format_name,
                             const std::string &dst_format_name) const {
  return sws_ctx_ && (width == width_) && (height == height_) &&
         (av_get_pix_fmt(src_format_name.c_str()) == src_format_) &&
         (av_get_pix_fmt(dst_format_name.c_str()) == dst_format_);
}

void Converter::reconfigure(const std::size_t width, const std::size_t height,
                            const std::string &src_format_name,
                            const std::string &dst_format_name) {
  const AVPixelFormat src_format = av_get_pix_fmt(src_format_name.c_str()),
                      dst_format = av_get_pix_fmt(dst_format_name.c_str());
  sws_ctx_.reset(sws_getContext(
      // src & dst descriptions
      width, height, src_format, width, height, dst_format,
      // options for scaling and filtering (won't be used as we keep the same size)
      SWS_BILINEAR, nullptr, nullptr, nullptr));
  if (!sws_ctx_) {
    throw Error("Converter::reconfigure(): Failed to create SwsContext");
  }
  width_ = width;
  height_ = height;
  src_format_ = src_format;
  dst_format_ = dst_format;
}

void Converter::convert(const Frame &src_frame, std::vector<std::uint8_t> *const dst_data) {
  if (!sws_ctx_) {
    throw Error("Converter::convert(): Converter context is not configured");
  }

  // Get the layout of the destination image
  // - linesize: bytes per line for each plane
  std::array<int, 4> dst_linesize;
  if (const int ret = av_image_fill_linesizes(dst_linesize.data(), dst_format_, width_); ret < 0) {
    throw Error("Converter::convert(): Failed to get destination linesizes", ret);
  }
  // - plane size: bytes per plane
  std::array<std::size_t, 4> dst_plane_size;
  if (const int ret =
          av_image_fill_plane_sizes(dst_plane_size.data(), dst_format_, height_,
                                    std::array<std::ptrdiff_t, 4>{dst_linesize[0], dst_linesize[1],
                                                                  dst_linesize[2], dst_linesize[3]}
                                        .data());
      ret < 0) {
    throw Error("Converter::convert(): Failed to get destination plane sizes", ret);
  }
  // - data offset: bytes to the start of each plane
  std::array<std::ptrdiff_t, 4> dst_data_offset;
  std::partial_sum(dst_plane_size.begin(), dst_plane_size.end(), dst_data_offset.begin());

  // Resize the destination buffer to the sum of the plane sizes
  dst_data->resize(dst_data_offset[3]);

  // Convert the pixel data
  sws_scale(sws_ctx_.get(),
            // src description
            src_frame->data, src_frame->linesize, 0, src_frame->height,
            // dst description
            std::array<std::uint8_t *const, 4>{
                dst_data->data(), dst_data->data() + dst_data_offset[0],
                dst_data->data() + dst_data_offset[1], dst_data->data() + dst_data_offset[2]}
                .data(),
            dst_linesize.data());
}

std::string Converter::src_format_name() const {
  return av_get_pix_fmt_name(sws_ctx_ ? src_format_ : AV_PIX_FMT_NONE);
}

std::string Converter::dst_format_name() const {
  return av_get_pix_fmt_name(sws_ctx_ ? dst_format_ : AV_PIX_FMT_NONE);
}

} // namespace ffmpeg_cpp