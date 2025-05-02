extern "C" {
#include <libavcodec/avcodec.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_cpp {

// ==============================================
// Parser - RAII wrapper for AVCodecParserContext
// ==============================================

Parser::Parser(const std::string &codec_name)
    : parser_ctx_(nullptr, &av_parser_close), codec_ctx_(nullptr, &free_context) {
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

  // Allocate the codec context
  codec_ctx_.reset(avcodec_alloc_context3(codec));
  if (!codec_ctx_) {
    throw Error("Parser::Parser(): Failed to allocate codec context for codec (" + codec_name +
                ")");
  }
}

Packet Parser::parse(BufferRef *const buffer, Decoder *const decoder) {
  // Parse and advance the buffer
  std::uint8_t *packet_data;
  int packet_size;
  const std::uint8_t *const buffer_data = (*buffer)->data;
  const int buffer_size = buffer->unpadded_size();
  const int len = av_parser_parse2(parser_ctx_.get(), decoder->get(), &packet_data, &packet_size,
                                   buffer_data, buffer_size, AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
  (*buffer)->data += len;
  (*buffer)->size -= len;

  // Set the packet content based on the parser's output
  if (packet_data) {
    if (buffer_data <= packet_data && packet_data + packet_size <= buffer_data + buffer_size) {
      // If a packet is available and the data is within the input buffer,
      // refer to the entire buffer from the packet, and set the data to the packet
      Packet packet(*buffer);
      packet->data = packet_data;
      packet->size = packet_size;
      return packet;
    } else {
      // If a packet is available but the data is within the parser's internal buffer,
      // copy the data from the parser's internal buffer to a new buffer
      // to guarantee the lifetime of the data, and refer to it from the packet
      return Packet(BufferRef(packet_data, packet_size));
    }
  } else {
    // If no packet is available, create an empty packet
    return Packet();
  }
}

std::pair<Packet, CodecParameters> Parser::parse_initial_packet(Packet *const buffer) {
  // Parse and advance the input buffer
  std::uint8_t *packet_data;
  int packet_size;
  const int len =
      av_parser_parse2(parser_ctx_.get(), codec_ctx_.get(), &packet_data, &packet_size,
                       (*buffer)->data, (*buffer)->size, (*buffer)->pts, (*buffer)->dts, 0);
  (*buffer)->data += len;
  (*buffer)->size -= len;

  // Return an empty packet and the default parameters
  // if no key-frame-equivalent packet is found
  if (!packet_data || parser_ctx_->key_frame == 0) {
    return {Packet(), CodecParameters()};
  }

  // Copy the codec parameters from the codec context
  CodecParameters params;
  if (const int ret = avcodec_parameters_from_context(params.get(), codec_ctx_.get()); ret < 0) {
    throw Error("Parser::parse(): Failed to copy codec parameters from context", ret);
  }

  // Set the packet content based on the parser's output
  if ((*buffer)->buf->data <= packet_data &&
      packet_data + packet_size <= (*buffer)->buf->data + (*buffer)->buf->size) {
    // If the packet data is within the input buffer, refer to the buffer from the packet
    Packet packet(*buffer);
    packet->data = packet_data;
    packet->size = packet_size;
    return {packet, params};
  } else {
    // If the packet data is within the parser's internal buffer,
    // refer to the copy of the data to guarantee the lifetime of the data
    return {Packet(packet_data, packet_size), params};
  }
}

Packet Parser::parse_next_packet(Packet *const buffer) {
  // Parse and advance the input buffer
  std::uint8_t *packet_data;
  int packet_size;
  const int len =
      av_parser_parse2(parser_ctx_.get(), codec_ctx_.get(), &packet_data, &packet_size,
                       (*buffer)->data, (*buffer)->size, (*buffer)->pts, (*buffer)->dts, 0);
  (*buffer)->data += len;
  (*buffer)->size -= len;

  // Return an empty packet if no packet data is found
  if (!packet_data) {
    return Packet();
  }

  // Set the packet content based on the parser's output
  if ((*buffer)->buf->data <= packet_data &&
      packet_data + packet_size <= (*buffer)->buf->data + (*buffer)->buf->size) {
    // If the packet data is within the input buffer, refer to the buffer from the packet
    Packet packet(*buffer);
    packet->data = packet_data;
    packet->size = packet_size;
    return packet;
  } else {
    // If the packet data is within the parser's internal buffer,
    // refer to the copy of the data to guarantee the lifetime of the data
    return Packet(packet_data, packet_size);
  }
}

std::vector<std::string> Parser::codec_names() const {
  std::vector<std::string> names;
  for (const auto id : parser_ctx_->parser->codec_ids) {
    if (id != AV_CODEC_ID_NONE) {
      names.push_back(avcodec_get_name(static_cast<AVCodecID>(id)));
    }
  }
  return names;
}

void Parser::free_context(AVCodecContext *codec_ctx) { avcodec_free_context(&codec_ctx); }

} // namespace ffmpeg_cpp