extern "C" {
#include <libavcodec/avcodec.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_cpp {

// ==============================================
// Parser - RAII wrapper for AVCodecParserContext
// ==============================================

Parser::Parser() {}

Parser::Parser(const std::string &codec_name) : Parser() {
  // Find the codec by name
  const AVCodec *const codec = avcodec_find_decoder_by_name(codec_name.c_str());
  if (!codec) {
    throw Error("Parser::Parser(): " + codec_name + " was not recognized as a codec name");
  }

  // Initialize the parser with the codec
  reset(av_parser_init(codec->id));
  if (!get()) {
    throw Error("Parser::Parser(): Failed to initialize parser for codec (" + codec_name + ")");
  }

  // Allocate the codec context
  codec_ctx_.reset(avcodec_alloc_context3(codec));
  if (!codec_ctx_) {
    throw Error("Parser::Parser(): Failed to allocate codec context for codec (" + codec_name +
                ")");
  }
}

std::pair<Packet, CodecParameters> Parser::parse_initial_packet(Packet *const buffer) {
  // Parse and advance the input buffer
  std::uint8_t *packet_data;
  int packet_size;
  const int len =
      av_parser_parse2(get(), codec_ctx_.get(), &packet_data, &packet_size, (*buffer)->data,
                       (*buffer)->size, (*buffer)->pts, (*buffer)->dts, 0);
  (*buffer)->data += len;
  (*buffer)->size -= len;

  // Return an empty packet and the default parameters
  // if no key-frame-equivalent packet is found
  if (!packet_data || get()->key_frame == 0) {
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
      av_parser_parse2(get(), codec_ctx_.get(), &packet_data, &packet_size, (*buffer)->data,
                       (*buffer)->size, (*buffer)->pts, (*buffer)->dts, 0);
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
  for (const auto id : get()->parser->codec_ids) {
    if (id != AV_CODEC_ID_NONE) {
      names.push_back(avcodec_get_name(static_cast<AVCodecID>(id)));
    }
  }
  return names;
}

} // namespace ffmpeg_cpp