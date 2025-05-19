extern "C" {
#include <libavcodec/avcodec.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

#include "internal.hpp"

namespace ffmpeg_cpp {

// ==============================================
// Parser - RAII wrapper for AVCodecParserContext
// ==============================================

Parser Parser::null() { return Parser(nullptr); }

Parser Parser::create(const std::string &decoder_name) {
  // Find the decoder this parser is associated with
  const AVCodec *const decoder = avcodec_find_decoder_by_name(decoder_name.c_str());
  if (!decoder) {
    throw Error("Parser::create(): " + decoder_name + " was not recognized as a decoder name");
  }

  // Initialize the parser with the decoder's codec
  Parser parser(av_parser_init(decoder->id));
  if (!parser) {
    throw Error("Parser::create(): Failed to initialize parser for the decoder (" + decoder_name +
                ")");
  }

  // Initialize the codec context to accumulate the codec parameters
  parser.decoder_ctx_.reset(avcodec_alloc_context3(decoder));
  if (!parser.decoder_ctx_) {
    throw Error("Parser::create(): Failed to allocate decoder context for the decoder (" +
                decoder_name + ")");
  }

  return parser;
}

static Packet make_packet(const Packet &buffer, std::uint8_t *const packet_data,
                          const int packet_size) {
  if (buffer->buf->data <= packet_data &&
      packet_data + packet_size <= buffer->buf->data + buffer->buf->size) {
    // If the packet data is within the input buffer, refer to the buffer from the packet
    Packet packet = buffer;
    packet->data = packet_data;
    packet->size = packet_size;
    return packet;
  } else {
    // If the packet data is within the parser's internal buffer,
    // refer to the copy of the data to guarantee the lifetime of the data
    return Packet::create(packet_data, packet_size);
  }
}

std::pair<Packet, CodecParameters> Parser::parse_initial_packet(const Packet &buffer,
                                                                std::int64_t *const pos) {
  // Parse and advance the input buffer
  std::uint8_t *packet_data;
  int packet_size;
  *pos += av_parser_parse2(get(), decoder_ctx_.get(), &packet_data, &packet_size, //
                           buffer->data, buffer->size, buffer->pts, buffer->dts, *pos);

  // Return an empty packet and the default parameters
  // if no key-frame-equivalent packet is found
  if (!packet_data || get()->key_frame == 0) {
    return {Packet::null(), CodecParameters::null()};
  }

  // Construct the initial packet based on the parsed data
  const Packet packet = make_packet(buffer, packet_data, packet_size);

  // Copy the codec parameters from the codec context
  CodecParameters params = CodecParameters::create();
  if (const int ret = avcodec_parameters_from_context(params.get(), decoder_ctx_.get()); ret < 0) {
    throw Error(
        "Parser::parse_initial_packet(): Failed to copy codec parameters from the decoder context",
        ret);
  }

  return {std::move(packet), std::move(params)};
}

Packet Parser::parse_next_packet(const Packet &buffer, std::int64_t *const pos) {
  // Parse and advance the input buffer
  std::uint8_t *packet_data;
  int packet_size;
  *pos += av_parser_parse2(get(), decoder_ctx_.get(), &packet_data, &packet_size, //
                           buffer->data, buffer->size, buffer->pts, buffer->dts, *pos);

  // Return an empty packet if no packet data is found
  if (!packet_data) {
    return Packet::null();
  }

  // Construct the packet based on the parsed data
  return make_packet(buffer, packet_data, packet_size);
}

std::vector<std::string> Parser::codec_names() const {
  std::vector<std::string> names;
  if (get() && get()->parser) {
    for (const auto id : get()->parser->codec_ids) {
      if (id != AV_CODEC_ID_NONE) {
        names.push_back(avcodec_get_name(static_cast<AVCodecID>(id)));
      }
    }
  }
  return names;
}

} // namespace ffmpeg_cpp