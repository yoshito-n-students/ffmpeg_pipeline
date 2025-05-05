extern "C" {
#include <libavcodec/avcodec.h>
}

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

namespace ffmpeg_cpp {

// ==============================================
// Parser - RAII wrapper for AVCodecParserContext
// ==============================================

Parser Parser::null() { return Parser(nullptr); }

Parser Parser::create(const std::string &codec_name) {
  // Find the codec by name
  const AVCodec *const codec = [&codec_name]() {
    void *iterate_ctx = nullptr;
    while (true) {
      if (const AVCodec *const codec = av_codec_iterate(&iterate_ctx);
          codec && codec->name == codec_name) {
        return codec;
      } else if (!codec) {
        throw Error("Parser::create(): " + codec_name + " was not recognized as a codec name");
      }
    }
  }();

  // Initialize the parser with the codec
  Parser parser(av_parser_init(codec->id));
  if (!parser) {
    throw Error("Parser::create(): Failed to initialize parser for codec (" + codec_name + ")");
  }

  // Initialize the codec context to accumulate the codec parameters
  parser.codec_ctx_.reset(avcodec_alloc_context3(codec));
  if (!parser.codec_ctx_) {
    throw Error("Parser::create(): Failed to allocate codec context for codec (" + codec_name +
                ")");
  }

  // TODO: Remove this if it works
  if (const int ret = avcodec_open2(parser.codec_ctx_.get(), codec, nullptr); ret < 0) {
    throw Error("Parser::create(): Failed to open codec", ret);
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
  *pos += av_parser_parse2(get(), codec_ctx_.get(), &packet_data, &packet_size, //
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
  if (const int ret = avcodec_parameters_from_context(params.get(), codec_ctx_.get()); ret < 0) {
    throw Error("Parser::parse_initial_packet(): Failed to copy codec parameters from context",
                ret);
  }

  return {packet, params};
}

Packet Parser::parse_next_packet(const Packet &buffer, std::int64_t *const pos) {
  // Parse and advance the input buffer
  std::uint8_t *packet_data;
  int packet_size;
  *pos += av_parser_parse2(get(), codec_ctx_.get(), &packet_data, &packet_size, //
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
  for (const auto id : get()->parser->codec_ids) {
    if (id != AV_CODEC_ID_NONE) {
      names.push_back(avcodec_get_name(static_cast<AVCodecID>(id)));
    }
  }
  return names;
}

} // namespace ffmpeg_cpp