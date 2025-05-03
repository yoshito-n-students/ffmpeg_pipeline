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

  // Initialize the codec context to accumulate the codec parameters
  codec_ctx_.reset(avcodec_alloc_context3(codec));
  if (!codec_ctx_) {
    throw Error("Parser::Parser(): Failed to allocate codec context for codec (" + codec_name +
                ")");
  }
  if (const int ret = avcodec_open2(codec_ctx_.get(), codec, nullptr); ret < 0) {
    throw Error("Parser::Parser(): Failed to open codec", ret);
  }
}

static Packet make_packet(const Packet &buffer, std::uint8_t *const packet_data,
                          const int packet_size) {
  if (buffer->buf->data <= packet_data &&
      packet_data + packet_size <= buffer->buf->data + buffer->buf->size) {
    // If the packet data is within the input buffer, refer to the buffer from the packet
    Packet packet(buffer);
    packet->data = packet_data;
    packet->size = packet_size;
    return packet;
  } else {
    // If the packet data is within the parser's internal buffer,
    // refer to the copy of the data to guarantee the lifetime of the data
    return Packet(packet_data, packet_size);
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
    return {Packet(), CodecParameters()};
  }

  // Construct the initial packet based on the parsed data
  const Packet packet = make_packet(buffer, packet_data, packet_size);

  // Decode the initial packet to get more information about the codec parameters
  if (const int ret = avcodec_send_packet(codec_ctx_.get(), packet.get()); ret < 0) {
    throw Error("Parser::parse_initial_packet(): Error sending packet for decoding", ret);
  }
  while (true) {
    Frame frame;
    if (const int ret = avcodec_receive_frame(codec_ctx_.get(), frame.get()); ret >= 0) {
      continue; // A frame was received, continue to receive more frames
    } else if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      break; // No more frames to receive, break the loop
    } else {
      throw Error("Parser::parse_initial_packet(): Error during decoding", ret);
    }
  }

  // Copy the codec parameters from the codec context
  CodecParameters params;
  if (const int ret = avcodec_parameters_from_context(params.get(), codec_ctx_.get()); ret < 0) {
    throw Error("Parser::parse_initial_packet(): Failed to copy codec parameters from context", ret);
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
    return Packet();
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