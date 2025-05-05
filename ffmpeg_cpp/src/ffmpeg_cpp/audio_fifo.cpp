#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

#include "internal.hpp"

namespace ffmpeg_cpp {
// ========================================
// AudioFifo - RAII wrapper for AVAudioFifo
// ========================================

AudioFifo::AudioFifo() {}

AudioFifo::AudioFifo(const std::string &ch_layout_str, const std::string &format_name,
                     const int sample_rate)
    : AudioFifo() {
  // Convert the given strings to data types
  const AVChannelLayout ch_layout = to_channel_layout(ch_layout_str);
  const AVSampleFormat format = av_get_sample_fmt(format_name.c_str());

  // Fill the template frame
  template_frame_ = Frame::create();
  av_channel_layout_copy(&template_frame_->ch_layout, &ch_layout);
  template_frame_->format = format;
  template_frame_->sample_rate = sample_rate;

  // Allocate the AVAudioFifo
  reset(av_audio_fifo_alloc(format, ch_layout.nb_channels, 1));
  if (!get()) {
    throw Error("AudioFifo::AudioFifo(): Failed to allocate AVAudioFifo");
  }
}

std::string AudioFifo::ch_layout_str() const { return to_string(template_frame_->ch_layout); }

std::string AudioFifo::format_name() const {
  return to_string(static_cast<AVSampleFormat>(template_frame_->format));
}

int AudioFifo::sample_rate() const { return template_frame_->sample_rate; }

void AudioFifo::write(const Frame &frame) {
  if (const int ret = av_audio_fifo_write(get(), reinterpret_cast<void *const *>(frame->data),
                                          frame->nb_samples);
      ret < 0) {
    throw Error("AudioFifo::write(): Failed to write to FIFO", ret);
  }
}

Frame AudioFifo::read(const int nb_samples) {
  // Prepare the empty frame with the same parameters as the template frame.
  // We do not copy parameters by Frame::Frame(const Frame&)
  // because the constructor copies the buffer as well.
  Frame frame = Frame::create();
  av_channel_layout_copy(&frame->ch_layout, &template_frame_->ch_layout);
  frame->format = template_frame_->format;
  frame->sample_rate = template_frame_->sample_rate;

  // Return the empty frame if the number of samples in the FIFO is less than required
  if (av_audio_fifo_size(get()) < nb_samples) {
    return frame;
  }

  // Allocate the frame data for nb_samples samples
  frame->nb_samples = nb_samples;
  if (const int ret = av_frame_get_buffer(frame.get(), 0); ret < 0) {
    throw Error("AudioFifo::read(): Failed to allocate frame buffer", ret);
  }

  // Take the data from the FIFO and fill the frame
  if (const int ret = av_audio_fifo_read(get(), reinterpret_cast<void **>(frame->data), nb_samples);
      ret < 0) {
    throw Error("AudioFifo::read(): Failed to read from FIFO", ret);
  }

  return frame;
}

} // namespace ffmpeg_cpp