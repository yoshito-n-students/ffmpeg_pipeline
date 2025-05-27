#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <ffmpeg_cpp/ffmpeg_cpp.hpp>

using namespace std::chrono_literals;
namespace av = ffmpeg_cpp;

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <input.wav>" << std::endl;
    return 1;
  }
  const std::string input_filename = argv[1];

  try {
    // Open the input file
    av::Input input = av::Input::create(argv[1], "", av::Dictionary::null(), "audio");
    std::cout << "Input device " << input_filename
              << " opened (codec: " << input.codec_parameters().codec_name() << ")" << std::endl;

    // Open the output device for the input codec parameters
    av::Output output = av::Output::create("pulse", "default", input.codec_parameters(), nullptr);
    std::cout << "Output device opened" << std::endl;

    // Read packets from the input file and play them on the output device
    while (true) {
      const av::Packet packet = input.read_frame();
      if (packet.empty()) {
        break;
      }

      while (!output.write_frame(packet)) {
        // write_frame() may return false if the internal buffer is full with previous frames.
        // In this case, wait a little for the previous frames to be processed and try again.
        // The duration of normal wav frames is about 23 ms = 1024 samples / 44100 Hz.
        std::this_thread::sleep_for(20ms);
      }
    }
  } catch (const std::runtime_error &error) {
    std::cerr << "Error: " << error.what() << std::endl;
    return 1;
  }

  return 0;
}